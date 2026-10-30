/* 复用CARCAN13已核对CAN链；本阶段仅改名、归类和增加接收序号。 */
#include "bus_internal.h"
#include "bus_board.h"
#include "bus_config.h"
#include <string.h>
volatile BusDiag bus_diag;
volatile BusTimeout bus_timeout;
enum
{
    JOB_NONE,
    JOB_DIAG,
    JOB_SPEED,
    JOB_QUERY,
    JOB_STOP,
    JOB_FINITE
};
static uint8_t addresses[4], valid_addresses;
static BusWheel wheels[4];
static MotorConfigRx config_rx[4];
static BusFrame inbox[CAN_RING_SIZE], diagnostic[2];
static uint8_t in_head, in_tail, in_count, diagnostic_count;
static uint8_t diagnostic_owner, diagnostic_pending, cancelling;
static CanState diagnostic_state;
static uint8_t job, job_wheel, batch, batch_wheel, latest, query_index, prefer_query;
static uint8_t stop_bits[32], stop_address, four_stop;
static uint16_t stop_cursor = 1;
static int16_t latest_rpm[4], batch_rpm[4];
/* 最新快照/当前批次是四轮FD位置小步而非速度；与速度互相替换，不并存。 */
static uint8_t latest_position, batch_position;
static uint8_t latest_dir[4], batch_dir[4];
static uint16_t latest_position_rpm, batch_position_rpm;
static uint32_t latest_pulses[4], batch_pulses[4];
static uint32_t batch_sent_tick[4];
static uint32_t latest_sequence, batch_sequence, batch_tick, query_tick, stop_tick;
static uint32_t seen_errors, seen_overflow;
static uint8_t waiting, wait_reported;
static uint32_t wait_tick;
static uint32_t cancel_tick;
static uint32_t reply_barrier, reply_start, reply_tick;
static uint8_t reply_wait, reply_matched, reply_address, reply_code;
static void (*observer)(const BusFrame *frame);
static MotorConfigRx query_config_rx;
static MotorConfig query_config;

void Bus_Observe(void (*receive)(const BusFrame *frame))
{
    observer = receive;
}
/* finite_owned直到人工确认后的BusRearm才清除；不能中途换地址或续发运动。 */
static uint8_t finite_owned, finite_pending, finite_wheel;
static uint8_t mech_watch;
static uint8_t hold_watch, hold_query_index;
static uint32_t hold_tx_tick, hold_rx_tick;
static uint8_t hold_tx_valid, hold_rx_valid;
static uint32_t hold_sample_tick[20];
static uint8_t hold_sample_valid[20];

void Bus_HoldWatch(uint8_t enabled)
{
    hold_watch = (uint8_t)!!enabled;
    hold_query_index = 0;
    hold_tx_valid = hold_rx_valid = 0;
}
static uint8_t mech_group_grant;
static uint8_t finite_address[4], finite_dir[4];
static uint16_t finite_rpm;
static uint32_t finite_pulses[4];
static void Depth(void)
{
    uint32_t count = diagnostic_pending ? 1U : 0U;
    if (latest)
        count += 4;
    if (batch)
        count += (uint32_t)(4 - batch_wheel);
    if (finite_pending)
        count += (uint32_t)(4 - finite_wheel);
    count += bus_diag.pending_stop_count;
    bus_diag.queue_depth = count;
}
static void ClearMotion(void)
{
    mech_group_grant = 0;
    latest = 0;
    batch = 0;
    batch_wheel = 0;
    finite_pending = 0;
    latest_position = 0;
    batch_position = 0;
    memset(latest_rpm, 0, sizeof(latest_rpm));
    memset(batch_rpm, 0, sizeof(batch_rpm));
}
static void StopMask(void)
{
    uint8_t i;
    bus_diag.stop_pending = 0;
    for (i = 0; i < 4; i++)
        if (addresses[i] && (stop_bits[addresses[i] / 8U] & (1U << (addresses[i] % 8U))))
            bus_diag.stop_pending |= (uint8_t)(1U << i);
}
static void Retain(uint8_t address)
{
    uint8_t mask = (uint8_t)(1U << (address % 8U));
    if (!address)
        return;
    if (!(stop_bits[address / 8U] & mask))
    {
        stop_bits[address / 8U] |= mask;
        bus_diag.pending_stop_count++;
        bus_diag.stop_sequence++;
    }
    StopMask();
}
static void KeepStops(void)
{
    uint8_t i;
    for (i = 0; i < 4; i++)
        Retain(finite_owned ? finite_address[i] : addresses[i]);
    if (mech_watch)
    {
        Retain(5);
        Retain(6);
    }
}
void Bus_MechWatch(void)
{
    mech_watch = 1;
}

uint8_t Bus_MechDone(void)
{
    /* 调用方已确认轴到位/停止；不清故障、不丢反馈、不重新使能。 */
    if (bus_diag.error || bus_diag.locked || bus_diag.active || diagnostic_owner || job ||
        cancelling || bus_diag.pending_stop_count || finite_owned)
        return 0;
    bus_diag.motion_uncertain = 0;
    mech_group_grant = 0;
    return 1;
}
uint8_t Bus_MechGroup(uint8_t axes)
{
    if (axes > 3 || can_diag.errors || can_diag.rx_overflow || bus_diag.rx_overflow ||
        bus_diag.error || bus_diag.locked || bus_diag.active || finite_owned ||
        bus_diag.motion_uncertain || bus_diag.pending_stop_count || batch || latest)
        return 0;
    mech_group_grant = axes;
    return 1;
}
static void Fault(uint32_t error)
{
    uint8_t moving = (uint8_t)(bus_diag.active || batch || four_stop || finite_owned ||
                              (mech_watch && bus_diag.motion_uncertain));
    bus_diag.error |= error;
    bus_diag.locked = 1;
    bus_diag.active = 0;
    ClearMotion();
    if (moving)
    {
        four_stop = 1;
        KeepStops();
    }
    if (finite_owned)
        diagnostic_state = CAN_FAILED;
    /* 位置和速度命令失败后不重放，Stop 单独保留。 */
    if (diagnostic_pending && diagnostic[0].data[0] != 0xFE)
        diagnostic_pending = 0;
}
void Bus_Setup(const uint8_t address[4])
{
    uint8_t i, j;
    if (bus_diag.active || bus_diag.locked || diagnostic_owner || job || cancelling ||
        bus_diag.pending_stop_count || batch || finite_owned)
    {
        bus_diag.rejected++;
        return;
    }
    memset(addresses, 0, sizeof(addresses));
    if (address)
        memcpy(addresses, address, 4);
    valid_addresses = 1;
    for (i = 0; i < 4; i++)
    {
        if (!addresses[i])
            valid_addresses = 0;
        for (j = 0; j < i; j++)
            if (addresses[j] == addresses[i])
                valid_addresses = 0;
    }
    memset(wheels, 0, sizeof(wheels));
    memset(config_rx, 0, sizeof(config_rx));
    query_index = 0;
    query_tick = HAL_GetTick();
    prefer_query = 0;
}
int Bus_Start(void)
{
    if (!valid_addresses || bus_diag.active || bus_diag.locked || bus_diag.motion_uncertain ||
        diagnostic_owner || job || cancelling || batch || finite_owned || bus_diag.pending_stop_count ||
        can_diag.tx_pending || can_diag.tx_isolated || can_diag.free_mailboxes != 3 ||
        Can_Status() == CAN_BUSY || Can_Status() == CAN_CANCELLING)
    {
        bus_diag.rejected++;
        return 0;
    }
    /* 段切换不能总从FL重来：旧反馈仍有年龄，先刷新最老项。
     * 这样短段后接长段也不会把RR旧样本拖出80ms门禁。 */
    {
        uint32_t now = HAL_GetTick(), oldest = 0, age;
        uint8_t n, next = query_index;
        for (n = 0; n < 8; n++)
        {
            MotorReply *reply = (n & 1U) ? &wheels[n / 2U].position : &wheels[n / 2U].speed;
            age = reply->valid ? now - reply->tick : UINT32_MAX;
            if (age > oldest)
            {
                oldest = age;
                next = n;
            }
        }
        query_index = next;
        query_tick = now - BUS_QUERY_INTERVAL_MS;
    }
    bus_diag.active = 1;
    prefer_query = hold_watch ? 8U : BUS_QUERIES_PER_BATCH;
    return 1;
}
void Bus_End(void)
{
    bus_diag.active = 0;
    latest = 0;
    /* 已开始的四轮批次完整发完，随后不再查询。 */
    Depth();
}
int Bus_Speed(const int16_t rpm[4])
{
    uint8_t i;
    if (!rpm || !bus_diag.active || bus_diag.locked)
    {
        bus_diag.rejected++;
        return 0;
    }
    for (i = 0; i < 4; i++)
        if (rpm[i] < -(int32_t)MOTOR_MAX_COMMAND_RPM || rpm[i] > (int32_t)MOTOR_MAX_COMMAND_RPM)
        {
            bus_diag.rejected++;
            return 0;
        }
    if (latest)
        bus_diag.replaced++;
    memcpy(latest_rpm, rpm, sizeof(latest_rpm));
    latest_position = 0;
    latest = 1;
    latest_sequence = ++bus_diag.speed_sequence;
    Depth();
    return 1;
}
int Bus_Position(const uint8_t dir[4], uint16_t rpm, const uint32_t pulses[4])
{
    uint8_t i;
    if (!dir || !pulses || !rpm || rpm > MOTOR_MAX_COMMAND_RPM || !bus_diag.active || bus_diag.locked)
    {
        bus_diag.rejected++;
        return 0;
    }
    for (i = 0; i < 4; i++)
        if (dir[i] > 1 || !pulses[i])
        {
            bus_diag.rejected++;
            return 0;
        }
    if (latest)
        bus_diag.replaced++;
    memcpy(latest_dir, dir, sizeof(latest_dir));
    memcpy(latest_pulses, pulses, sizeof(latest_pulses));
    latest_position_rpm = rpm;
    latest_position = 1;
    latest = 1;
    latest_sequence = ++bus_diag.speed_sequence;
    Depth();
    return 1;
}
static uint8_t BuildBatch(uint8_t wheel, BusFrame frames[2])
{
    uint16_t rpm;
    if (batch_position)
        return Drive_Build(addresses[wheel], 0xFD, batch_dir[wheel], batch_position_rpm, 0,
                           batch_pulses[wheel], 2, 0, frames);
    rpm = (uint16_t)(batch_rpm[wheel] < 0 ? -batch_rpm[wheel] : batch_rpm[wheel]);
    return Drive_Build(addresses[wheel], 0xF6, (uint8_t)(batch_rpm[wheel] < 0), rpm, 0, 0, 0, 0, frames);
}
void Bus_Cancel(void)
{
    reply_wait = reply_matched = 0;
    bus_diag.query_waiting = 0;
    ClearMotion();
    if (bus_diag.active || finite_owned)
    {
        bus_diag.active = 0;
        bus_diag.locked = 1;
        bus_diag.error |= BUS_ERROR_STOP;
        four_stop = 1;
        KeepStops();
    }
    if (diagnostic_pending && diagnostic[0].data[0] != 0xFE)
        diagnostic_pending = 0;
    diagnostic_state = CAN_CANCELLING;
    job = JOB_NONE;
    cancelling = 1;
    bus_diag.cancel_count++;
    cancel_tick = HAL_GetTick();
    waiting = 0;
    wait_reported = 0;
    Can_Cancel();
    Depth();
}
void Bus_Stop(void)
{
    bus_diag.locked = 1;
    bus_diag.active = 0;
    bus_diag.error |= BUS_ERROR_STOP;
    stop_tick = HAL_GetTick() - BUS_STOP_RETRY_MS;
    four_stop = 1;
    KeepStops();
    Bus_Cancel();
}
int Bus_Rearm(void)
{
    if ((bus_diag.error & ~BUS_ERROR_STOP) || can_diag.errors || can_diag.rx_overflow || bus_diag.rx_overflow ||
        bus_diag.active || batch || latest || bus_diag.pending_stop_count || cancelling || diagnostic_owner ||
        job || can_diag.tx_pending || can_diag.tx_isolated || can_diag.free_mailboxes != 3 ||
        Can_Status() == CAN_BUSY || Can_Status() == CAN_CANCELLING)
    {
        bus_diag.rejected++;
        return 0;
    }
    ClearMotion();
    bus_diag.active = 0;
    bus_diag.locked = 0;
    bus_diag.error = 0;
    four_stop = 0;
    stop_address = 0;
    finite_owned = 0;
    bus_diag.motion_uncertain = 0;
    diagnostic_state = CAN_IDLE;
    bus_diag.ack_observed_mask = 0;
    Bus_Drop();
    memset(wheels, 0, sizeof(wheels));
    memset(config_rx, 0, sizeof(config_rx));
    Depth();
    return 1;
}
uint8_t Bus_Send(const BusFrame *frames, uint8_t count)
{
    uint8_t i, stop, move, enable, group_bit = 0;
    if (!frames || !count || count > 2)
    {
        bus_diag.rejected++;
        return 0;
    }
    for (i = 0; i < count; i++)
        if (frames[i].len < 2 || frames[i].len > 8 || !(frames[i].id >> 8) || (frames[i].id >> 8) > 255 ||
            (frames[i].id & 255) != i ||
            (i && ((frames[i].id >> 8) != (frames[0].id >> 8) || frames[i].data[0] != frames[0].data[0])))
        {
            bus_diag.rejected++;
            return 0;
        }
    stop = (uint8_t)(frames[0].data[0] == 0xFE);
    if (stop && count != 1)
    {
        bus_diag.rejected++;
        return 0;
    }
    move = (uint8_t)(frames[0].data[0] == 0xF6 || frames[0].data[0] == 0xFD);
    if (frames[0].data[0] == 0xFD && count == 2 && (frames[0].id >> 8) >= 5 && (frames[0].id >> 8) <= 6)
        group_bit = (uint8_t)(mech_group_grant & (1U << ((frames[0].id >> 8) - 5)));
    enable = (uint8_t)(frames[0].data[0] == 0xF3 && frames[0].len > 2 && frames[0].data[2]);
    if (!stop && (diagnostic_owner || diagnostic_pending || bus_diag.active || batch || job || cancelling ||
                  bus_diag.pending_stop_count))
    {
        bus_diag.rejected++;
        return 0;
    }
    if ((move || enable) && ((bus_diag.motion_uncertain && !group_bit) || bus_diag.locked || finite_owned ||
                            (mech_group_grant && !group_bit)))
    {
        bus_diag.rejected++;
        return 0;
    }
    if (stop && (bus_diag.active || batch || finite_owned))
        Bus_Stop();
    else if (stop && job == JOB_DIAG)
        Bus_Cancel();
    memcpy(diagnostic, frames, count * sizeof(*frames));
    diagnostic_count = count;
    diagnostic_owner = 1;
    diagnostic_pending = (uint8_t)!stop;
    diagnostic_state = CAN_BUSY;
    if (move)
    {
        bus_diag.motion_uncertain = 1;
        mech_group_grant &= (uint8_t)~group_bit;
    }
    if (stop)
    {
        stop_address = (uint8_t)(frames[0].id >> 8);
        Retain(stop_address);
        bus_diag.locked = 1;
        bus_diag.error |= BUS_ERROR_STOP;
        stop_tick = HAL_GetTick() - BUS_STOP_RETRY_MS;
    }
    Depth();
    return 1;
}
uint8_t Bus_Finite(const uint8_t dir[4], uint16_t rpm, const uint32_t pulses[4])
{
    uint8_t selected_dir[4], i;
    uint32_t selected_pulses[4];
    CanState state = Can_Status();
    if (!dir || !pulses || !rpm || rpm > MOTOR_MAX_COMMAND_RPM || !valid_addresses || diagnostic_owner ||
        diagnostic_pending || bus_diag.active || batch || latest || job || cancelling || finite_owned ||
        bus_diag.queue_depth || bus_diag.pending_stop_count || bus_diag.motion_uncertain || bus_diag.locked ||
        bus_diag.error || can_diag.tx_pending || can_diag.tx_isolated || can_diag.free_mailboxes != 3 ||
        state == CAN_BUSY || state == CAN_CANCELLING)
    {
        bus_diag.rejected++;
        return 0;
    }
    memcpy(selected_dir, dir, sizeof(selected_dir));
    memcpy(selected_pulses, pulses, sizeof(selected_pulses));
    for (i = 0; i < 4; i++)
        if (selected_dir[i] > 1 || !selected_pulses[i])
        {
            bus_diag.rejected++;
            return 0;
        }
    /* 校验全部通过后一次接纳；第一帧前即锁定运动不确定性，逐轮不解锁。 */
    memcpy(finite_address, addresses, sizeof(finite_address));
    memcpy(finite_dir, selected_dir, sizeof(finite_dir));
    memcpy(finite_pulses, selected_pulses, sizeof(finite_pulses));
    finite_rpm = rpm;
    finite_wheel = 0;
    finite_pending = 1;
    finite_owned = 1;
    diagnostic_owner = 1;
    diagnostic_state = CAN_BUSY;
    stop_address = 0;
    bus_diag.motion_uncertain = 1;
    bus_diag.finite_sequence++;
    bus_diag.finite_sent_mask = 0;
    Depth();
    return 1;
}
CanState Bus_Status(void)
{
    return diagnostic_state;
}
void Bus_Release(void)
{
    /* 回包等待阶段仍独占；调用者完成或失败后才释放。 */
    if (finite_pending || job == JOB_FINITE ||
        ((diagnostic_pending || job == JOB_DIAG) && diagnostic[0].data[0] != 0xFE))
        Bus_Cancel();
    diagnostic_owner = 0;
}
uint8_t Bus_Read(BusFrame *frame)
{
    if (!frame || !in_count)
        return 0;
    *frame = inbox[in_tail];
    in_tail = (uint8_t)((in_tail + 1U) % CAN_RING_SIZE);
    in_count--;
    return 1;
}
void Bus_Drop(void)
{
    in_head = 0;
    in_tail = 0;
    in_count = 0;
    Can_Drop();
}
const BusWheel *Bus_Wheels(void)
{
    return wheels;
}
static void Receive(void)
{
    BusFrame frame;
    MotorReply reply;
    uint8_t i, n, finite_target;
    for (n = 0; n < CAN_RING_SIZE && Can_Read(&frame); n++)
    {
        uint8_t address = (uint8_t)(frame.id >> 8);
        if (address >= 1 && address <= 6 && Drive_Parse(&frame, address, &reply))
        {
            uint8_t channel = reply.code == 0x35 ? 0 : reply.code == 0x36 ? 1 :
                              reply.code == 0x3A ? 2 : reply.code == 0x33 ? 3 : 255;
            if ((address <= 4 && channel < 3) || (address >= 5 && channel < 4))
            {
                uint8_t slot = address <= 4 ? (uint8_t)((address - 1U) * 3U + channel) :
                    (uint8_t)(12U + (address - 5U) * 4U + channel);
                hold_sample_tick[slot] = reply.tick;
                hold_sample_valid[slot] = 1;
            }
        }
        if (hold_watch && (frame.id >> 8) >= 5 && (frame.id >> 8) <= 6)
        {
            hold_rx_tick = frame.tick;
            hold_rx_valid = 1;
        }
        if (observer)
            observer(&frame);
        /* RX时必须已有当前事务真实TXOK证据；不比较RX与延迟执行的TX回调时间。
         * 实际发送前、无TXOK、旧事务、错地址/命令及期限外回包均不解锁。
         * 匹配只记录回复，Complete仍等待正式HAL发送完成状态。 */
        if (reply_wait && !reply_matched && frame.rx_sequence != reply_barrier &&
            (uint32_t)(frame.rx_sequence - reply_barrier) < 0x80000000UL &&
            frame.tx_sequence == bus_diag.query_tx_sequence &&
            (uint32_t)(frame.tick - reply_start) < BUS_REPLY_TIMEOUT_MS &&
            ((reply_code == 0x42 && Drive_Config(&query_config_rx, &frame, reply_address, &query_config) == 1) ||
             (Drive_Parse(&frame, reply_address, &reply) && reply.code == reply_code)))
        {
            reply_matched = 1;
            reply_tick = frame.tick;
            bus_diag.query_reply_tick = frame.tick;
            bus_diag.query_replies++;
        }
        finite_target = 0;
        if (finite_owned)
            for (i = 0; i < 4; i++)
                if ((frame.id >> 8) == finite_address[i])
                    finite_target = 1;
        /* 遥测直接更新wheels；只有独占诊断事务需要保留原始分包。
         * 整车执行器不消费诊断inbox，不能复制全部遥测并制造假溢出。 */
        /* 批次释放查询owner后仍保留本组FE原始应答，先入队再处理负应答。 */
        if (diagnostic_owner || (finite_target && frame.len && frame.data[0] == 0xFE))
        {
            if (in_count < CAN_RING_SIZE)
            {
                inbox[in_head] = frame;
                in_head = (uint8_t)((in_head + 1U) % CAN_RING_SIZE);
                in_count++;
            }
            else
            {
                bus_diag.rx_overflow++;
                if (finite_owned && !(bus_diag.error & BUS_ERROR_RX))
                {
                    Fault(BUS_ERROR_RX);
                    Bus_Cancel();
                }
                else
                    bus_diag.error |= BUS_ERROR_RX;
            }
        }
        for (i = 0; i < 4; i++)
            if (addresses[i] && (frame.id >> 8) == addresses[i])
            {
                if (frame.len && frame.data[0] == 0x42)
                {
                    if (!(frame.id & 255U))
                        wheels[i].config.valid = 0;
                    if (Drive_Config(&config_rx[i], &frame, addresses[i], &wheels[i].config) == 2)
                        wheels[i].config.valid = 0;
                }
                else if (Drive_Parse(&frame, addresses[i], &reply))
                {
                    /* 先取出完整帧，再记录处理时刻，避免跨毫秒后处理时间早于接收。 */
                    reply.handled_ms = HAL_GetTick();
                    if (reply.code == 0x35)
                        wheels[i].speed = reply;
                    else if (reply.code == 0x36)
                        wheels[i].position = reply;
                    else if (reply.code == 0x3A)
                        wheels[i].status = reply;
                    else
                        bus_diag.ack_observed_mask |= (uint8_t)(1U << i);
                    /* ACK 无事务号，只观察；不改变速度批次或运动完成状态。 */
                    if (((reply.code == 0xF3 || reply.code == 0xF6 || reply.code == 0xFD) &&
                         (bus_diag.active || batch) && (reply.result == 0xE2 || reply.result == 0xEE)) ||
                        (finite_target &&
                         (reply.code == 0xF3 || reply.code == 0xF6 || reply.code == 0xFD ||
                          reply.code == 0xFE) &&
                         (reply.result == 0xE2 || reply.result == 0xEE || reply.result == 0x12 ||
                          reply.result == 0x22)))
                    {
                        Fault(BUS_ERROR_DRIVER);
                        Bus_Cancel();
                    }
                }
            }
    }
}
static int Submit(uint8_t type, uint8_t wheel, const BusFrame *frames, uint8_t count)
{
    uint32_t now = HAL_GetTick();
    if (!Can_Send(frames, count))
    {
        if (!waiting)
        {
            waiting = 1;
            wait_reported = 0;
            wait_tick = now;
        }
        bus_diag.wait_ms = now - wait_tick;
        if (bus_diag.wait_ms >= CAN_TX_TIMEOUT_MS && !wait_reported)
        {
            wait_reported = 1;
            bus_diag.error |= BUS_ERROR_CAN;
            if (type == JOB_STOP)
            {
                bus_diag.stop_retries++;
                stop_tick = now;
                if (wheel == stop_address)
                    diagnostic_state = CAN_FAILED;
            }
            else
            {
                if (type == JOB_DIAG)
                    diagnostic_pending = 0;
                Fault(BUS_ERROR_CAN);
                Bus_Cancel();
                if (type == JOB_DIAG || type == JOB_FINITE)
                    diagnostic_state = CAN_FAILED;
            }
        }
        return 0;
    }
    waiting = 0;
    wait_reported = 0;
    bus_diag.wait_ms = 0;
    if (type == JOB_SPEED && !wheel)
        batch_tick = HAL_GetTick();
    job = type;
    job_wheel = wheel;
    reply_code = frames[0].data[0];
    reply_wait = (uint8_t)((type == JOB_QUERY || type == JOB_DIAG) && count == 1 &&
                          (reply_code == 0x33 || reply_code == 0x35 || reply_code == 0x36 ||
                           reply_code == 0x3A || reply_code == 0x42));
    reply_matched = 0;
    if (reply_wait)
    {
        reply_address = (uint8_t)(frames[0].id >> 8);
        Drive_Reset(&query_config_rx);
        reply_barrier = can_diag.rx_sequence;
        reply_start = now;
        bus_diag.query_waiting = 1;
        bus_diag.query_address = reply_address;
        bus_diag.query_code = reply_code;
        bus_diag.query_tx_sequence = can_diag.tx_sequence;
    }
    return 1;
}
static void TimeoutRecord(uint32_t now)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (!bus_timeout.valid)
    {
        bus_timeout.version = 1;
        bus_timeout.tick = now;
        bus_timeout.address = reply_address;
        bus_timeout.code = reply_code;
        bus_timeout.start = reply_start;
        bus_timeout.barrier = reply_barrier;
        bus_timeout.tx_sequence = bus_diag.query_tx_sequence;
        bus_timeout.tx_done_tick = can_diag.tx_done_tick;
        bus_timeout.tx_done_sequence = can_diag.tx_done_sequence;
        bus_timeout.rx_sequence = can_diag.rx_sequence;
        bus_timeout.error = bus_diag.error;
        bus_timeout.active = bus_diag.active;
        bus_timeout.locked = bus_diag.locked;
        bus_timeout.job = job;
        bus_timeout.tx_state = (uint32_t)Can_Status();
        bus_timeout.query_count = bus_diag.query_count;
        bus_timeout.query_timeouts = bus_diag.query_timeouts;
        bus_timeout.last_rx_id = can_diag.last_rx_id;
        bus_timeout.last_rx_length = can_diag.last_rx_length;
        bus_timeout.last_rx_data0 = can_diag.last_rx_data0;
        bus_timeout.last_rx_data1 = can_diag.last_rx_data1;
        bus_timeout.last_rx_tick = can_diag.last_rx_tick;
        __DMB();
        bus_timeout.valid = 1;
    }
    __set_PRIMASK(mask);
}
static void Complete(uint32_t now)
{
    CanState state = Can_Status();
    uint8_t previous = job, i;
    if (!job || state == CAN_BUSY || state == CAN_CANCELLING)
        return;
    if (state == CAN_SENT && reply_wait)
    {
        if (hold_watch && reply_address >= 5 && reply_address <= 6)
        {
            hold_tx_tick = can_diag.tx_done_tick;
            hold_tx_valid = 1;
        }
        if (!reply_matched)
        {
            if ((uint32_t)(now - reply_start) < BUS_REPLY_TIMEOUT_MS)
                return;
            bus_diag.query_timeouts++;
            TimeoutRecord(now);
            reply_wait = 0;
            bus_diag.query_waiting = 0;
            job = JOB_NONE;
            if (previous == JOB_DIAG)
                diagnostic_state = CAN_FAILED;
            /* 缺回复不自动重查或重放运动；保留四轮停止优先级。 */
            Fault(BUS_ERROR_REPLY);
            return;
        }
        if ((uint32_t)(now - reply_tick) < CAN_TX_TURNAROUND_MS)
            return;
    }
    reply_wait = 0;
    bus_diag.query_waiting = 0;
    job = JOB_NONE;
    if (state != CAN_SENT)
    {
        if (previous == JOB_DIAG || previous == JOB_FINITE)
            diagnostic_state = CAN_FAILED;
        if (previous == JOB_STOP)
        {
            bus_diag.stop_retries++;
            stop_tick = now;
            if (job_wheel == stop_address)
                diagnostic_state = CAN_FAILED;
        }
        Fault(BUS_ERROR_CAN);
        return;
    }
    if (previous == JOB_DIAG)
        diagnostic_state = CAN_SENT;
    else if (previous == JOB_FINITE)
    {
        bus_diag.finite_sent_mask |= 1UL << job_wheel;
        finite_wheel++;
        if (finite_wheel == 4)
        {
            finite_pending = 0;
            bus_diag.finite_sent_sequence = bus_diag.finite_sequence;
            diagnostic_state = CAN_SENT;
        }
    }
    else if (previous == JOB_STOP)
    {
        stop_bits[job_wheel / 8U] &= (uint8_t) ~(1U << (job_wheel % 8U));
        bus_diag.pending_stop_count--;
        StopMask();
        if (!bus_diag.pending_stop_count)
            bus_diag.stop_sent_sequence = bus_diag.stop_sequence;
        if (job_wheel == stop_address)
            diagnostic_state = CAN_SENT;
    }
    else if (previous == JOB_SPEED)
    {
        batch_sent_tick[batch_wheel] = can_diag.tx_done_tick;
        batch_wheel++;
        if (batch_wheel == 4 && batch_position)
        {
            batch = 0;
            batch_position = 0;
            bus_diag.position_sent_tick = now;
            bus_diag.position_sent_sequence = batch_sequence;
            bus_diag.speed_span_ms = now - batch_tick;
            prefer_query = hold_watch ? 8U : BUS_QUERIES_PER_BATCH;
            bus_diag.speed_batches++;
        }
        else if (batch_wheel == 4)
        {
            batch = 0;
            for (i = 0; i < 4; i++)
            {
                bus_diag.sent_rpm[i] = batch_rpm[i];
                bus_diag.sent_wheel_tick[i] = batch_sent_tick[i];
            }
            bus_diag.sent_tick = now;
            bus_diag.speed_sent_sequence = batch_sequence;
            bus_diag.speed_span_ms = now - batch_tick;
            prefer_query = hold_watch ? 8U : BUS_QUERIES_PER_BATCH;
            bus_diag.speed_batches++;
        }
    }
}
static void CheckErrors(void)
{
    /* 每个计数只取一次；取消过程中新增的中断事件留给下一轮，不能记作已检查。 */
    uint32_t errors = can_diag.errors, overflow = can_diag.rx_overflow;
    uint32_t changed = 0;
    if (errors != seen_errors)
        changed |= BUS_ERROR_CAN;
    if (overflow != seen_overflow)
        changed |= BUS_ERROR_RX;
    if ((bus_diag.active || finite_owned || job) && changed)
    {
        /* 同轮发送错误与接收溢出分别保留，不能用二选一丢失原因。 */
        Fault(changed);
        Bus_Cancel();
    }
    seen_errors = errors;
    seen_overflow = overflow;
}
void Bus_Poll(void)
{
    BusFrame frames[2];
    uint32_t now = HAL_GetTick();
    uint8_t i, count, op;
    uint16_t scan, address;
    /* 已观察到的错误先取消旧事务，避免先推进下一帧再处理故障。 */
    CheckErrors();
    Can_Poll();
    CheckErrors();
    Receive();
    /* RX搬运/观察回调期间也可能进错误中断；本轮不得先报告完成。 */
    CheckErrors();
    now = HAL_GetTick();
    if (cancelling)
    {
        if (Can_Status() == CAN_CANCELLING)
        {
            bus_diag.wait_ms = now - cancel_tick;
            if (bus_diag.wait_ms >= CAN_TX_TIMEOUT_MS)
                bus_diag.error |= BUS_ERROR_CAN;
            Depth();
            return;
        }
        cancelling = 0;
        if (diagnostic_state == CAN_CANCELLING)
            diagnostic_state = CAN_FAILED;
    }
    Complete(now);
    if (job || Can_Status() == CAN_BUSY || Can_Status() == CAN_CANCELLING)
    {
        Depth();
        return;
    }
    if (bus_diag.pending_stop_count)
    {
        if ((uint32_t)(now - stop_tick) < BUS_STOP_RETRY_MS)
        {
            Depth();
            return;
        }
        for (scan = 0; scan < 255; scan++)
        {
            address = (uint16_t)(1U + (stop_cursor - 1U + scan) % 255U);
            if (!(stop_bits[address / 8U] & (1U << (address % 8U))))
                continue;
            count = Drive_Build((uint8_t)address, 0xFE, 0, 0, 0, 0, 0, 0, frames);
            if (count && Submit(JOB_STOP, (uint8_t)address, frames, count))
                stop_cursor = (uint16_t)(1U + address % 255U);
            break;
        }
    }
    else if (finite_pending)
    {
        count = Drive_Build(finite_address[finite_wheel], 0xFD, finite_dir[finite_wheel], finite_rpm, 0,
                            finite_pulses[finite_wheel], 2, 0, frames);
        if (count == 2)
            (void)Submit(JOB_FINITE, finite_wheel, frames, count);
        else
        {
            Fault(BUS_ERROR_CAN);
            Bus_Cancel();
        }
    }
    else if (diagnostic_pending)
    {
        if (diagnostic[0].data[0] == 0xFE && (uint32_t)(now - stop_tick) < BUS_STOP_RETRY_MS)
        {
            Depth();
            return;
        }
        if (Submit(JOB_DIAG, 0, diagnostic, diagnostic_count))
            diagnostic_pending = 0;
    }
    else if (diagnostic_owner || bus_diag.locked)
    {
        Depth();
        return;
    }
    else if (batch)
    {
        count = BuildBatch(batch_wheel, frames);
        if (count)
            (void)Submit(JOB_SPEED, batch_wheel, frames, count);
    }
    else if (bus_diag.active && (uint32_t)(now - query_tick) >= BUS_QUERY_INTERVAL_MS &&
             (prefer_query || !latest))
    {
        uint8_t address;
        if (hold_watch)
        {
            /* 轮/轴交错，避免连续两笔机构查询违反旧驱动冷却间隔。 */
            static const uint8_t slots[20] = {0,12,1,13,2,14,3,15,4,16,5,17,6,18,7,19,8,9,10,11};
            static const uint8_t codes[4] = {0x35, 0x36, 0x3A, 0x33};
            uint8_t slot = slots[hold_query_index];
            uint32_t oldest = 0;
            for (uint8_t n = 0; n < 20; n++)
            {
                uint8_t candidate = slots[(hold_query_index + n) % 20U];
                uint32_t age = hold_sample_valid[candidate] ? now - hold_sample_tick[candidate] : UINT32_MAX;
                /* 活动轮询延续旧四轮速度/位置优先级，机构八项同等优先。
                 * 轮状态在使能/停车时由静态查询补齐，活动期仍低频观察。 */
                if (candidate < 12 && candidate % 3U == 2 && age != UINT32_MAX)
                    age /= 2U;
                if (candidate >= 12 &&
                    ((hold_tx_valid && (uint32_t)(now - hold_tx_tick) < 10U) ||
                     (hold_rx_valid && (uint32_t)(now - hold_rx_tick) < 12U)))
                    continue;
                if (age >= oldest)
                {
                    oldest = age;
                    slot = candidate;
                }
            }
            i = slot < 12 ? slot / 3U : 0;
            address = slot < 12 ? addresses[i] : (uint8_t)(5U + (slot - 12U) / 4U);
            op = slot < 12 ? codes[slot % 3U] : codes[(slot - 12U) % 4U];
            if (address >= 5 && ((hold_tx_valid && (uint32_t)(now - hold_tx_tick) < 10U) ||
                                (hold_rx_valid && (uint32_t)(now - hold_rx_tick) < 12U)))
                return;
        }
        else
        {
            i = (uint8_t)(query_index / 2U);
            address = addresses[i];
            op = (query_index & 1U) ? 0x36 : 0x35;
        }
        count = Drive_Build(address, op, 0, 0, 0, 0, 0, 0, frames);
        if (count && Submit(JOB_QUERY, i, frames, count))
        {
            if (hold_watch)
                hold_query_index = (uint8_t)((hold_query_index + 1U) % 20U);
            else
                query_index = (uint8_t)((query_index + 1U) % 8U);
            query_tick = now;
            if (prefer_query)
                prefer_query--;
            bus_diag.query_count++;
        }
    }
    else if (bus_diag.active && latest && !prefer_query)
    {
        memcpy(batch_rpm, latest_rpm, sizeof(batch_rpm));
        memcpy(batch_dir, latest_dir, sizeof(batch_dir));
        memcpy(batch_pulses, latest_pulses, sizeof(batch_pulses));
        batch_position_rpm = latest_position_rpm;
        batch_position = latest_position;
        batch_sequence = latest_sequence;
        latest = 0;
        latest_position = 0;
        batch = 1;
        batch_wheel = 0;
        batch_tick = now;
        count = BuildBatch(0, frames);
        if (count)
            (void)Submit(JOB_SPEED, 0, frames, count);
    }
    Depth();
}
