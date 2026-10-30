#include "base_port.h"
#include "exec.h"
#include "bus_internal.h"
#include "bus_config.h"
#include "port.h"
#include "comm.h"
#include "execute.h"
#include "addresses.h"
#include "can.h"
#include "mech_port.h"
#include "mech_guard.h"
#include "start.h"
#include "gpio.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

static Exec executor;
static Start start_key;
static Select zone_switch;
static ExecGate start_failure;
static uint32_t diag_busy, query_index, query_ms, enable_sent;
static uint8_t query_slot;
/* 旧零速票据路径扩展到micro整批非零TXOK；必须同时匹配动作、序号及四轮RPM。 */
static uint32_t zero_sequence, zero_action, zero_id;
static int16_t acknowledged_rpm[4];
/* 位置小步整批TXOK票据：同一动作、同一Bus序号才回调Exec_Sent。 */
static uint32_t position_sequence, position_action, position_token;
static uint32_t stop_barrier, stable_tick;
static int64_t stable_position[4];
static ImuSample gyro;
static const uint8_t addresses[4] = BASE_SAFE_ADDRESSES;
static uint32_t startup_pending, startup_ms;
static uint32_t recovery_started, recovery_version;
static uint32_t recovery_token;
static uint8_t recovery_pending;
/* 首次机构停车门槛失败；本次启动内冻结，GET/STOP均不覆盖。 */
static struct
{
    uint32_t captured, tick, mask, wheel, stopped, exec_fault, bus_error, can_errors, action, channel;
    BusWheel wheels[4];
} park_failure;
typedef char ExecConfigSize[(sizeof(ExecConfig) == 52) ? 1 : -1];
typedef char ExecGoalSize[(sizeof(TrackGoal) == 96) ? 1 : -1];

/* 每次config10动作最多224个50ms样本，完成/故障冻结；SWD只读，不占UART/CAN。 */
typedef struct
{
    uint32_t tick, phase, state, output_token, batch_sequence;
    int16_t command[4], sent[4], measured[4];
    int64_t position[4];
    uint32_t tx_tick[4], position_tick[4], speed_tick[4];
    float yaw, gyro, position_error, yaw_error;
} MicroTraceSample;
typedef struct
{
    uint32_t action, count, frozen, last_ms, end_ms, exec_fault, track_fault;
    MicroTraceSample samples[224];
} MicroTrace;
static MicroTrace micro_trace;
static void MicroCapture(uint32_t now)
{
    const Track *t = &executor.track;
    const BusWheel *w = Bus_Wheels();
    uint8_t active = executor.mode == EXEC_ACTIVE;
    if (executor.config.version != 10 || !t->output.id) return;
    if (micro_trace.action != t->output.id)
    {
        if (!active) return;
        memset(&micro_trace, 0, sizeof(micro_trace));
        micro_trace.action = t->output.id;
    }
    if (micro_trace.frozen) return;
    if (micro_trace.count && active && (uint32_t)(now-micro_trace.last_ms) < 50U) return;
    if (micro_trace.count < 224)
    {
        MicroTraceSample *p = &micro_trace.samples[micro_trace.count++];
        p->tick = now; p->phase = t->micro_phase; p->state = t->output.state;
        p->output_token = Track_Token(t); p->batch_sequence = bus_diag.speed_sent_sequence;
        for (unsigned i=0; i<4; i++)
        {
            p->command[i] = (int16_t)t->output.rpm[i];
            p->sent[i] = bus_diag.sent_rpm[i]; p->tx_tick[i] = bus_diag.sent_wheel_tick[i];
            p->measured[i] = (int16_t)w[i].speed.value;
            p->position[i] = w[i].position.value;
            p->position_tick[i] = w[i].position.tick; p->speed_tick[i] = w[i].speed.tick;
        }
        p->yaw = (float)executor.observation.yaw_rad;
        p->gyro = (float)executor.observation.gyro_rad_s;
        p->position_error = (float)t->output.position_error_m;
        p->yaw_error = (float)t->output.yaw_error_rad;
        micro_trace.last_ms = now;
    }
    if (!active)
    {
        micro_trace.exec_fault = executor.fault; micro_trace.track_fault = t->output.fault;
        micro_trace.end_ms = now; micro_trace.frozen = 1;
    }
}

static int WheelReady(uint32_t now, int enabled, uint32_t since, int check_speed)
{
    const BusWheel *wheels = Bus_Wheels();
    unsigned index;
    for (index = 0; index < 4; index++)
    {
        const BusWheel *wheel = wheels + index;
        if (!wheel->speed.valid || !wheel->status.valid || !wheel->position.valid ||
            (uint32_t)(now - wheel->speed.tick) > 250 || (uint32_t)(now - wheel->status.tick) > 250 ||
            (uint32_t)(now - wheel->position.tick) > 250 || (check_speed && llabs(wheel->speed.value) > 1) ||
            (wheel->status.value & 12) || (enabled && !(wheel->status.value & 1)) ||
            (enabled && (uint32_t)(wheel->status.tick - since) >= 0x80000000UL))
        {
            return 0;
        }
    }
    return 1;
}

static int Ready(uint32_t now, int enabled, uint32_t since)
{
    return WheelReady(now, enabled, since, 1);
}

static int EnableWheel(unsigned wheel)
{
    BusFrame frames[2];
    uint8_t count;
    if (diag_busy)
    {
        return 0;
    }
    if (enable_sent)
    {
        enable_sent = 0;
        return 1;
    }
    count = Drive_Build(executor.config.address[wheel], 0xF3, 0, 0, 0, 0, 0, 1, frames);
    if (count && Bus_Send(frames, count))
    {
        diag_busy = 1;
        enable_sent = 1;
    }
    return 0;
}

static int Send(const int16_t rpm[4], uint32_t action, uint32_t zero)
{
    if (!Bus_Speed(rpm))
    {
        return 0;
    }
    if (!zero || !action)
    {
        zero_id = zero_action = zero_sequence = 0;
    }
    else if (zero_id != zero || zero_action != action ||
             memcmp(acknowledged_rpm, rpm, sizeof(acknowledged_rpm)))
    {
        /* 同一小步斜坡可由1变2RPM，Bus会合并尚未发送的旧速度。
         * 只在向量变化时更新匹配批次；相同速度重发不能推迟确认门槛。 */
        zero_id = zero;
        zero_action = action;
        zero_sequence = bus_diag.speed_sequence;
        memcpy(acknowledged_rpm, rpm, sizeof(acknowledged_rpm));
    }
    return 1;
}

/* 每轮FD脉冲数 = 电机整步数 × 驱动器细分，取自复位或重新使能后读回的0x42配置。 */
static uint32_t PulsesPerRev(const MotorConfig *config)
{
    if (!config->valid)
    {
        return 0;
    }
    return (config->motor_type == 0x19 ? 200U : 400U) * config->microsteps;
}

static int WheelConfigReady(void)
{
    const BusWheel *wheels = Bus_Wheels();
    unsigned index;
    for (index = 0; index < 4; index++)
    {
        if (!PulsesPerRev(&wheels[index].config))
        {
            return 0;
        }
    }
    return 1;
}

/* 微调配置10依赖四轮细分换算位置小步；配置未读齐前不准备PC0、不点火。 */
static int MicroReady(void)
{
    return executor.config.version != 10 || WheelConfigReady();
}

static int Position(const double revolutions[4], uint32_t rpm, uint32_t action, uint32_t token)
{
    const BusWheel *wheels = Bus_Wheels();
    uint8_t dir[4];
    uint32_t pulses[4];
    unsigned index;
    if (!action || !token || !rpm || rpm > MOTOR_MAX_COMMAND_RPM)
    {
        return 0;
    }
    for (index = 0; index < 4; index++)
    {
        double magnitude = fabs(revolutions[index]) * PulsesPerRev(&wheels[index].config);
        /* 小步上限6mm轮缘弧长，远小于一圈；越界说明换算或配置异常。 */
        if (!isfinite(magnitude) || magnitude < .5 || magnitude > PulsesPerRev(&wheels[index].config))
        {
            return 0;
        }
        pulses[index] = (uint32_t)(magnitude + .5);
        dir[index] = (uint8_t)(revolutions[index] < 0);
    }
    if (!Bus_Position(dir, (uint16_t)rpm, pulses))
    {
        return 0;
    }
    position_sequence = bus_diag.speed_sequence;
    position_action = action;
    position_token = token;
    zero_id = zero_action = zero_sequence = 0;
    return 1;
}

static void Stop(void)
{
    Bus_Stop();
    diag_busy = enable_sent = 0;
    zero_id = zero_sequence = zero_action = 0;
    position_sequence = position_action = position_token = 0;
    stop_barrier = stable_tick = 0;
    Bus_Release();
}

static uint32_t Fault(void)
{
    return (bus_diag.error & ~BUS_ERROR_STOP) | can_diag.errors | can_diag.rx_overflow |
        (MechPort_Holding() && !MechPort_HoldCheck(HAL_GetTick()) ? 0x4000U : 0U);
}

static uint8_t Park_Manual(void)
{
    /* 用户显式手动模式才取消轮速门槛；仍要求停车事务完成、底盘无运动控制权。 */
    return MechPort_Manual() && !startup_pending && executor.mode <= EXEC_OBSERVING &&
           executor.stop_confirmed && !bus_diag.active && !bus_diag.pending_stop_count &&
           bus_diag.stop_sequence == bus_diag.stop_sent_sequence && !Fault() && !executor.fault;
}

static uint8_t Park_Ready(uint32_t now)
{
    return executor.stop_confirmed && WheelReady(now, 0, 0, !Park_Manual());
}

static void Park_Capture(uint32_t now)
{
    uint32_t id, channel, mask = 0, first = 0;
    const BusWheel *wheels = Bus_Wheels();
    if (park_failure.captured || !MechPort_Active(&id, &channel))
        return;
    for (unsigned i = 0; i < 4; i++)
    {
        uint32_t reason = 0;
        const BusWheel *w = wheels + i;
        if (!w->speed.valid || !w->status.valid || !w->position.valid)
            reason |= 1;
        if ((uint32_t)(now - w->speed.tick) > 250 || (uint32_t)(now - w->status.tick) > 250 ||
            (uint32_t)(now - w->position.tick) > 250)
            reason |= 2;
        if (!Park_Manual() && llabs(w->speed.value) > 1)
            reason |= 4;
        if (w->status.value & 12)
            reason |= 8;
        if (reason && !first)
            first = i + 1;
        mask |= reason;
    }
    if (!executor.stop_confirmed)
        mask |= 16;
    if (Fault() || executor.fault)
        mask |= 32;
    if (!mask)
        return;
    park_failure.tick = now;
    park_failure.mask = mask;
    park_failure.wheel = first;
    park_failure.stopped = executor.stop_confirmed;
    park_failure.exec_fault = executor.fault;
    park_failure.bus_error = bus_diag.error;
    park_failure.can_errors = can_diag.errors;
    park_failure.action = id;
    park_failure.channel = channel;
    memcpy(park_failure.wheels, wheels, sizeof(park_failure.wheels));
    park_failure.captured = 1;
}

static uint16_t Park_Read(uint8_t *reply)
{
    uint32_t fields[10] = {2, park_failure.captured, park_failure.tick, park_failure.mask,
        park_failure.wheel, park_failure.stopped, park_failure.exec_fault,
        park_failure.can_errors, park_failure.action, park_failure.channel};
    memcpy(reply, fields, sizeof(fields));
    for (unsigned i = 0; i < 4; i++)
    {
        const BusWheel *w = park_failure.wheels + i;
        uint8_t *p = reply + sizeof(fields) + i * 20;
        Frame_Put(p, (uint32_t)w->speed.value, 4);
        Frame_Put(p + 4, (uint32_t)w->status.value, 4);
        Frame_Put(p + 8, w->speed.valid ? (uint32_t)(park_failure.tick - w->speed.tick) : UINT32_MAX, 4);
        Frame_Put(p + 12, w->position.valid ? (uint32_t)(park_failure.tick - w->position.tick) : UINT32_MAX, 4);
        Frame_Put(p + 16, w->status.valid ? (uint32_t)(park_failure.tick - w->status.tick) : UINT32_MAX, 4);
    }
    {
        uint32_t policy = Park_Manual() ? 1U : 0U;
        const BusWheel *w = Bus_Wheels();
        for (unsigned i = 0; i < 4; i++)
            if (w[i].speed.valid && llabs(w[i].speed.value) > 1)
                policy |= 2U; /* 仅观测到轮速，不等于当前停车门槛失败。 */
        Frame_Put(reply + 120, policy, 4);
    }
    return 124;
}

static int Rearm(void)
{
    /* 仅清理正常STOP的本地发送锁；任何CAN/RX/驱动异常不自动清除。 */
    return !Fault() && bus_diag.error == BUS_ERROR_STOP && Bus_Rearm();
}

static int Stopped(uint32_t since)
{
    const BusWheel *wheels = Bus_Wheels();
    uint32_t now = HAL_GetTick();
    uint32_t oldest = now;
    unsigned index;
    if (bus_diag.pending_stop_count || bus_diag.stop_sequence != bus_diag.stop_sent_sequence)
    {
        return 0;
    }
    if (!stop_barrier)
    {
        stop_barrier = now;
    }
    if (!Ready(now, 0, 0) ||
        (executor.config.version && (!Stamp_Check(&gyro.stamp, now, 250) ||
                                     fabs(gyro.axes[2] * executor.config.gyro_scale_rad_s) > 0.035)))
    {
        stable_tick = 0;
        return 0;
    }
    for (index = 0; index < 4; index++)
    {
        uint32_t tick = wheels[index].position.tick;
        if ((uint32_t)(tick - stop_barrier) >= 0x80000000UL || tick == stop_barrier ||
            (uint32_t)(wheels[index].speed.tick - stop_barrier) >= 0x80000000UL ||
            (uint32_t)(tick - since) >= 0x80000000UL)
        {
            return 0;
        }
        if ((uint32_t)(now - tick) > (uint32_t)(now - oldest))
        {
            oldest = tick;
        }
        if (stable_tick && llabs(wheels[index].position.value - stable_position[index]) > 32)
        {
            stable_tick = 0;
        }
    }
    if (!stable_tick)
    {
        for (index = 0; index < 4; index++)
        {
            stable_position[index] = wheels[index].position.value;
        }
        stable_tick = oldest;
        return 0;
    }
    return (uint32_t)(oldest - stable_tick) >= 150;
}

static void StopCommand(uint32_t reason, uint32_t now)
{
    Bus_HoldWatch(0);
    recovery_pending = 0; /* 明确STOP可取消尚未执行的重启。 */
    Start_Cancel(&start_key);
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    MechPort_Stop(reason, now);
    Exec_Stop(&executor, reason, now);
}

static void Recover_Run(uint32_t now)
{
    if (recovery_pending && (uint32_t)(now - recovery_started) >= 250)
    {
        /* 给ACK及停车留出时间；新反馈仍静止才执行一次系统复位。 */
        if (Stopped(stop_barrier) && MechPort_CanReset(now, recovery_version))
        {
            recovery_pending = 0;
            NVIC_SystemReset();
        }
        else if ((uint32_t)(now - recovery_started) >= 2000)
            recovery_pending = 0;
    }
}

static uint8_t Command(uint8_t type, const uint8_t *data, uint16_t size, uint8_t *reply, uint16_t *reply_size,
                       uint32_t now)
{
    uint32_t configured = executor.config.version;
    if (type == 71 || type == 72)
    {
        uint8_t stopped = (uint8_t)(!startup_pending && !Fault() && !executor.fault &&
            executor.mode != EXEC_ACTIVE && executor.mode != EXEC_ARMING &&
            !bus_diag.motion_uncertain && Stopped(stop_barrier));
        uint8_t result;
        uint8_t previous_holding = MechPort_Holding();
        if (type == 71 && size == 76 &&
            ((Frame_Get(data, 4) == 3 && MechPort_Holding()) ||
             (Frame_Get(data, 4) == 4 && !MechPort_Holding())))
            return COMM_BUSY; /* 活动租约必须走返回事务，不能以取消跳过底盘结束。 */
        result = MechPort_Handoff(type, data, size, reply, reply_size, now, stopped);
        if (type == 71 && size == 76 && Frame_Get(data, 4) == 4 &&
            result == COMM_OK && *reply_size == 4 && Frame_Get(reply, 4) == MECH_UNSTABLE &&
            MechPort_Holding() && executor.mode == EXEC_READY && !Fault() && !executor.fault)
        {
            /* 已完成的小位移结束活动批次，静态六节点补齐停车证据；不STOP/CONFIG。 */
            Bus_End();
            Bus_HoldWatch(0);
            executor.mode = EXEC_OBSERVING;
        }
        if (type == 71 && result == COMM_OK && *reply_size == 4 && !Frame_Get(reply, 4))
        {
            if (previous_holding != MechPort_Holding())
                Bus_HoldWatch(MechPort_Holding());
            if (Frame_Get(data, 4) == 4 && !MechPort_Holding())
            {
                Bus_End();
                executor.mode = EXEC_OBSERVING;
                executor.stop_confirmed = 1;
                Start_Init(&start_key); /* 正常返回结束本轮票据；下轮仍须新合同和新PC0。 */
            }
        }
        return result;
    }
    if (!MechPort_Holding() && type == EXEC_CONFIG && size == sizeof(ExecConfig) &&
        Frame_Get(data, 4) == 10)
        return COMM_READONLY;
    if (MechPort_Holding())
    {
        /* 本轮租约只开放单轴1～5mm平移或3°以内纯转向的低速微调。 */
        if (type == 59 && (size != 8 || Frame_Get(data, 4) != MechPort_HoldTicket() ||
                          Frame_Get(data + 4, 4) != 10))
            return COMM_READONLY;
        if (type == EXEC_CONFIG)
        {
            ExecConfig config;
            if (size != sizeof(config))
                return COMM_FORMAT;
            memcpy(&config, data, sizeof(config));
            if (config.version != 10 || config.max_rpm > 6)
                return COMM_READONLY;
        }
        if (type == EXEC_GOAL)
        {
            TrackGoal goal;
            if (size != sizeof(goal))
                return COMM_FORMAT;
            memcpy(&goal, data, sizeof(goal));
            if (executor.config.version != 10 || goal.reference.linear.velocity > .01 ||
                goal.reference.angular.velocity > .1001 ||
                (goal.reference.kind == 1 ? (goal.reference.yaw_rad != 0 ||
                    !((goal.reference.x_m == 0) != (goal.reference.y_m == 0)) ||
                    hypot(goal.reference.x_m, goal.reference.y_m) > .00501) :
                 goal.reference.kind != 2 || goal.reference.x_m != 0 || goal.reference.y_m != 0 ||
                    fabs(goal.reference.yaw_rad) > .05236))
                return COMM_READONLY;
        }
        if (((type >= 48 && type <= 57) || (type >= 61 && type <= 63) ||
             type == 65 || type == 66 || type == 67))
            return COMM_BUSY;
    }
    if (type == 69)
    {
        if (size)
            return COMM_FORMAT;
        *reply_size = Park_Read(reply);
        return COMM_OK;
    }
    Select_Run(&zone_switch, now, HAL_GPIO_ReadPin(ZONE_SWITCH_GPIO_Port, ZONE_SWITCH_Pin) == GPIO_PIN_SET);
    Start_Check(&start_key, &zone_switch, now);
    if (type == 65)
    {
        uint32_t result = MECH_OK;
        if (size != 16 || !Frame_Get(data, 4) || Frame_Get(data, 4) <= recovery_token ||
            Frame_Get(data + 4, 4) != 0x52454356UL ||
            Frame_Get(data + 8, 4) != 7)
            return COMM_FORMAT;
        if (recovery_pending || executor.mode == EXEC_ACTIVE || executor.mode == EXEC_ARMING ||
            executor.mode == EXEC_READY || !executor.stop_confirmed || !Stopped(stop_barrier) ||
            !MechPort_CanReset(now, (uint32_t)Frame_Get(data + 12, 4)))
            result = MECH_UNSTABLE;
        else
        {
            StopCommand(MECH_MANUAL, now);
            if (!MechPort_PwmOff())
                result = MECH_FAULT;
            else
            {
                recovery_version = (uint32_t)Frame_Get(data + 12, 4);
                recovery_token = (uint32_t)Frame_Get(data, 4);
                recovery_started = now;
                recovery_pending = 1;
            }
        }
        Frame_Put(reply, result, 4);
        *reply_size = 4;
        return COMM_OK;
    }
    if (recovery_pending && type != EXEC_STATE && type != 58 && type != 64 && type != 68 && type != 70 && type != 36 && type != 38)
        return COMM_BUSY;
    if (type == EXEC_ARM && (start_key.phase != START_FIRED ||
        start_key.version != configured || start_key.boot != Comm_Boot() ||
        start_key.session != Comm_Session() ||
        (configured == 9 && (!start_key.zone || start_key.zone != executor.route_zone))))
        return COMM_READONLY;
    if (type == 59 || type == 60)
    {
        if (type == 59)
        {
            uint32_t ticket = (size == 8 || size == 16) ? (uint32_t)Frame_Get(data, 4) : 0;
            uint32_t version = (size == 8 || size == 16) ? (uint32_t)Frame_Get(data + 4, 4) : 0;
            uint32_t zone = size == 16 ? (uint32_t)Frame_Get(data + 8, 4) : 0;
            uint32_t epoch = size == 16 ? (uint32_t)Frame_Get(data + 12, 4) : 0;
            uint8_t ready = (uint8_t)(!startup_pending && !Fault() && !executor.fault &&
                Exec_Ready(&executor, now) && MicroReady() &&
                !MechPort_Owned() && Ready(now, 0, 0));
            if ((configured == 9 && (size != 16 || !zone || zone != Select_Zone(&zone_switch) ||
                                     !epoch || epoch != zone_switch.epoch)) ||
                (configured != 9 && size != 8))
                return COMM_READONLY;
            if (!ticket || version != configured || !Start_Prepare(&start_key, Comm_Boot(), Comm_Session(),
                ticket, version, now, HAL_GPIO_ReadPin(START_KEY_GPIO_Port, START_KEY_Pin) == GPIO_PIN_RESET, ready))
                return COMM_READONLY;
            if (configured == 9 && (!Exec_Select(&executor, zone) ||
                !Start_Bind(&start_key, &zone_switch, zone, epoch)))
            {
                Start_Cancel(&start_key);
                return COMM_READONLY;
            }
        }
        else if (size)
            return COMM_FORMAT;
        Frame_Put(reply, 3, 4);
        Frame_Put(reply + 4, start_key.phase, 4);
        Frame_Put(reply + 8, start_key.ticket, 4);
        Frame_Put(reply + 12, start_key.version, 4);
        Frame_Put(reply + 16, start_key.phase == START_FIRED, 4);
        Frame_Put(reply + 20, start_key.beep, 4);
        Frame_Put(reply + 24, start_key.reason, 4);
        Frame_Put(reply + 28, start_key.aborted_ms, 4);
        Frame_Put(reply + 32, start_failure.mask, 4);
        Frame_Put(reply + 36, start_failure.now_ms, 4);
        Frame_Put(reply + 40, start_failure.pose_ms, 4);
        Frame_Put(reply + 44, start_failure.yaw_ms, 4);
        Frame_Put(reply + 48, start_failure.gyro_ms, 4);
        memcpy(reply + 52, start_failure.position_ms, 16);
        memcpy(reply + 68, start_failure.speed_ms, 16);
        Frame_Put(reply + 84, zone_switch.raw, 4);
        Frame_Put(reply + 88, Select_Zone(&zone_switch), 4);
        Frame_Put(reply + 92, zone_switch.valid, 4);
        Frame_Put(reply + 96, zone_switch.epoch, 4);
        Frame_Put(reply + 100, start_key.zone, 4);
        Frame_Put(reply + 104, start_key.select_epoch, 4);
        *reply_size = 108;
        return COMM_OK;
    }
    if ((type >= 48 && type <= 58) || (type >= 61 && type <= 64) || (type >= 66 && type <= 68) || type == 70)
    {
        if (type != 58 && type != 64 && type != 68 && type != 70 && (startup_pending || executor.mode == EXEC_ACTIVE ||
                           executor.mode == EXEC_ARMING || executor.mode == EXEC_READY))
            return COMM_BUSY;
        if (type == 61 && Fault())
        {
            Frame_Put(reply, MECH_FAULT, 4);
            *reply_size = 4;
            return COMM_OK;
        }
        if (type == 61 && (!executor.stop_confirmed || !Stopped(stop_barrier)))
            return COMM_BUSY;
        uint8_t mech_result = MechPort_Command(type, data, size, reply, reply_size, now);
        if (Bus_Status() == CAN_BUSY)
            diag_busy = 1;
        return mech_result;
    }
    if (MechPort_Owned() && (type == EXEC_ARM || type == EXEC_GOAL || type == EXEC_CONFIG))
        return COMM_BUSY;
    if (type == 36 && size == 0)
    {
        *reply_size = Port_Diag(reply);
        return COMM_OK;
    }
    if (type == 38 && size == 1)
    {
        *reply_size = Can_Capture(data[0], reply);
        return *reply_size ? COMM_OK : COMM_FORMAT;
    }
    if (startup_pending && type != EXEC_STATE && type != 16)
    {
        return COMM_BUSY;
    }
    uint8_t result = Execute_Command(&executor, type, data, size, reply, reply_size, now, can_diag.errors,
                                     can_diag.rx_sequence);
    if (!configured && executor.config.version && result == COMM_OK)
    {
        Bus_Setup(executor.config.address);
    }
    return result;
}

void Base_Init(void)
{
    CAN_FilterTypeDef filter = {0};
    ExecIo io = {Ready, EnableWheel, Bus_Start, Send, Stop, Fault, Stopped, Rearm, Position};
    CommHooks hooks = {StopCommand, Command};
    memset(&park_failure, 0, sizeof(park_failure));
    query_slot = 0;
    recovery_pending = 0;
    recovery_token = 0;
    Exec_Init(&executor, io);
    Start_Init(&start_key);
    Select_Init(&zone_switch, HAL_GetTick(),
        HAL_GPIO_ReadPin(ZONE_SWITCH_GPIO_Port, ZONE_SWITCH_Pin) == GPIO_PIN_SET);
    MechPort_Init();
    Bus_Observe(MechPort_Feed);
    /* 启动接收不等于使能或动作；配置/使能/目标分别显式提交。 */
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdLow = CAN_ID_EXT;
    filter.FilterMaskIdLow = CAN_ID_EXT | CAN_RTR_REMOTE;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK || HAL_CAN_Start(&hcan1) != HAL_OK ||
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_OVERRUN |
                                                 CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_ERROR | CAN_IT_BUSOFF) !=
            HAL_OK)
    {
        Exec_Stop(&executor, 0x400U, HAL_GetTick());
    }
    /* 复位不能让外部驱动继续保持旧RPM；只发送停止，不使能、不启动轨迹。 */
    Bus_Setup(addresses);
    Bus_MechWatch();
    Bus_Stop();
    startup_pending = 1;
    startup_ms = HAL_GetTick();
    Comm_Bind(hooks);
}

static void Query(uint32_t now)
{
    BusFrame frames[2];
    uint8_t count;
    const uint8_t kinds[3] = {0x35, 0x36, 0x3A};
    /* 12项轮反馈与8项轴反馈按3:2分配；不因拒收或冷却跳过任何项。 */
    static const uint8_t mech_slots[5] = {0, 1, 0, 1, 0};
    /* 启动STOP由Bus立即调度；准备期只延后新查询，不清故障、不提前准许动作。 */
    if (startup_pending && (uint32_t)(now - startup_ms) < BUS_STARTUP_SETTLE_MS)
    {
        return;
    }
    if (diag_busy || executor.mode == EXEC_READY ||
        executor.mode == EXEC_ACTIVE || (uint32_t)(now - query_ms) < BUS_QUERY_INTERVAL_MS)
    {
        return;
    }
    if (MechPort_Owned() || MechPort_Holding())
    {
        count = MechPort_Output(frames, now);
        if (count)
        {
            if (Bus_Send(frames, count))
                diag_busy = 1;
            else
                MechPort_Sent(0, now);
            return;
        }
        if (mech_slots[query_slot])
        {
            count = MechPort_Query(frames, now);
            if (count && Bus_Send(frames, count))
            {
                MechPort_QueryAccept();
                diag_busy = 1;
                query_ms = now;
                query_slot = (uint8_t)((query_slot + 1U) % 5U);
            }
            return;
        }
    }
    if (executor.config.version == 10 && !WheelConfigReady())
    {
        /* 微调换算需要四轮细分；复位/Rearm后先补读缺失配置，不推进常规轮询序号。 */
        const BusWheel *wheels = Bus_Wheels();
        unsigned wheel = 0;
        while (PulsesPerRev(&wheels[wheel].config))
        {
            wheel++;
        }
        count = Drive_Build(addresses[wheel], 0x42, 0, 0, 0, 0, 0, 0, frames);
        if (count && Bus_Send(frames, count))
        {
            diag_busy = 1;
            query_ms = now;
            if (MechPort_Owned() || MechPort_Holding())
                query_slot = (uint8_t)((query_slot + 1U) % 5U);
        }
        return;
    }
    count = Drive_Build(addresses[query_index / 3], kinds[query_index % 3], 0, 0, 0, 0, 0, 0, frames);
    if (count && Bus_Send(frames, count))
    {
        diag_busy = 1;
        query_index = (query_index + 1) % 12;
        query_ms = now;
        if (MechPort_Owned() || MechPort_Holding())
            query_slot = (uint8_t)((query_slot + 1U) % 5U);
    }
}

void Base_Run(void)
{
    ImuSample sample;
    MotorSample motor;
    BusFrame ignored;
    const BusWheel *wheels;
    const MotorReply *reply;
    uint32_t now;
    unsigned index, kind;
    Bus_Poll();
    now = HAL_GetTick();
    /* 准备期只是延后查询；STOP或接收期间的真实CAN错误仍须立即锁定。 */
    if (startup_pending && Fault() && !executor.fault)
    {
        Exec_Stop(&executor, 0x800U, now);
    }
    if (diag_busy && Bus_Status() != CAN_BUSY && Bus_Status() != CAN_CANCELLING)
    {
        MechPort_Sent((uint8_t)(Bus_Status() == CAN_SENT), now);
        /* 正常STOP会主动取消未完成查询；只有纯STOP且无真实错误时不报总线故障。
         * TERR、回包超时、RX溢出等仍由Fault检查及原失败分支立即锁定。 */
        if (Bus_Status() != CAN_SENT && !(bus_diag.error == BUS_ERROR_STOP && !Fault()))
        {
            Exec_Stop(&executor, 0x800U, now);
        }
        while (Bus_Read(&ignored))
        {
        }
        Bus_Release();
        diag_busy = 0;
    }
    wheels = Bus_Wheels();
    for (index = 0; index < 4; index++)
    {
        for (kind = 0; kind < 3; kind++)
        {
            reply = kind == 2   ? &wheels[index].status
                    : kind == 1 ? &wheels[index].position
                                : &wheels[index].speed;
            if (!reply->valid)
            {
                continue;
            }
            memset(&motor, 0, sizeof(motor));
            motor.address = reply->address;
            motor.kind = reply->code;
            motor.value = reply->value;
            motor.stamp.first_ms = motor.stamp.last_ms = reply->tick;
            motor.stamp.handled_ms = reply->handled_ms;
            motor.stamp.sequence = reply->rx_sequence;
            motor.stamp.epoch = 1;
            Exec_Motor(&executor, &motor, HAL_GetTick());
        }
    }
    while (Port_Imu(&sample))
    {
        if (sample.kind == 0x52)
        {
            gyro = sample;
        }
        Exec_Imu(&executor, &sample, HAL_GetTick());
    }
    now = HAL_GetTick();
    if (startup_pending && Stopped(startup_ms) && Rearm())
    {
        startup_pending = 0;
        executor.stop_confirmed = 1;
    }
    if (zero_sequence && (uint32_t)(bus_diag.speed_sent_sequence - zero_sequence) < 0x80000000UL &&
        bus_diag.sent_rpm[0] == acknowledged_rpm[0] && bus_diag.sent_rpm[1] == acknowledged_rpm[1] &&
        bus_diag.sent_rpm[2] == acknowledged_rpm[2] && bus_diag.sent_rpm[3] == acknowledged_rpm[3])
    {
        Exec_Sent(&executor, zero_action, zero_id, bus_diag.sent_tick);
    }
    if (position_sequence && bus_diag.position_sent_sequence == position_sequence)
    {
        Exec_Sent(&executor, position_action, position_token, bus_diag.position_sent_tick);
        position_sequence = 0;
    }
    Exec_Run(&executor, now);
    MicroCapture(now);
    {
        uint32_t phase = start_key.phase;
        uint8_t ready = (uint8_t)Exec_Ready(&executor, now);
        Select_Run(&zone_switch, now, HAL_GPIO_ReadPin(ZONE_SWITCH_GPIO_Port, ZONE_SWITCH_Pin) == GPIO_PIN_SET);
        Start_Check(&start_key, &zone_switch, now);
        Start_Run(&start_key, Comm_Boot(), Comm_Session(), now,
            HAL_GPIO_ReadPin(START_KEY_GPIO_Port, START_KEY_Pin) == GPIO_PIN_RESET,
            (uint8_t)(!Fault() && ready && MicroReady() && !MechPort_Owned()));
        if (phase >= START_RELEASE && phase <= START_BEEP && start_key.phase == START_ABORTED)
            start_failure = executor.gate;
    }
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, start_key.beep ? GPIO_PIN_RESET : GPIO_PIN_SET);
    Park_Capture(HAL_GetTick());
    {
        uint8_t was_holding = MechPort_Holding();
        MechPort_Run(HAL_GetTick(), Park_Ready(HAL_GetTick()), Fault() | executor.fault);
        if (was_holding && !MechPort_Holding())
        {
            Bus_HoldWatch(0);
            Exec_Stop(&executor, 0x4000U, HAL_GetTick());
        }
    }
    if (Fault() && !executor.fault)
        Exec_Stop(&executor, Fault(), HAL_GetTick());
    Query(now);
    Recover_Run(now);
}
