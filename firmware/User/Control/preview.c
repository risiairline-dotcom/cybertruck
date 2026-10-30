#include "preview.h"
#ifdef EXEC_PREVIEW
#include "exec.h"
#include "execute.h"
#include "comm.h"
#include "start.h"
#include <stdlib.h>
#include <string.h>

/* 单一主机模拟器，由Python进程锁保护；无HAL、串口或CAN设备依赖。
 * 复用正式Comm/Start/Exec，驱动应答和GPIO电平明确由独立模型产生。 */
static Exec executor;
static Start start_key;
static Select zone_switch;
static ExecGate start_failure;
static uint32_t now, sequence, fault, missing, enabled_mask, pending_id, pending_zero, pending_tick;
static uint32_t stable_ms;
static int64_t stable_counts[4];
static int16_t targets[4];
static uint8_t key_level;

static int Ready(uint32_t tick, int enabled, uint32_t since)
{
    unsigned index;
    for (index = 0; index < 4; index++)
    {
        const MotorSample *status = &executor.status[index];
        if (!Stamp_Check(&status->stamp, tick, 250) || (status->value & 12) ||
            !Stamp_Check(&executor.speed[index].stamp, tick, 250) ||
            llabs(executor.speed[index].value) > 1 ||
            (enabled && (!(status->value & 1) || (uint32_t)(status->stamp.last_ms - since) >= 0x80000000UL)))
            return 0;
    }
    return !fault;
}

static int Enable(unsigned wheel)
{
    enabled_mask |= 1U << wheel;
    return 1;
}

static int Begin(void)
{
    return !fault;
}

static int Send(const int16_t *rpm, uint32_t action, uint32_t zero)
{
    memcpy(targets, rpm, sizeof(targets));
    if (!zero)
        pending_zero = 0;
    else if (pending_zero != zero || pending_id != action)
    {
        pending_id = action;
        pending_zero = zero;
        pending_tick = now + 12;
    }
    return 1;
}

static void Stop(void)
{
    memset(targets, 0, sizeof(targets));
    pending_zero = stable_ms = 0;
}

static uint32_t Fault(void)
{
    return fault;
}

static int Stopped(uint32_t since)
{
    unsigned index;
    if (!Ready(now, 0, 0))
    {
        stable_ms = 0;
        return 0;
    }
    for (index = 0; index < 4; index++)
    {
        if (!Stamp_Check(&executor.position[index].stamp, now, 250) ||
            (uint32_t)(executor.position[index].stamp.last_ms - since) >= 0x80000000UL)
            return 0;
        if (stable_ms && llabs(executor.position[index].value - stable_counts[index]) > 32)
            stable_ms = 0;
    }
    if (!stable_ms)
    {
        for (index = 0; index < 4; index++)
            stable_counts[index] = executor.position[index].value;
        stable_ms = now;
    }
    return (uint32_t)(now - stable_ms) >= 150;
}

static int Rearm(void)
{
    return !fault;
}

static void StopCommand(uint32_t reason, uint32_t tick)
{
    Start_Cancel(&start_key);
    Exec_Stop(&executor, reason, tick);
}

static uint8_t Command(uint8_t type, const uint8_t *data, uint16_t size,
                       uint8_t *reply, uint16_t *reply_size, uint32_t tick)
{
    Start_Check(&start_key, &zone_switch, tick);
    if (type == EXEC_ARM && (start_key.phase != START_FIRED || start_key.boot != Comm_Boot() ||
        start_key.session != Comm_Session() || start_key.version != executor.config.version ||
        start_key.zone != executor.route_zone))
        return COMM_READONLY;
    if (type == 59 || type == 60)
    {
        if (type == 59)
        {
            uint32_t ticket = size == 16 ? (uint32_t)Frame_Get(data, 4) : 0;
            uint32_t version = size == 16 ? (uint32_t)Frame_Get(data + 4, 4) : 0;
            uint32_t zone = size == 16 ? (uint32_t)Frame_Get(data + 8, 4) : 0;
            uint32_t epoch = size == 16 ? (uint32_t)Frame_Get(data + 12, 4) : 0;
            if (!zone || zone != Select_Zone(&zone_switch) || epoch != zone_switch.epoch ||
                version != 9 || version != executor.config.version || !Ready(tick, 0, 0) ||
                !Start_Prepare(&start_key, Comm_Boot(), Comm_Session(), ticket, version,
                              tick, key_level, (uint8_t)Exec_Ready(&executor, tick)) ||
                !Exec_Select(&executor, zone) || !Start_Bind(&start_key, &zone_switch, zone, epoch))
                return COMM_READONLY;
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
        memcpy(reply + 32, &start_failure, sizeof(start_failure));
        Frame_Put(reply + 84, zone_switch.raw, 4);
        Frame_Put(reply + 88, Select_Zone(&zone_switch), 4);
        Frame_Put(reply + 92, zone_switch.valid, 4);
        Frame_Put(reply + 96, zone_switch.epoch, 4);
        Frame_Put(reply + 100, start_key.zone, 4);
        Frame_Put(reply + 104, start_key.select_epoch, 4);
        *reply_size = 108;
        return COMM_OK;
    }
    return Execute_Command(&executor, type, data, size, reply, reply_size, tick, fault, sequence);
}

void Preview_Init(uint64_t boot)
{
    /* 路线预演不含配置10微调，没有位置小步出口；误入时Exec按发送失败停止。 */
    ExecIo io = {Ready, Enable, Begin, Send, Stop, Fault, Stopped, Rearm, 0};
    CommHooks hooks = {StopCommand, Command};
    Exec_Init(&executor, io);
    executor.stop_confirmed = 1;
    Start_Init(&start_key);
    Select_Init(&zone_switch, 0, 0);
    memset(&start_failure, 0, sizeof(start_failure));
    now = sequence = fault = missing = enabled_mask = pending_zero = pending_id = stable_ms = 0;
    key_level = 0;
    memset(targets, 0, sizeof(targets));
    Comm_Init(boot, 0);
    Comm_Bind(hooks);
}

int Preview_Load(uint32_t zone, const uint8_t *goals, uint32_t count)
{
    return Exec_Preview(&executor, zone, (const TrackGoal *)goals, count);
}

void Preview_Push(const uint8_t *data, uint32_t size)
{
    uint32_t index;
    for (index = 0; index < size; index++)
        Comm_Feed(data[index], now, now);
}

uint16_t Preview_Read(uint8_t *data)
{
    return Comm_Read(data);
}

void Preview_Tick(uint32_t tick, const int64_t *counts, const int16_t *rpm,
                  int16_t yaw, int16_t gyro, uint8_t key, uint8_t select)
{
    MotorSample motor = {0};
    ImuSample imu = {0};
    unsigned index;
    uint32_t phase = start_key.phase;
    now = tick;
    key_level = key;
    for (index = 0; index < 4; index++)
    {
        motor.address = (uint8_t)(index + 1);
        motor.stamp.first_ms = motor.stamp.last_ms = motor.stamp.handled_ms = tick;
        motor.stamp.epoch = 1;
        motor.stamp.sequence = ++sequence;
        motor.kind = 0x36;
        motor.value = counts[index];
        if (!(missing & 1)) Exec_Motor(&executor, &motor, tick);
        motor.stamp.sequence = ++sequence;
        motor.kind = 0x35;
        motor.value = rpm[index];
        if (!(missing & 2)) Exec_Motor(&executor, &motor, tick);
        motor.stamp.sequence = ++sequence;
        motor.kind = 0x3A;
        motor.value = (enabled_mask >> index) & 1;
        if (!(missing & 16)) Exec_Motor(&executor, &motor, tick);
    }
    imu.stamp.first_ms = imu.stamp.last_ms = imu.stamp.handled_ms = tick;
    imu.stamp.sequence = ++sequence;
    imu.stamp.epoch = imu.source_epoch = 1;
    imu.kind = 0x53;
    imu.axes[2] = yaw;
    if (!(missing & 4)) Exec_Imu(&executor, &imu, tick);
    imu.stamp.sequence = ++sequence;
    imu.kind = 0x52;
    imu.axes[2] = gyro;
    if (!(missing & 8)) Exec_Imu(&executor, &imu, tick);
    if (pending_zero && (int32_t)(tick - pending_tick) >= 0 && !(missing & 32))
        Exec_Sent(&executor, pending_id, pending_zero, tick);
    Comm_Tick(tick, key);
    Exec_Run(&executor, tick);
    Select_Run(&zone_switch, tick, select);
    Start_Check(&start_key, &zone_switch, tick);
    Start_Run(&start_key, Comm_Boot(), Comm_Session(), tick, key, (uint8_t)Exec_Ready(&executor, tick));
    if (phase >= START_RELEASE && phase <= START_BEEP && start_key.phase == START_ABORTED)
        start_failure = executor.gate;
}

void Preview_Target(int16_t *rpm)
{
    memcpy(rpm, targets, sizeof(targets));
}

void Preview_Fault(uint32_t reason, uint32_t drop)
{
    fault = reason;
    missing = drop;
}
#else
typedef int Preview_Disabled;
#endif
