#include "mech_group.h"
#include "mech_hw.h"
#include <string.h>

void Group_Init(Group *group, Axis *axis, Servo *servo)
{
    memset(group, 0, sizeof(*group));
    group->axis = axis;
    group->servo = servo;
}

static int Same(const GroupGoal *a, const GroupGoal *b)
{
    unsigned i;
    if (a->id != b->id || a->zero_version != b->zero_version || a->timeout_ms != b->timeout_ms || a->mask != b->mask)
        return 0;
    for (i = 0; i < 5; i++)
        if (a->target[i] != b->target[i] || a->speed[i] != b->speed[i])
            return 0;
    for (i = 0; i < 2; i++)
        if ((a->accel_set ? a->accel[i] : MECH_AXIS_DEFAULT_ACC) !=
            (b->accel_set ? b->accel[i] : MECH_AXIS_DEFAULT_ACC))
            return 0;
    return 1;
}

uint32_t Group_Begin(Group *group, const GroupGoal *goal, uint32_t now)
{
    unsigned i;
    uint32_t result;
    MechGuard *guard = group->axis->guard;
    if (!goal->id || goal->id < group->goal.id)
        return AXIS_ID;
    if (goal->id == group->goal.id)
        return Same(goal, &group->goal) ? MECH_OK : AXIS_ID;
    if (group->state == GROUP_RUNNING)
        return MECH_UNSTABLE;
    if (!goal->mask || goal->mask > 31 || goal->timeout_ms < 1000 || goal->timeout_ms > MECH_SERVO_TIMEOUT_MS)
        return MECH_RANGE;
    result = Mech_Ready(guard, now, goal->zero_version);
    if (result)
        return result;
    /* 全部成员先验证且不产生输出；有一项拒绝就不启动其余成员。 */
    group->axis_mask = 0;
    for (i = 0; i < 5; i++)
    {
        if (!(goal->mask & (1U << i)))
            continue;
        if (i < 2)
        {
            AxisGoal request = {goal->id, goal->zero_version,
                goal->timeout_ms > MECH_MANUAL_TIMEOUT_MS ? MECH_MANUAL_TIMEOUT_MS : goal->timeout_ms,
                goal->target[i], (uint16_t)goal->speed[i], (uint8_t)i, goal->accel[i], goal->accel_set};
            if (!goal->speed[i] || goal->speed[i] > MECH_MANUAL_MAX_RPM || goal->target[i] < 0)
                return MECH_RANGE;
            group->axes[i] = *group->axis;
            group->axes[i].state = AXIS_IDLE;
            group->axes[i].goal.id = 0;
            result = Axis_Manual(&group->axes[i], &request, now);
            if (result == MECH_RANGE && Mech_ManualPulses((uint8_t)i, goal->target[i]) != UINT32_MAX &&
                Mech_ManualPulses((uint8_t)i, goal->target[i]) == (uint32_t)group->axes[i].pulse_position[i])
                group->axes[i].state = AXIS_COMPLETE; /* 已在同一脉冲目标，保留真实静止证据。 */
            else if (result)
                return result;
            else
                group->axis_mask |= (uint8_t)(1U << i);
        }
        else
        {
            ServoGoal request = {goal->id, goal->zero_version, goal->timeout_ms, goal->target[i], (uint8_t)(i - 1), 1, goal->speed[i]};
            if (!goal->speed[i])
                return MECH_RANGE;
            group->servos[i - 2] = *group->servo;
            group->servos[i - 2].state = SERVO_IDLE;
            group->servos[i - 2].goal.id = 0;
            result = Servo_Manual(&group->servos[i - 2], &request, now);
            if (result)
                return result;
        }
    }
    group->goal = *goal;
    group->reason = 0;
    group->sending = 0;
    group->started_ms = now;
    group->stationary_raw[0] = guard->sample[1].value;
    group->stationary_raw[1] = guard->sample[5].value;
    group->state = GROUP_RUNNING;
    guard->concurrent_axes = group->axis_mask;
    return MECH_OK;
}

void Group_Stop(Group *group, uint32_t reason, uint32_t now)
{
    unsigned i;
    if (group->state != GROUP_RUNNING)
        return;
    Mech_Freeze(group->axis->guard, reason, group->axis_mask == 1 ? 1 : group->axis_mask == 2 ? 2 : 0, now);
    group->reason = reason;
    group->state = reason == MECH_MANUAL ? GROUP_CANCELLED : GROUP_FAILED;
    for (i = 0; i < 2; i++)
        if (group->axis_mask & (1U << i))
            Axis_Stop(&group->axes[i], reason, now);
    for (i = 0; i < 3; i++)
        if (group->goal.mask & (1U << (i + 2)))
            Servo_Stop(&group->servos[i], reason, now);
    Mech_Invalidate(group->axis->guard, reason);
    group->axis->guard->stop_required = 1;
}

void Group_Poll(Group *group, uint32_t now, uint8_t base_stopped)
{
    unsigned i;
    uint8_t done = 1;
    MechGuard *guard = group->axis->guard;
    if (group->state != GROUP_RUNNING)
        return;
    Mech_Poll(guard, now, base_stopped);
    if (guard->fault || !guard->zero_valid || !base_stopped || guard->zero_version != group->goal.zero_version ||
        guard->boot != group->axis->boot || guard->session != group->axis->session ||
        (uint32_t)(now - group->started_ms) >= group->goal.timeout_ms)
    {
        Group_Stop(group, guard->fault ? guard->fault : !base_stopped ? MECH_BASE : AXIS_TIMEOUT, now);
        return;
    }
    for (i = 0; i < 2; i++)
        if (!(group->axis_mask & (1U << i)) && (guard->sample[i * 4].value ||
            guard->sample[i * 4 + 1].value - group->stationary_raw[i] > Mech_RawTolerance((uint8_t)i, MECH_MANUAL_TOL_UM) ||
            group->stationary_raw[i] - guard->sample[i * 4 + 1].value > Mech_RawTolerance((uint8_t)i, MECH_MANUAL_TOL_UM)))
        {
            Group_Stop(group, AXIS_OTHER, now);
            return;
        }
    for (i = 0; i < 5; i++)
    {
        if (!(group->goal.mask & (1U << i)))
            continue;
        if (i < 2)
        {
            Axis *a = group->axes + i;
            if (group->axis_mask & (1U << i))
                Axis_Poll(a, now, base_stopped);
            if (a->state >= AXIS_STOPPING && a->state != AXIS_COMPLETE)
            {
                Group_Stop(group, a->reason ? a->reason : MECH_FAULT, now);
                return;
            }
            if (a->state != AXIS_COMPLETE)
                done = 0;
            else if (guard->sample[i * 4].value ||
                     Mech_Position(guard, (uint8_t)i) < group->goal.target[i] - MECH_MANUAL_ENDPOINT_UM ||
                     Mech_Position(guard, (uint8_t)i) > (int64_t)group->goal.target[i] + MECH_MANUAL_ENDPOINT_UM)
            {
                Group_Stop(group, AXIS_TARGET, now);
                return;
            }
        }
        else
        {
            Servo *s = group->servos + i - 2;
            Servo_Poll(s, now, base_stopped);
            if (s->state == SERVO_FAILED || s->state == SERVO_CANCELLED)
            {
                Group_Stop(group, s->reason, now);
                return;
            }
            if (s->state != SERVO_MANUAL_DONE)
                done = 0;
        }
    }
    if (!done)
        return;
    /* 组交接继续采用原300ms静止窗口，不能仅凭最后一条ACK放行下一组。 */
    uint32_t ready = Mech_Ready(guard, now, group->goal.zero_version);
    if (ready == MECH_UNSTABLE)
        return;
    if (ready)
    {
        Group_Stop(group, ready, now);
        return;
    }
    for (i = 0; i < 2; i++)
        if (group->axis_mask & (1U << i))
            group->axis->pulse_position[i] = group->axes[i].pulse_position[i];
    group->axis->zero_version = guard->zero_version;
    group->axis->goal.id = group->servo->goal.id = group->goal.id;
    group->axis->state = AXIS_IDLE;
    group->servo->state = SERVO_MANUAL_DONE;
    guard->concurrent_axes = 0;
    group->state = GROUP_DONE;
}

uint8_t Group_Take(Group *group, BusFrame frames[2], uint32_t now)
{
    unsigned i;
    if (group->state != GROUP_RUNNING || group->sending)
        return 0;
    for (i = 0; i < 2; i++)
        if (group->axis_mask & (1U << i))
        {
            uint8_t count = Axis_Take(group->axes + i, frames, now);
            if (count)
            {
                group->sending = (uint8_t)(i + 1);
                return count;
            }
        }
    return 0;
}

void Group_Sent(Group *group, uint8_t ok, uint32_t now)
{
    if (group->state == GROUP_RUNNING && group->sending)
        Axis_Sent(group->axes + group->sending - 1, group->goal.id, ok, now);
    group->sending = 0;
    if (!ok)
        Group_Stop(group, AXIS_TX, now);
}

void Group_Ack(Group *group, const MotorReply *reply, uint32_t now)
{
    unsigned i;
    if (group->state == GROUP_RUNNING)
        for (i = 0; i < 2; i++)
            if (group->axis_mask & (1U << i))
                Axis_Ack(group->axes + i, reply, now);
}

uint8_t Group_Pwm(Group *group, unsigned index, ServoOutput *output, uint32_t now)
{
    return group->state == GROUP_RUNNING && index < 3 && (group->goal.mask & (1U << (index + 2))) &&
           Servo_Take(group->servos + index, output, now);
}

void Group_Applied(Group *group, unsigned index, const ServoOutput *output, uint8_t ok, uint32_t now)
{
    Servo *s = group->servos + index;
    Servo_Applied(s, output->id, output->sequence, ok, now);
    group->servo->command_mdeg[index] = s->command_mdeg[index];
    group->servo->output_mask |= (uint8_t)(s->output_mask & (1U << index));
    group->servo->confirmed_mask &= (uint8_t)~(1U << index);
}
