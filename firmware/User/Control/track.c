#include "track.h"
#include <math.h>
#include <string.h>

#define TRACK_HALF 0x80000000UL

uint32_t Track_ConfigBytes(void)
{
    return (uint32_t)sizeof(TrackConfig);
}

static void Publish(Track *track)
{
    track->output.output_id = ++track->next_output;
    if (!track->output.output_id)
    {
        track->output.output_id = ++track->next_output;
    }
}

static void Zero(Track *track)
{
    unsigned index;
    for (index = 0; index < 4; index++)
    {
        track->output.rpm[index] = 0;
        track->slew[index] = 0;
    }
    track->output.micro_hold = 0;
}

static int Fail(Track *track, unsigned reason)
{
    Zero(track);
    /* 失败也发布新零速版本，消费者不能因旧序号而漏掉停止输出。 */
    Publish(track);
    track->output.state = TRACK_FAILED;
    track->output.fault = reason;
    track->output.done_ms = 0;
    track->stabilizing = 0;
    return -1;
}

static int Config(const TrackConfig *config)
{
    float rpm[4];
    KinBody zero = {0};
    double values[16];
    unsigned index;
    if (!config || !Kin_Inverse(config->geometry, zero, rpm))
    {
        return 0;
    }
    if (!isfinite(config->lag_ff_s) || config->lag_ff_s < 0 || config->lag_ff_s > .15)
        return 0;
    if (config->micro_steps > 1 || (config->micro_steps &&
        (config->calibration != 10 || config->max_rpm != 6))) return 0;
    values[0] = config->kp_xy;
    values[1] = config->kp_yaw;
    values[2] = config->kd_gyro;
    values[3] = config->correction_m_s;
    values[4] = config->correction_rad_s;
    values[5] = config->max_rpm;
    values[6] = config->slew_rpm_s;
    values[7] = config->position_tol_m;
    values[8] = config->yaw_tol_rad;
    values[9] = config->still_rpm;
    values[10] = config->still_m_s;
    values[11] = config->still_rad_s;
    for (index = 0; index < 4; index++)
    {
        values[12 + index] = config->count_drift[index];
    }
    for (index = 0; index < 16; index++)
    {
        if (!isfinite(values[index]) || values[index] < 0)
        {
            return 0;
        }
    }
    if (config->correction_m_s <= 0 || config->correction_rad_s <= 0 ||
        config->max_rpm < 1 || config->max_rpm > 3000 || floor(config->max_rpm) != config->max_rpm ||
        config->slew_rpm_s <= 0 || config->position_tol_m <= 0 || config->yaw_tol_rad <= 0 ||
        config->still_rpm <= 0 || config->still_m_s <= 0 || config->still_rad_s <= 0 ||
        !config->fresh_ms || config->fresh_ms > 250 || !config->feedback_gap_ms ||
        config->feedback_gap_ms > config->fresh_ms || !config->control_gap_ms ||
        config->control_gap_ms > config->fresh_ms || !config->stable_ms ||
        config->stable_ms >= TRACK_HALF || !config->can_epoch || !config->imu_epoch ||
        !config->imu_source_epoch || !config->calibration)
    {
        return 0;
    }
    for (index = 0; index < 4; index++)
    {
        if (config->count_drift[index] <= 0)
        {
            return 0;
        }
    }
    return 1;
}

static int SameLimit(CurveLimit first, CurveLimit last)
{
    return first.velocity == last.velocity && first.acceleration == last.acceleration &&
        first.jerk == last.jerk;
}

static int SameGoal(const TrackGoal *first, const TrackGoal *last)
{
    const RefSpec *a = &first->reference;
    const RefSpec *b = &last->reference;
    if (first->id != last->id || first->timeout_ms != last->timeout_ms || a->kind != b->kind ||
        a->x_m != b->x_m || a->y_m != b->y_m || a->yaw_rad != b->yaw_rad || a->tangent != b->tangent ||
        a->path_heading_rad != b->path_heading_rad || a->curve_velocity_m_s != b->curve_velocity_m_s ||
        a->yaw_begin_m != b->yaw_begin_m || a->yaw_end_m != b->yaw_end_m ||
        a->tail_velocity_m_s != b->tail_velocity_m_s ||
        !SameLimit(a->angular, b->angular) || (a->kind != 2 && !SameLimit(a->linear, b->linear)))
    {
        return 0;
    }
    return (a->kind != 3 && a->kind != 4) || (a->path.lead_m == b->path.lead_m && a->path.tail_m == b->path.tail_m &&
        a->path.radius_m == b->path.radius_m && a->path.blend_m == b->path.blend_m &&
        a->path.turn == b->path.turn && a->path.reverse == b->path.reverse);
}

/* 微调只接受单轴1～5mm平移，或0.1°～3°纯转向（修正粗停航向，夹爪偏置下航向误差会变成横向误差）。 */
static int MicroGoal(const RefSpec *reference)
{
    double distance = hypot(reference->x_m, reference->y_m);
    if (reference->kind == 1)
    {
        return reference->yaw_rad == 0 && (reference->x_m == 0 || reference->y_m == 0) &&
            distance >= .000999 && distance <= .005001;
    }
    return reference->kind == 2 && distance == 0 &&
        fabs(reference->yaw_rad) >= .0017 && fabs(reference->yaw_rad) <= .05236;
}

static int Stamp(const Track *track, const RxStamp *sample, const RxStamp *previous,
                 double value, double old_value, uint32_t epoch, uint32_t now)
{
    uint32_t elapsed;
    if (sample->epoch != epoch)
    {
        return TRACK_SOURCE;
    }
    if (!isfinite(value) || !Stamp_Check(sample, now, track->config.fresh_ms))
    {
        return TRACK_STALE;
    }
    if (!previous)
    {
        return TRACK_OK;
    }
    if (sample->sequence == previous->sequence)
    {
        return sample->first_ms == previous->first_ms && sample->last_ms == previous->last_ms &&
            sample->handled_ms == previous->handled_ms && value == old_value ? TRACK_OK : TRACK_TIME;
    }
    if (!Stamp_Next(previous, sample))
    {
        return TRACK_TIME;
    }
    elapsed = sample->last_ms - previous->last_ms;
    if (!elapsed && value != old_value)
    {
        return TRACK_TIME;
    }
    return elapsed <= track->config.feedback_gap_ms ? TRACK_OK : TRACK_STALE;
}

static int Observe(const Track *track, const TrackObservation *sample, uint32_t now, unsigned compare)
{
    const TrackObservation *old = compare ? &track->previous : 0;
    unsigned index;
    int result;
    if (!sample || sample->pose.valid != 1 || sample->pose.fault ||
        sample->pose.calibration != track->config.calibration ||
        sample->imu_source_epoch != track->config.imu_source_epoch)
    {
        return TRACK_SOURCE;
    }
    if (!isfinite(sample->pose.x_m) || !isfinite(sample->pose.y_m) ||
        !isfinite(sample->pose.yaw_rad) || (uint32_t)(now - sample->pose.tick_ms) >= track->config.fresh_ms)
    {
        return TRACK_STALE;
    }
    if (old && (sample->pose.origin_ms != old->pose.origin_ms ||
        (uint32_t)(sample->pose.tick_ms - old->pose.tick_ms) >= TRACK_HALF ||
        (sample->pose.tick_ms == old->pose.tick_ms &&
         (sample->pose.x_m != old->pose.x_m || sample->pose.y_m != old->pose.y_m ||
          sample->pose.yaw_rad != old->pose.yaw_rad || sample->pose.steps != old->pose.steps))))
    {
        return TRACK_TIME;
    }
    for (index = 0; index < 4; index++)
    {
        if (fabs(sample->rpm[index]) > 5000 || fabs(sample->counts[index]) > 4294967295.0)
        {
            return TRACK_FEEDBACK;
        }
        result = Stamp(track, &sample->position[index], old ? &old->position[index] : 0,
                       sample->counts[index], old ? old->counts[index] : 0, track->config.can_epoch, now);
        if (result)
        {
            return result;
        }
        result = Stamp(track, &sample->speed[index], old ? &old->speed[index] : 0,
                       sample->rpm[index], old ? old->rpm[index] : 0, track->config.can_epoch, now);
        if (result)
        {
            return result;
        }
    }
    result = Stamp(track, &sample->yaw_stamp, old ? &old->yaw_stamp : 0,
                   sample->yaw_rad, old ? old->yaw_rad : 0, track->config.imu_epoch, now);
    if (result)
    {
        return result;
    }
    return Stamp(track, &sample->gyro_stamp, old ? &old->gyro_stamp : 0,
                 sample->gyro_rad_s, old ? old->gyro_rad_s : 0, track->config.imu_epoch, now);
}

static int Still(const Track *track, const TrackObservation *sample)
{
    float rpm[4];
    KinBody body;
    unsigned index;
    for (index = 0; index < 4; index++)
    {
        if (fabs(sample->rpm[index]) > track->config.still_rpm)
        {
            return 0;
        }
        rpm[index] = (float)sample->rpm[index];
    }
    return Kin_Forward(track->config.geometry, rpm, &body) &&
        hypot(body.vx_m_s, body.vy_m_s) <= track->config.still_m_s &&
        fabs(body.wz_rad_s) <= track->config.still_rad_s &&
        fabs(sample->gyro_rad_s) <= track->config.still_rad_s;
}

static double Seconds(const Track *track, uint32_t tick)
{
    uint32_t elapsed = tick - track->start_ms;
    return elapsed < TRACK_HALF ? elapsed / 1000.0 : 0;
}

int Track_Reference(const Track *track, uint32_t tick_ms, RefPoint *point)
{
    return track && track->output.id && Ref_At(&track->plan, Seconds(track, tick_ms), point);
}

size_t Track_Bytes(void)
{
    return sizeof(Track);
}

int Track_Init(Track *track, const TrackConfig *config)
{
    TrackConfig input;
    if (!track)
    {
        return 0;
    }
    if (config)
    {
        input = *config;
    }
    memset(track, 0, sizeof(*track));
    track->output.state = TRACK_FAILED;
    track->output.fault = TRACK_CONFIG;
    if (!config || !Config(&input))
    {
        return 0;
    }
    track->config = input;
    track->output.state = TRACK_IDLE;
    track->output.fault = TRACK_OK;
    return 1;
}

int Track_Start(Track *track, const TrackGoal *goal,
                const TrackObservation *observation, uint32_t now_ms)
{
    if (!track || !goal || !goal->id || track->output.fault ||
        track->output.state == TRACK_CANCELLED)
    {
        return -1;
    }
    if (track->output.id == goal->id)
    {
        return SameGoal(goal, &track->goal) ? 0 : -2;
    }
    if (track->output.state == TRACK_ACCEPTED || track->output.state == TRACK_RUNNING ||
        track->output.state == TRACK_SETTLING)
    {
        return -3;
    }
    if ((track->output.id && (uint32_t)(goal->id - track->output.id) >= TRACK_HALF) ||
        !goal->timeout_ms || goal->timeout_ms >= TRACK_HALF ||
        Observe(track, observation, now_ms, 0) || !Still(track, observation) ||
        !Ref_Make(&track->candidate, &goal->reference, observation->pose.x_m,
                  observation->pose.y_m, observation->pose.yaw_rad) ||
        track->candidate.duration_s * 1000 + track->config.stable_ms >= goal->timeout_ms)
    {
        return -1;
    }
    if (track->config.micro_steps && !MicroGoal(&goal->reference))
    {
        return -1;
    }
    if (track->config.micro_steps && !track->micro_heading_valid)
    {
        track->micro_heading = observation->yaw_rad;
        track->micro_heading_valid = 1;
    }
    if (track->config.micro_steps && goal->reference.kind == 2)
    {
        track->micro_heading = observation->yaw_rad + goal->reference.yaw_rad;
    }
    track->plan = track->candidate;
    track->goal = *goal;
    memset(&track->output, 0, sizeof(track->output));
    track->output.id = goal->id;
    track->output.state = TRACK_ACCEPTED;
    track->start_ms = track->last_ms = now_ms;
    track->previous = *observation;
    track->have_previous = 1;
    track->have_zero_ack = track->stabilizing = 0;
    track->position_active = track->yaw_active = 0;
    track->micro_phase = track->micro_begin_ms = track->micro_duration_ms = 0;
    track->micro_token = track->micro_requested_ms = track->micro_acked = 0;
    Zero(track);
    return 1;
}

static double Clamp(double value, double limit)
{
    return value < -limit ? -limit : value > limit ? limit : value;
}

/* 四轮位置与速度反馈都在指定时刻之后收到。 */
static int FeedbackAfter(const TrackObservation *sample, uint32_t tick)
{
    unsigned index;
    for (index = 0; index < 4; index++)
    {
        uint32_t position = sample->position[index].last_ms - tick;
        uint32_t speed = sample->speed[index].last_ms - tick;
        if (!position || position >= TRACK_HALF || !speed || speed >= TRACK_HALF)
        {
            return 0;
        }
    }
    return 1;
}

/* 本步已沿指令方向走过至少一半：驱动器起步迟滞时估算时间到、车轮仍静止不能当作已执行，
 * 否则零速命令会打断尚未开始的位置小步。 */
static int MicroProgress(const Track *track, const TrackObservation *sample)
{
    double moved;
    if (track->micro_axis == 2)
    {
        moved = sample->yaw_rad - track->micro_origin[2];
    }
    else
    {
        double heading = track->micro_origin[2] + (track->micro_axis == 1 ? 1.5707963267948966 : 0);
        moved = cos(heading) * (sample->pose.x_m - track->micro_origin[0]) +
            sin(heading) * (sample->pose.y_m - track->micro_origin[1]);
    }
    return moved * track->micro_sign >= track->micro_amount / 2;
}

/* 毫米级标定不在旧反馈上连续换向。每次只选一个车体系平移轴或纯转向，
 * 交给驱动器按相对当前实时位置执行四轮同幅小步，位移不再取决于低速死区和启停滞后。
 * 驱动器执行期间不发速度、反馈查询继续；估算时间到且新反馈静止后发零速TXOK，
 * 再经150ms新反馈稳定窗口。是否到位仍完全由编码器/IMU决定，未到位按新误差再走一步。 */
static int MicroCommand(Track *track, const TrackObservation *sample, uint32_t now)
{
    double dx = track->plan.target[0] - sample->pose.x_m;
    double dy = track->plan.target[1] - sample->pose.y_m;
    double angle = track->micro_heading - sample->yaw_rad;
    double c = cos(sample->yaw_rad), s = sin(sample->yaw_rad);
    double x = c * dx + s * dy, y = -s * dx + c * dy;
    double lever_m = track->config.geometry.half_length_m + track->config.geometry.half_width_m;
    double circumference_m = 2 * 3.14159265358979323846 * track->config.geometry.radius_m;
    double budget_m, sign, rpm, ceiling, revolutions;
    static const int pattern[3][4] = {{1,1,1,1},{-1,1,1,-1},{-1,1,-1,1}};
    unsigned axis, i;
    if (!Track_Reference(track, now, &track->output.reference)) return TRACK_REFERENCE;
    track->output.reference.yaw_rad = track->micro_heading;
    if (track->micro_phase == 1 && track->micro_token && !track->micro_acked &&
        (uint32_t)(now-track->micro_requested_ms) >= track->config.fresh_ms) return TRACK_TIMEOUT;
    if (track->micro_phase == 1 && track->micro_acked &&
        (uint32_t)(now-track->micro_begin_ms) > track->micro_duration_ms * 3 + 500) return TRACK_TIMEOUT;
    track->output.position_error_m = hypot(dx,dy);
    track->output.yaw_error_rad = fabs(angle);
    track->output.scale = 1;
    if (!isfinite(x) || !isfinite(y) || !isfinite(angle)) return TRACK_FEEDBACK;
    track->position_active = track->yaw_active = 0;
    if (track->micro_phase == 0)
    {
        /* 航向先于平移：夹爪中心与车体中心不重合，残余航向会在夹爪处放大为横向误差。 */
        if (fabs(angle) > track->config.yaw_tol_rad)
        {
            axis = 2; sign = angle < 0 ? -1 : 1;
            budget_m = fabs(angle) * lever_m;
        }
        else if (hypot(dx,dy) > track->config.position_tol_m)
        {
            axis = fabs(x) >= fabs(y) ? 0 : 1;
            sign = (axis == 0 ? x : y) < 0 ? -1 : 1;
            budget_m = fabs(axis == 0 ? x : y);
        }
        else
        {
            track->micro_phase = 3;
            budget_m = 0; axis = 0; sign = 0;
        }
        if (track->micro_phase != 3)
        {
            /* 平移2RPM、纯转向1RPM，仍受整轮限幅和纠偏速度上限约束。 */
            ceiling = (axis == 2 ? track->config.correction_rad_s * lever_m : track->config.correction_m_s) /
                (circumference_m / 60);
            rpm = fmin(axis != 2 ? 2 : 1, floor(fmin(ceiling, track->config.max_rpm)));
            if (rpm < 1) return TRACK_REFERENCE;
            /* 单步轮缘位移不超过6mm；超出部分按新反馈下一步再走，不一次追完。 */
            revolutions = fmin(budget_m, .006) / circumference_m;
            track->micro_duration_ms = (uint32_t)fmax(40, revolutions / (rpm / 60) * 1000);
            track->micro_origin[0] = sample->pose.x_m;
            track->micro_origin[1] = sample->pose.y_m;
            track->micro_origin[2] = sample->yaw_rad;
            track->micro_axis = axis;
            track->micro_sign = sign;
            track->micro_amount = fmin(budget_m, .006) / (axis == 2 ? lever_m : 1);
            track->micro_begin_ms = 0;
            track->micro_token = track->micro_acked = 0;
            track->micro_phase = 1;
            track->output.micro_rpm = (uint32_t)rpm;
            track->output.micro_hold = 1;
            for (i=0;i<4;i++) track->output.micro_rev[i] = sign * pattern[axis][i] * revolutions;
        }
    }
    if (track->micro_phase == 1 && track->micro_acked &&
        (uint32_t)(now-track->micro_begin_ms) >= track->micro_duration_ms &&
        FeedbackAfter(sample, track->micro_begin_ms) && Still(track, sample) && MicroProgress(track, sample))
    {
        track->micro_phase = 2;
        track->output.micro_hold = 0;
    }
    for (i=0;i<4;i++)
    {
        track->output.rpm[i] = 0;
        track->slew[i] = 0;
    }
    return TRACK_OK;
}

static int Command(Track *track, const TrackObservation *sample, uint32_t now, double dt)
{
    RefPoint position;
    RefPoint heading;
    RefPoint gyro;
    KinBody body;
    float ff[4];
    float rpm[4];
    double x;
    double y;
    double norm;
    double angle;
    double correction;
    double cosine = cos(sample->yaw_rad);
    double sine = sin(sample->yaw_rad);
    double change = 0;
    double scale;
    unsigned index;
    if (track->config.micro_steps) return MicroCommand(track, sample, now);
    if (!Track_Reference(track, now, &track->output.reference) ||
        !Track_Reference(track, sample->pose.tick_ms, &position) ||
        !Track_Reference(track, sample->yaw_stamp.last_ms, &heading) ||
        !Track_Reference(track, sample->gyro_stamp.last_ms, &gyro))
    {
        return TRACK_REFERENCE;
    }
    /* 前馈用当前时刻；位置、航向和角速度误差分别回到各自反馈时刻。 */
    x = position.x_m - sample->pose.x_m;
    y = position.y_m - sample->pose.y_m;
    norm = hypot(x, y);
    angle = heading.yaw_rad - sample->yaw_rad;
    track->output.position_error_m = hypot(track->plan.target[0] - sample->pose.x_m,
                                           track->plan.target[1] - sample->pose.y_m);
    track->output.yaw_error_rad = fabs(track->plan.target[2] - sample->yaw_rad);
    if (!isfinite(norm) || !isfinite(angle) || !isfinite(track->output.position_error_m))
    {
        return TRACK_FEEDBACK;
    }
    if (norm > track->config.position_tol_m)
    {
        track->position_active = 1;
    }
    else if (norm <= track->config.position_tol_m / 2)
    {
        track->position_active = 0;
    }
    if (fabs(angle) > track->config.yaw_tol_rad)
    {
        track->yaw_active = 1;
    }
    else if (fabs(angle) <= track->config.yaw_tol_rad / 2)
    {
        track->yaw_active = 0;
    }
    x = track->position_active ? x * track->config.kp_xy : 0;
    y = track->position_active ? y * track->config.kp_xy : 0;
    norm = hypot(x, y);
    if (norm > track->config.correction_m_s)
    {
        x *= track->config.correction_m_s / norm;
        y *= track->config.correction_m_s / norm;
    }
    correction = (track->yaw_active ? track->config.kp_yaw * angle : 0) +
        track->config.kd_gyro * (gyro.wz_rad_s - sample->gyro_rad_s);
    if (!isfinite(correction))
    {
        return TRACK_FEEDBACK;
    }
    body.vx_m_s = (float)(cosine * track->output.reference.vx_m_s + sine * track->output.reference.vy_m_s);
    body.vy_m_s = (float)(-sine * track->output.reference.vx_m_s + cosine * track->output.reference.vy_m_s);
    body.wz_rad_s = (float)track->output.reference.wz_rad_s;
    if (track->config.lag_ff_s > 0)
    {
        /* 对车体系目标速度求导，包含旋转坐标的交叉项；不积分加速度定位。 */
        double ax = cosine * track->output.reference.ax_m_s2 + sine * track->output.reference.ay_m_s2 +
                    track->output.reference.wz_rad_s * body.vy_m_s;
        double ay = -sine * track->output.reference.ax_m_s2 + cosine * track->output.reference.ay_m_s2 -
                    track->output.reference.wz_rad_s * body.vx_m_s;
        body.vx_m_s += (float)(track->config.lag_ff_s * ax);
        body.vy_m_s += (float)(track->config.lag_ff_s * ay);
        body.wz_rad_s += (float)(track->config.lag_ff_s * track->output.reference.angular_accel_rad_s2);
    }
    if (!Kin_Inverse(track->config.geometry, body, ff) || Kin_Limit(ff, (float)track->config.max_rpm) < 1)
    {
        return TRACK_LIMIT;
    }
    body.vx_m_s += (float)(cosine * x + sine * y);
    body.vy_m_s += (float)(-sine * x + cosine * y);
    body.wz_rad_s += (float)Clamp(correction, track->config.correction_rad_s);
    if (!Kin_Inverse(track->config.geometry, body, rpm))
    {
        return TRACK_LIMIT;
    }
    scale = Kin_Limit(rpm, (float)track->config.max_rpm);
    if (scale <= 0)
    {
        return TRACK_LIMIT;
    }
    track->output.scale = scale;
    if (track->output.reference.done && track->output.position_error_m <= track->config.position_tol_m &&
        track->output.yaw_error_rad <= track->config.yaw_tol_rad &&
        !track->position_active && !track->yaw_active)
    {
        for (index = 0; index < 4; index++)
        {
            rpm[index] = 0;
        }
    }
    for (index = 0; index < 4; index++)
    {
        if (fabs(rpm[index] - track->slew[index]) > change)
        {
            change = fabs(rpm[index] - track->slew[index]);
        }
    }
    /* 四轮共用变化比例，最后取整最多引入0.5RPM量化误差。 */
    scale = change > track->config.slew_rpm_s * dt ? track->config.slew_rpm_s * dt / change : 1;
    for (index = 0; index < 4; index++)
    {
        track->slew[index] += scale * (rpm[index] - track->slew[index]);
        track->output.rpm[index] = floor(fabs(track->slew[index]) + .5) * (track->slew[index] < 0 ? -1 : 1);
    }
    return TRACK_OK;
}

static uint32_t Oldest(const TrackObservation *sample, uint32_t now)
{
    uint32_t age = now - sample->pose.tick_ms;
    uint32_t next;
    unsigned index;
    for (index = 0; index < 4; index++)
    {
        next = now - sample->position[index].last_ms;
        if (next > age)
        {
            age = next;
        }
        next = now - sample->speed[index].last_ms;
        if (next > age)
        {
            age = next;
        }
    }
    next = now - sample->yaw_stamp.last_ms;
    if (next > age)
    {
        age = next;
    }
    next = now - sample->gyro_stamp.last_ms;
    if (next > age)
    {
        age = next;
    }
    return now - age;
}

static void Arrival(Track *track, const TrackObservation *sample, uint32_t now)
{
    /* 用最慢反馈的时间推进稳定窗口，反复读取缓存不累计停车时间。 */
    uint32_t tick = Oldest(sample, now);
    unsigned index;
    unsigned zero = 1;
    unsigned capture;
    for (index = 0; index < 4; index++)
    {
        if (track->output.rpm[index] != 0)
        {
            zero = 0;
        }
    }
    capture = track->output.reference.done && track->output.position_error_m <= track->config.position_tol_m &&
        track->output.yaw_error_rad <= track->config.yaw_tol_rad &&
        !track->position_active && !track->yaw_active && zero;
    if (track->config.micro_steps)
        capture = zero && (track->micro_phase == 2 || track->micro_phase == 3);
    track->output.blockers = track->output.reference.done ? 0 : 1;
    if (!capture)
    {
        track->output.blockers |= 2;
        track->output.zero_id = 0;
        track->output.zero_ack_ms = 0;
        track->have_zero_ack = track->stabilizing = 0;
        track->output.stable_elapsed_ms = 0;
        track->output.state = TRACK_RUNNING;
        return;
    }
    if (!track->output.zero_id)
    {
        track->output.zero_id = track->output.output_id;
        track->zero_requested_ms = now;
        track->have_zero_ack = 0;
    }
    track->output.state = TRACK_SETTLING;
    if (!track->have_zero_ack)
    {
        track->output.blockers |= 4;
    }
    if (!track->have_zero_ack || (uint32_t)(tick - track->output.zero_ack_ms) == 0 ||
        (uint32_t)(tick - track->output.zero_ack_ms) >= TRACK_HALF)
    {
        track->output.blockers |= 8;
        track->stabilizing = 0;
        return;
    }
    if (!Still(track, sample))
    {
        track->output.blockers |= 16;
        track->stabilizing = 0;
        track->output.stable_elapsed_ms = 0;
        return;
    }
    if (track->config.micro_steps)
    {
        for (index=0;index<4;index++)
            if (sample->rpm[index] != 0)
            {
                track->output.blockers |= 16;
                track->stabilizing = 0;
                track->output.stable_elapsed_ms = 0;
                return;
            }
    }
    for (index = 0; index < 4; index++)
    {
        if (track->stabilizing && fabs(sample->counts[index] - track->stable_counts[index]) >
            track->config.count_drift[index])
        {
            track->output.blockers |= 32;
            track->stabilizing = 0;
        }
    }
    if (!track->stabilizing)
    {
        track->stable_tick = tick;
        for (index = 0; index < 4; index++)
        {
            track->stable_counts[index] = sample->counts[index];
        }
        track->stabilizing = 1;
    }
    track->output.stable_elapsed_ms = tick - track->stable_tick;
    if (track->output.stable_elapsed_ms < track->config.stable_ms)
    {
        track->output.blockers |= 64;
        return;
    }
    if (track->config.micro_steps &&
        (track->output.position_error_m > track->config.position_tol_m ||
         track->output.yaw_error_rad > track->config.yaw_tol_rad))
    {
        track->micro_phase = 0;
        track->output.zero_id = track->output.zero_ack_ms = 0;
        track->have_zero_ack = track->stabilizing = 0;
        track->output.stable_elapsed_ms = 0;
        track->output.state = TRACK_RUNNING;
        return;
    }
    if (track->config.micro_steps && !track->output.reference.done) return;
    track->output.state = TRACK_COMPLETE;
    track->output.done_ms = now;
}

int Track_Step(Track *track, const TrackObservation *observation, uint32_t now_ms)
{
    uint32_t elapsed;
    int result;
    uint32_t previous_zero;
    if (!track)
    {
        return -1;
    }
    if (track->output.state == TRACK_COMPLETE)
    {
        return 1;
    }
    if (track->output.state != TRACK_ACCEPTED && track->output.state != TRACK_RUNNING &&
        track->output.state != TRACK_SETTLING)
    {
        return -1;
    }
    elapsed = now_ms - track->last_ms;
    if (elapsed >= TRACK_HALF || elapsed > track->config.control_gap_ms)
    {
        return Fail(track, TRACK_TIME);
    }
    result = Observe(track, observation, now_ms, track->have_previous);
    if (result)
    {
        return Fail(track, (unsigned)result);
    }
    if ((uint32_t)(now_ms - track->start_ms) > track->goal.timeout_ms)
    {
        return Fail(track, TRACK_TIMEOUT);
    }
    if (!elapsed)
    {
        return 0;
    }
    previous_zero = track->output.zero_id;
    result = Command(track, observation, now_ms, elapsed / 1000.0);
    if (result)
    {
        return Fail(track, (unsigned)result);
    }
    if (!previous_zero)
    {
        Publish(track);
    }
    if (track->config.micro_steps && track->micro_phase == 1 &&
        !track->micro_token && track->output.micro_hold)
    {
        track->micro_token = track->output.output_id;
        track->micro_requested_ms = now_ms;
    }
    Arrival(track, observation, now_ms);
    if (previous_zero && !track->output.zero_id)
    {
        Publish(track);
    }
    track->previous = *observation;
    track->last_ms = now_ms;
    return track->output.state == TRACK_COMPLETE ? 1 : 0;
}

uint32_t Track_Token(const Track *track)
{
    if (!track) return 0;
    return track->config.micro_steps && track->micro_phase == 1 ?
        track->micro_token : track->output.zero_id;
}

int Track_Sent(Track *track, uint32_t id, uint32_t output_id, uint32_t sent_ms)
{
    if (track && track->config.micro_steps && track->micro_phase == 1 &&
        track->output.state == TRACK_RUNNING && track->output.id == id &&
        output_id && output_id == track->micro_token && !track->micro_acked &&
        (uint32_t)(sent_ms - track->micro_requested_ms) < TRACK_HALF &&
        (uint32_t)(sent_ms - track->start_ms) <= track->goal.timeout_ms)
    {
        track->micro_acked = 1;
        track->micro_begin_ms = sent_ms;
        return 1;
    }
    if (!track || track->output.state != TRACK_SETTLING || track->output.id != id ||
        !output_id || track->output.zero_id != output_id || track->have_zero_ack ||
        (uint32_t)(sent_ms - track->zero_requested_ms) >= TRACK_HALF ||
        (uint32_t)(sent_ms - track->start_ms) > track->goal.timeout_ms)
    {
        return 0;
    }
    track->have_zero_ack = 1;
    track->output.zero_ack_ms = sent_ms;
    return 1;
}

void Track_Cancel(Track *track)
{
    if (track && (track->output.state == TRACK_ACCEPTED || track->output.state == TRACK_RUNNING ||
                  track->output.state == TRACK_SETTLING))
    {
        Zero(track);
        Publish(track);
        track->output.state = TRACK_CANCELLED;
        track->output.done_ms = 0;
        track->stabilizing = 0;
    }
}

void Track_Read(const Track *track, TrackOutput *output)
{
    if (output)
    {
        memset(output, 0, sizeof(*output));
        if (track)
        {
            *output = track->output;
        }
    }
}
