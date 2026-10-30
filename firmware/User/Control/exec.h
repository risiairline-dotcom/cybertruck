#ifndef EXEC_H
#define EXEC_H

#include "track.h"

enum
{
    EXEC_DISABLED,
    EXEC_OBSERVING,
    EXEC_ARMING,
    EXEC_READY,
    EXEC_ACTIVE,
    EXEC_STOPPING,
    EXEC_LOCKED
};
enum
{
    EXEC_CONFIG = 32,
    EXEC_ARM = 33,
    EXEC_GOAL = 34,
    EXEC_STATE = 35
};

/* 显式下发的短段试验参数；没有车辆默认值或已标定标志。 */
typedef struct
{
    uint32_t version;
    KinGeometry geometry;
    float counts_per_rev;
    int32_t yaw_sign;
    float angle_scale_rad;
    float gyro_scale_rad_s;
    float max_rpm;
    uint8_t address[4];
    int8_t command_sign[4];
    int8_t feedback_sign[4];
} ExecConfig;

typedef struct
{
    int (*ready)(uint32_t now_ms, int enabled, uint32_t since_ms);
    int (*enable)(unsigned wheel);
    int (*start)(void);
    int (*send)(const int16_t rpm[4], uint32_t id, uint32_t zero_id);
    void (*stop)(void);
    uint32_t (*fault)(void);
    int (*stopped)(uint32_t since_ms);
    int (*rearm)(void);
    /* 四轮相对当前实时位置的小步，圈数已含指令极性；同一票据只调用一次，TXOK后回调Exec_Sent。 */
    int (*position)(const double revolutions[4], uint32_t rpm, uint32_t id, uint32_t token);
} ExecIo;

typedef struct
{
    uint32_t valid, fault, source, now_ms, action_id, kind, address;
    RxStamp frame, previous;
    uint32_t latest_ms[5]; /* 四轮位置及IMU航向的最新接收时刻。 */
    uint32_t yaw_fault, source_epoch;
    int64_t value, previous_value;
} ExecFault;

/* 准备检查失败证据；只读快照，不替代停止或动作结果。 */
typedef struct
{
    uint32_t mask, now_ms, pose_ms, yaw_ms, gyro_ms;
    uint32_t position_ms[4], speed_ms[4];
} ExecGate;

typedef struct
{
    ExecConfig config;
    ExecIo io;
    Odom odom;
    Track track;
    TrackObservation observation;
    TrackGoal last_goal;
    ImuSample angle, gyro;
    MotorSample position[4], speed[4], status[4];
    uint32_t mode, fault, tick_ms, arm_ms, stop_ms, control_ms;
    uint32_t have_goal, enabled_wheels, stop_confirmed;
    uint32_t route_zone; /* version9在PC0准备前绑定，启动后不可修改。 */
    uint32_t position_token; /* 已交给io.position的小步票据，防止重复下发。 */
#ifdef EXEC_PREVIEW
    /* 仅主机模型拥有此缓冲；板端编译和串口协议没有动态装载入口。 */
    uint32_t preview_count, preview_zone;
    uint8_t preview_refs[64][88];
#endif
    ExecFault first_fault; /* 只锁存首次里程计故障，普通查询/STOP不清除。 */
    ExecGate gate;
} Exec;

void Exec_Init(Exec *exec, ExecIo io);
int Exec_Config(Exec *exec, const ExecConfig *config, uint32_t now_ms);
int Exec_Select(Exec *exec, uint32_t zone);
#ifdef EXEC_PREVIEW
int Exec_Preview(Exec *exec, uint32_t zone, const TrackGoal *goals, uint32_t count);
#endif
int Exec_Arm(Exec *exec, uint32_t now_ms);
int Exec_Ready(Exec *exec, uint32_t now_ms);
int Exec_Goal(Exec *exec, const TrackGoal *goal, uint32_t now_ms);
void Exec_Motor(Exec *exec, const MotorSample *sample, uint32_t now_ms);
void Exec_Imu(Exec *exec, const ImuSample *sample, uint32_t now_ms);
void Exec_Run(Exec *exec, uint32_t now_ms);
void Exec_Stop(Exec *exec, uint32_t reason, uint32_t now_ms);
void Exec_Sent(Exec *exec, uint32_t id, uint32_t zero_id, uint32_t now_ms);

#endif
