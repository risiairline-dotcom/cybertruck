#ifndef MECH_HW_H
#define MECH_HW_H

/* 核对旧mech_cfg.h及本工程IOC。供校验和离线意图执行，不注册机构收发或PWM。 */
#define MECH_LIFT_ID                 5U
#define MECH_REACH_ID                6U
#define MECH_LIFT_SIGN               1
#define MECH_REACH_SIGN             -1
#define MECH_LIFT_UM_PER_320       4098L
#define MECH_REACH_UM_PER_320     12400L
#define MECH_RAW_PER_320          6554L
/* 自动姿态已核对包络；手动调试受机械行程限制（升降最高为0向下，伸缩最近为0向外，无硬限位）。 */
#define MECH_LIFT_MAX_UM         49000L
#define MECH_REACH_MAX_UM        50000L
#define MECH_LIFT_TRAVEL_UM     140000L
#define MECH_REACH_TRAVEL_UM    107000L
#define MECH_FEEDBACK_AGE_MS       250U
/* 沿用旧axis.c：上一笔查询完成后，再留10ms机构查询间隔。 */
#define MECH_QUERY_GAP_MS           10U
#define MECH_ZERO_STABLE_MS        300U
#define MECH_STABLE_SAMPLES          5U
#define MECH_STILL_RAW              16L
#define MECH_STABLE_RAW              8L
#define MECH_FEEDBACK_TOL_UM       300L
/* 手动联调使用已确认的0.5mm容差；自动姿态和人工建零仍用原严格档。 */
#define MECH_MANUAL_TOL_UM         500L
#define MECH_MANUAL_ENDPOINT_UM   1000L /* 手动单轴/联调终点误差，独立于反向/非执行轴/零点保护。 */
#define MECH_MANUAL_STABLE_MS      100U
#define MECH_MANUAL_SAMPLES          2U

/* 板级输出唯一来源仍为Car.ioc；这里只定义逻辑通道，不引入HAL。 */
#define MECH_TRAY_CHANNEL            1U
#define MECH_ARM_CHANNEL             2U
#define MECH_GRIP_CHANNEL            3U
#define MECH_ARM_RANGE_MDEG      359000L
#define MECH_TRAY_RANGE_MDEG     270000L
/* 默认夹持目标与允许范围分开，复用旧hold98/max100；不自动改写已存目标。 */
#define MECH_GRIP_HOLD_MDEG       98000L
#define MECH_GRIP_MAX_MDEG       100000L
#define MECH_GRIP_RANGE_MDEG     270000L
#if MECH_GRIP_HOLD_MDEG > MECH_GRIP_MAX_MDEG || MECH_GRIP_MAX_MDEG > MECH_GRIP_RANGE_MDEG
#error Gripper hold target exceeds safe range
#endif
#define MECH_PWM_MIN_US            500U
#define MECH_PWM_SPAN_US          2000U
#define MECH_CLEARANCE_UM         1000L
#define MECH_EXTEND_MIN_LIFT_UM  45000L
#define MECH_ARM_CLEAR_MDEG      180000L

/* 复用旧axis.c的受限单轴参数；仅供未绑定IO的执行状态机。 */
#define MECH_AXIS_MAX_RPM            30U
#define MECH_AXIS_DEFAULT_ACC         1U
#define MECH_MANUAL_MAX_RPM        3000U
#define MECH_MANUAL_TIMEOUT_MS    300000U
#define MECH_AXIS_SPEED_LIMIT        33U
#define MECH_AXIS_TARGET_RAW          4L
#define MECH_AXIS_OTHER_RAW          32L
#define MECH_AXIS_ARRIVE_RAW         64L
#define MECH_AXIS_REVERSE_RAW        64L
#define MECH_AXIS_TX_MS              50U
#define MECH_AXIS_ACK_MS            250U
#define MECH_AXIS_ARRIVE_SAMPLES       3U
#define MECH_SERVO_STEP_MS            20U
#define MECH_SERVO_STEP_MDEG         900L
#define MECH_SERVO_SPEED_MIN       1000U
#define MECH_SERVO_SPEED_MAX     360000U
/* 仅PE11初始平滑档；不是舵机实际转速/力矩反馈。 */
#define MECH_SERVO_ACCEL_DEG_S2    180.0f
#define MECH_SERVO_JERK_DEG_S3     720.0f
#define MECH_SERVO_TIMEOUT_MS    420000U
#define MECH_SERVO_WAIT_MS           600U
#define MECH_SERVO_OUTPUT_MS          50U
#define MECH_ARM_MIN_MDEG          83000L
#define MECH_ARM_MAX_MDEG         180000L
#define MECH_MANUAL_ARM_MAX_MDEG  359000L

#endif
