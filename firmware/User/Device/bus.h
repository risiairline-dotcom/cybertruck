/* 复用CARCAN13已核对CAN链；本阶段仅改名、归类和增加接收序号。 */
#ifndef BUS_H
#define BUS_H
#include "can_tx.h"
typedef struct
{
    MotorReply speed, position, status;
    MotorConfig config;
} BusWheel;
typedef struct
{
    uint8_t active, locked, stop_pending, ack_observed_mask, motion_uncertain;
    uint32_t error, queue_depth, replaced, rejected;
    uint32_t speed_sequence, speed_sent_sequence, query_count, speed_span_ms;
    /* 完整四轮TXOK批次计数，仅诊断；不能用目标sequence跳号冒充实发漏样。 */
    uint32_t speed_batches;
    /* 整批四轮各自TXOK后冻结，同speed_sent_sequence对应。signed协议RPM；
     * 只读诊断，不改变调度，不表示四轮同时发送或电机已执行。 */
    int16_t sent_rpm[4];
    uint32_t sent_tick;
    uint32_t sent_wheel_tick[4]; /* 同一完整批次每轮TXOK时间，不是物理起动时间。 */
    uint32_t finite_sequence, finite_sent_sequence, finite_sent_mask;
    uint32_t rx_overflow, stop_retries, cancel_count;
    uint32_t pending_stop_count, wait_ms;
    /* 新Stop地址保留序号；全部地址TXOK后才更新sent，两者均非物理停车。 */
    uint32_t stop_sequence, stop_sent_sequence;
    uint32_t query_waiting, query_address, query_code, query_tx_sequence;
    uint32_t query_replies, query_timeouts, query_reply_tick;
    /* 四轮相对位置小步整批TXOK：与speed_sequence共用序号，不改sent_rpm；不表示已到位。 */
    uint32_t position_sent_sequence, position_sent_tick;
} BusDiag;
enum
{
    BUS_ERROR_STOP = 1U,
    BUS_ERROR_CAN = 2U,
    BUS_ERROR_RX = 4U,
    BUS_ERROR_DRIVER = 8U,
    BUS_ERROR_REPLY = 16U
};
extern volatile BusDiag bus_diag;
/* 主循环唯一CAN接收消费者；业务层不得调用私有Bus/Can运动原语。 */
void Bus_Poll(void);

/* 首次查询缺回包现场，独立于CAN硬件错误历史；仅复位清空，不改变保护。 */
typedef struct
{
    uint32_t version, valid, tick, address, code;
    uint32_t start, barrier, tx_sequence, tx_done_tick, tx_done_sequence, rx_sequence;
    /* 记录Fault()之前的总线状态；query_timeouts已包含本次超时。 */
    uint32_t error, active, locked, job, tx_state, query_count, query_timeouts;
    uint32_t last_rx_id, last_rx_length, last_rx_data0, last_rx_data1, last_rx_tick;
} BusTimeout;
extern volatile BusTimeout bus_timeout;
#endif
