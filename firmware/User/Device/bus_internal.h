/* 复用CARCAN13已核对CAN链；本阶段仅改名、归类和增加接收序号。 */
#ifndef BUS_INTERNAL_H
#define BUS_INTERNAL_H

#include "bus.h"
#include "can_internal.h"

/* 仅底层实现与软件夹具使用。上层动作必须走chassis.h的控制权检查，
 * 单轮诊断必须走Test状态机；不向业务层公开直接驱动原语。 */
void Bus_Setup(const uint8_t address[4]);
int Bus_Start(void);
void Bus_End(void);
int Bus_Speed(const int16_t rpm[4]);
/* 活动流内四轮FD（模式02，相对当前实时位置），与速度共用最新快照和整批发送，期间查询照常。 */
int Bus_Position(const uint8_t dir[4], uint16_t rpm, const uint32_t pulses[4]);
void Bus_Stop(void);
int Bus_Rearm(void);
uint8_t Bus_Send(const BusFrame *frames, uint8_t count);
/* 四轮有限FD软件批次：先锁存全部目标，逐轮完整发送；不是硬件同步。 */
uint8_t Bus_Finite(const uint8_t dir[4], uint16_t rpm, const uint32_t pulses[4]);
CanState Bus_Status(void);
void Bus_Cancel(void);
uint8_t Bus_Read(BusFrame *frame);
void Bus_Drop(void);
void Bus_Release(void);
const BusWheel *Bus_Wheels(void);
/* 机构接入后保留ID5/6停止地址；只由唯一板级适配器调用。 */
void Bus_MechWatch(void);
void Bus_HoldWatch(uint8_t enabled);
uint8_t Bus_MechDone(void);
uint8_t Bus_MechGroup(uint8_t axes); /* 仅登记联调组内各轴一次FD授权，不清运动不确定性。 */
void Bus_Observe(void (*receive)(const BusFrame *frame));

#endif
