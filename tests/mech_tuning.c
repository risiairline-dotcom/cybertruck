/* 复用合成反馈，验证PE11重复操作、指令速度和手动扩展行程。 */
#define main ExistingAxisMain
#include "mech_axis.c"
#undef main
#include "mech_servo.h"

static Servo servo;
static unsigned outputs;
static void Setup(void)
{
    int32_t initial[3]={0,180000,98000};
    Init();Servo_Init(&servo,&guard,0);
    CHECK(!Servo_Seed(&servo,initial,now,1));
}
static void ServoTick(void)
{
    ServoOutput output;
    now+=20;
    Feed(0,0x35,0);Feed(0,0x36,lift);Feed(0,0x3A,3);Feed(0,0x33,lift);
    Feed(1,0x35,0);Feed(1,0x36,reach);Feed(1,0x3A,3);Feed(1,0x33,reach);
    Mech_Poll(&guard,now,1);
    if(Servo_Take(&servo,&output,now))
    {
        int32_t previous=servo.command_mdeg[output.channel-1];
        uint32_t rate=servo.goal.speed_mdeg_s?servo.goal.speed_mdeg_s:45000;
        CHECK(llabs((int64_t)output.command_mdeg-previous)<=rate*20/1000);
        CHECK(output.pulse_us==Mech_Pulse(output.channel,output.command_mdeg));
        outputs++;Servo_Applied(&servo,output.id,output.sequence,1,now);
    }
}
static uint32_t Move(uint32_t id,uint8_t channel,int32_t target,uint32_t rate)
{
    ServoGoal request={id,1,300000,target,channel,1,rate};
    uint32_t start=now;
    CHECK(!Servo_Manual(&servo,&request,now));
    for(unsigned i=0;i<15000 && servo.state!=SERVO_MANUAL_DONE;i++)ServoTick();
    CHECK(servo.state==SERVO_MANUAL_DONE&&!guard.fault);
    CHECK(servo.command_mdeg[channel-1]==target&&!Servo_Reference(&servo,channel));
    return now-start;
}
#ifndef TUNING_MAIN
#define TUNING_MAIN main
#endif
int TUNING_MAIN(int argc,char **argv)
{
    unsigned which=argc>1?(unsigned)atoi(argv[1]):0;
    Setup();
    if(which==0)
    {
        const int32_t angles[]={175000,180000,150000,90000,180000,83000,175000,180000};
        for(unsigned i=0;i<8;i++)Move(i+1,2,angles[i],45000);
        CHECK(servo.output_mask==2&&outputs);
    }
    else if(which==1)
    {
        uint32_t slow=Move(1,1,90000,30000);
        Setup();uint32_t fast=Move(1,1,90000,120000);
        CHECK(slow>3600&&slow<4300&&fast>1350&&fast<2500&&fast<slow);
        ServoGoal conflict=servo.goal;conflict.speed_mdeg_s=60000;
        CHECK(Servo_Manual(&servo,&conflict,now)==SERVO_ID);
    }
    else if(which==2)
    {
        uint32_t duration=Move(1,1,270000,1000);
        CHECK(duration>=270600&&duration<271000);
        ServoGoal bad={2,1,300000,0,1,1,999};
        CHECK(Servo_Manual(&servo,&bad,now)==MECH_RANGE);
        bad.speed_mdeg_s=360001;CHECK(Servo_Manual(&servo,&bad,now)==MECH_RANGE);
    }
    else if(which==3 || which==4)
    {
        goal.target_um=100000;goal.rpm=300;goal.timeout_ms=30000;
        CHECK(Axis_Begin(&axis,&goal,now,1,180000)==MECH_RANGE);
        CHECK(!Axis_Manual(&axis,&goal,now)&&guard.manual_range);
        if(which==4)
        {
            now+=251;Axis_Poll(&axis,now,1);
            CHECK(guard.fault&&axis.state==AXIS_STOPPING);return 0;
        }
        BusFrame frames[2];CHECK(Axis_Take(&axis,frames,now)==2);
        Axis_Sent(&axis,goal.id,1,++now);Ack(2);Arrive();
        CHECK(axis.state==AXIS_COMPLETE&&Mech_Position(&guard,0)>99000&&guard.zero_valid);
        for(unsigned i=0;i<6;i++)Round();
        goal.id++;goal.target_um=0;CHECK(!Axis_Manual(&axis,&goal,now));
    }
    else if(which==5)
    {
        goal.target_um=100000;goal.rpm=3001;goal.timeout_ms=30000;
        CHECK(Axis_Manual(&axis,&goal,now)==MECH_RANGE);
        goal.rpm=1000;CHECK(!Axis_Manual(&axis,&goal,now));
        BusFrame frames[2];CHECK(Axis_Take(&axis,frames,now)==2);
        Axis_Sent(&axis,goal.id,1,++now);Ack(2);
        Feed(0,0x35,1100);Axis_Poll(&axis,now,1);CHECK(axis.state==AXIS_RUNNING);
        Feed(0,0x35,1400);Axis_Poll(&axis,now,1);CHECK(axis.state==AXIS_STOPPING&&guard.fault);
    }
    else if(which==6)
    {
        const int32_t angles[]={0,270000,0,180000,30000,270000};
        for(unsigned i=0;i<6;i++)Move(i+1,2,angles[i],120000);
        CHECK(servo.output_mask==2&&Mech_Pulse(2,0)==2500&&Mech_Pulse(2,270000)==996);
        ServoGoal bad={7,1,30000,-1,2,1,45000};
        CHECK(Servo_Manual(&servo,&bad,now)==MECH_RANGE);
        bad.target_mdeg=359001;CHECK(Servo_Manual(&servo,&bad,now)==MECH_RANGE);
        bad.target_mdeg=0;CHECK(Servo_Begin(&servo,&bad,now)==MECH_RANGE);
        bad.target_mdeg=270000;CHECK(Servo_Begin(&servo,&bad,now)==MECH_RANGE);
    }
    else if(which==7)
    {
        int32_t initial[3]={0,0,98000};
        Servo_Init(&servo,&guard,0);
        CHECK(Servo_Seed(&servo,initial,now,1)==MECH_RANGE);
        CHECK(!Servo_SeedManual(&servo,initial,now,1)&&!servo.output_mask);
        Move(1,2,270000,120000);
        Servo_Init(&servo,&guard,0);initial[1]=270000;
        CHECK(!Servo_SeedManual(&servo,initial,now,1)&&!servo.output_mask);
        Servo_Init(&servo,&guard,0);initial[1]=359001;
        CHECK(Servo_SeedManual(&servo,initial,now,1)==MECH_RANGE);
    }
    else if(which==8)
    {
        /* 手动调试同样受机械行程限制：升降140mm、伸缩107mm。 */
        goal.rpm=1000;goal.timeout_ms=300000;
        goal.target_um=140001;CHECK(Axis_Manual(&axis,&goal,now)==MECH_RANGE);
        goal.axis=1;goal.target_um=107001;CHECK(Axis_Manual(&axis,&goal,now)==MECH_RANGE);
        CHECK(Mech_ManualPulses(0,140001)==UINT32_MAX&&Mech_ManualPulses(1,107001)==UINT32_MAX);
        CHECK(Mech_ManualPulses(0,140000)!=UINT32_MAX&&Mech_ManualPulses(1,107000)!=UINT32_MAX);
        goal.target_um=107000;CHECK(!Axis_Manual(&axis,&goal,now));
    }
    else CHECK(0);
    return 0;
}
