/* 正式Base/MechPort/Bus/Comm/Exec；传感器、驱动与PWM由合成HAL提供。 */
#define QUERY_ALLOW_PWM
#define QUERY_SCHEDULE_MAIN PreviousScheduleMain
#include "mech_query_schedule.c"

static unsigned request_counter;
static void SessionPacket(unsigned type, unsigned value)
{
    Frame frame = {0}; uint8_t wire[160], reply[160];
    frame.type=(uint8_t)type; frame.boot=123; frame.session=456;
    frame.request=++request_counter; frame.ttl_ms=250; frame.size=4;
    Frame_Put(frame.data,value,4); unsigned size=Frame_Encode(&frame,wire);
    for(unsigned i=0;i<size;i++)Comm_Feed(wire[i],tick,tick);
    CHECK(Comm_Read(reply)>0);
}

static void WheelReply(unsigned address, unsigned code, int64_t value)
{
    memset(&incoming,0,sizeof(incoming));incoming.id=address<<8;
    incoming.len=code==0x35?5:code==0x36?7:3;incoming.data[0]=(uint8_t)code;
    uint32_t mag=(uint32_t)(value<0?-value:value);
    if(code==0x3A)incoming.data[1]=(uint8_t)value;
    else {
        incoming.data[1]=value<0;
        if(code==0x35){incoming.data[2]=mag>>8;incoming.data[3]=mag;}
        else{incoming.data[2]=mag>>24;incoming.data[3]=mag>>16;incoming.data[4]=mag>>8;incoming.data[5]=mag;}
    }
    incoming.data[incoming.len-1]=0x6B;HAL_CAN_RxFifo0MsgPendingCallback(&board_can);
}

int main(int argc, char **argv)
{
    float delta=argc>1?(float)atof(argv[1]):5.f;
    unsigned lateral=argc>2?(unsigned)atoi(argv[2]):0;
    uint8_t data[96]={0},response[128],ticket[76]={0};uint16_t size=0;
    unsigned enabled=0,configured=0,prepared=0,armed=0,goal_sent=0,returned=0,returning=0,held_writes=0;
    int rpm[4]={0};double counts[4]={0};unsigned imu_sequence=1;
    /* FD模式02：按3200脉冲/圈走完相对位移，F6速度命令中断位置小步。 */
    double position_left[4]={0};unsigned fd_rpm[4]={0},fd_dir[4]={0},fd_pulses[4]={0},fd_moves[4]={0};int fd_signed[4]={0};
    Can_Refresh();Comm_Init(123,tick);Base_Init();SessionPacket(COMM_OPEN,1);
    for(tick=0;tick<18000;tick++)
    {
        for(unsigned i=0;i<4;i++)
        {
            if(position_left[i]!=0)
            {
                double step=fd_rpm[i]*65536./60000.;
                if(step>fabs(position_left[i]))step=fabs(position_left[i]);
                step=position_left[i]<0?-step:step;
                counts[i]+=step;position_left[i]-=step;
                rpm[i]=position_left[i]!=0?(int)fd_rpm[i]*(step<0?-1:1):0;
            }
            else counts[i]+=rpm[i]*65536./60000.;
        }
        if(occupied)
        {
            unsigned address=last.id>>8,code=last.data[0];
            if(code==0xF3){CHECK(address<=4);enabled|=1U<<(address-1);}
            if(code==0xF6){CHECK(address<=4);rpm[address-1]=((last.data[2]<<8)|last.data[3])*(last.data[1]?-1:1);position_left[address-1]=0;}
            if(code==0xFD&&!(last.id&255U))
            {
                CHECK(address>=1&&address<=4&&last.data[4]==0);unsigned w=address-1;
                fd_dir[w]=last.data[1];fd_rpm[w]=(last.data[2]<<8)|last.data[3];
                fd_pulses[w]=((unsigned)last.data[5]<<24)|((unsigned)last.data[6]<<16)|((unsigned)last.data[7]<<8);
            }
            else if(code==0xFD)
            {
                unsigned w=address-1;CHECK(last.data[2]==2&&last.data[3]==0);fd_pulses[w]|=last.data[1];
                fd_signed[w]=(int)fd_pulses[w]*(fd_dir[w]?-1:1);fd_moves[w]++;
                position_left[w]=fd_signed[w]*65536./3200.;rpm[w]=0;
            }
            if(code==0xFE&&address<=4){rpm[address-1]=0;position_left[address-1]=0;}
            Done();
            if(code==0x42)ConfigReply(address);
            else if(code==0x35||code==0x36||code==0x3A||code==0x33)
            {
                if(address>=5)FeedbackReply(address,code);
                else WheelReply(address,code,code==0x35?rpm[address-1]:code==0x36?(int64_t)llround(counts[address-1]):(enabled&(1U<<(address-1))?1:0));
            }
        }
        if(tick%100==0)SessionPacket(COMM_HEART,tick/100+1);
        if(tick%10==0)
        {
            ImuSample sample={0};sample.stamp.epoch=sample.source_epoch=1;
            sample.stamp.first_ms=sample.stamp.last_ms=sample.stamp.handled_ms=tick;
            sample.stamp.sequence=imu_sequence++;sample.kind=0x52;
            gyro=sample;Exec_Imu(&executor,&sample,tick);sample.kind=0x53;Exec_Imu(&executor,&sample,tick);
        }
        if(tick==1800){uint8_t attach[8]={1,0,0,0,0x48,0x43,0x45,0x4D};CHECK(Command(48,attach,8,response,&size,tick)==COMM_OK&&!Frame_Get(response,4));}
        if(tick==3500){memset(data,0,sizeof(data));data[0]=2;data[8]=1;CHECK(Command(50,data,9,response,&size,tick)==COMM_OK&&!Frame_Get(response,4));}
        if(tick==3600){data[0]=3;data[4]=1;data[8]=data[9]=1;CHECK(Command(62,data,10,response,&size,tick)==COMM_OK&&!Frame_Get(response,4));}
        if(tick==3700){memset(data,0,sizeof(data));data[0]=4;data[4]=1;data[8]=4;data[12]=1;CHECK(Command(66,data,13,response,&size,tick)==COMM_OK&&!Frame_Get(response,4));}
        if(tick==4500){data[0]=5;CHECK(Command(67,data,13,response,&size,tick)==COMM_OK&&!Frame_Get(response,4));CHECK(fixture_pwm_writes==3);}
        if(tick==5000)
        {
            ticket[0]=1;ticket[4]=10;ticket[12]=1;ticket[16]=1;ticket[40]=1;
            CHECK(Command(71,ticket,76,response,&size,tick)==COMM_OK&&!Frame_Get(response,4));
        }
        if(tick==5010){ticket[0]=2;CHECK(Command(71,ticket,76,response,&size,tick)==COMM_OK&&!Frame_Get(response,4));CHECK(MechPort_Holding());held_writes=fixture_pwm_writes;}
        if(tick>=5100&&!configured)
        {
            ExecConfig config={0};config.version=10;config.geometry=(KinGeometry){.05,.094,.11,1};
            config.counts_per_rev=65536;config.yaw_sign=1;config.angle_scale_rad=.0000958738f;
            config.gyro_scale_rad_s=.0010652644f;config.max_rpm=6;
            for(unsigned i=0;i<4;i++){config.address[i]=(uint8_t)(i+1);config.command_sign[i]=config.feedback_sign[i]=(i==1||i==3)?-1:1;}
            if(Command(EXEC_CONFIG,(uint8_t*)&config,sizeof(config),response,&size,tick)==COMM_OK)configured=1;
        }
        if(tick>=5700&&configured&&!prepared)
        {
            uint8_t key[8]={10,0,0,0,10,0,0,0};
            key[0]=9;CHECK(Command(59,key,8,response,&size,tick)==COMM_READONLY);key[0]=10;
            if(Command(59,key,8,response,&size,tick)==COMM_OK)prepared=1;
        }
        physical_key=prepared&&tick>=6000&&tick<6100;
        if(tick>=6300&&start_key.phase==START_FIRED&&!armed)
            if(Command(EXEC_ARM,0,0,response,&size,tick)==COMM_OK)armed=1;
        if(armed&&!goal_sent&&executor.mode==EXEC_READY)
        {
            TrackGoal goal={0};goal.id=1;goal.timeout_ms=10000;goal.reference.kind=1;
            if(lateral)goal.reference.y_m=delta/1000.f;else goal.reference.x_m=delta/1000.f;
            goal.reference.linear=(CurveLimit){.01f,.05f,.2f};goal.reference.angular=(CurveLimit){.1f,.3f,1.f};
            CHECK(Command(EXEC_GOAL,(uint8_t*)&goal,sizeof(goal),response,&size,tick)==COMM_OK);goal_sent=1;
        }
        if(goal_sent&&executor.mode==EXEC_READY&&executor.track.output.state==TRACK_COMPLETE) returning=1;
        if(returning)
        {
            ticket[0]=4;CHECK(Command(71,ticket,76,response,&size,tick)==COMM_OK);
            if(!Frame_Get(response,4)){returned=1;break;}
        }
        Base_Run();
        if(executor.fault){fprintf(stderr,"tick%u basefault%u hold%u bus%u\n",tick,executor.fault,MechPort_Holding(),(unsigned)bus_diag.error);CHECK(0);}
        if(tick>5020)CHECK(fixture_pwm_writes==held_writes);
    }
    if(!(configured&&prepared&&armed&&goal_sent&&returned))
        fprintf(stderr,"stages %u/%u/%u/%u/%u mode%u key%u gate%u fault%u\n",configured,prepared,armed,goal_sent,returned,executor.mode,start_key.phase,executor.gate.mask,executor.fault);
    CHECK(configured&&prepared&&armed&&goal_sent&&returned);
    CHECK(micro_trace.frozen && micro_trace.action==1 && micro_trace.count>3 && micro_trace.count<224);
    CHECK(!micro_trace.exec_fault && !micro_trace.track_fault);
    /* 位置小步四轮同幅；乘回command_sign后符合麦轮x/y模式；期间不发非零速度。 */
    CHECK(fd_moves[0]>=1);
    for(unsigned i=1;i<4;i++)CHECK(fd_moves[i]==fd_moves[0]&&abs(fd_signed[i])==abs(fd_signed[0])&&fd_rpm[i]==2);
    {
        int body[4];for(unsigned i=0;i<4;i++)body[i]=fd_signed[i]*((i==1||i==3)?-1:1);
        if(lateral)CHECK(body[0]==-body[1]&&body[1]==body[2]&&body[2]==-body[3]);
        else CHECK(body[0]==body[1]&&body[1]==body[2]&&body[2]==body[3]);
        CHECK((body[lateral?1:0]>0)==(delta>0));
    }
    for(unsigned n=0;n<micro_trace.count;n++)
        for(unsigned i=0;i<4;i++)CHECK(!micro_trace.samples[n].command[i]&&!micro_trace.samples[n].sent[i]);

    CHECK(!MechPort_Holding()&&MechPort_Owned()&&executor.stop_confirmed);
    CHECK(start_key.phase==START_IDLE&&!start_key.ticket);
    CHECK(Command(58,0,0,response,&size,tick)==COMM_OK&&Frame_Get(response+20,4)==1&&Frame_Get(response+16,4)==1&&!Frame_Get(response+12,4));
    CHECK(Comm_Boot()==123&&Comm_Session()==456&&fixture_pwm_writes==3);
    CHECK(fabs((lateral?executor.odom.pose.y_m:executor.odom.pose.x_m)-delta/1000.f)<.0009);
    /* 正常返回后可重新准备，但旧票据不能授权下一轮；不复位、不动PWM。 */
    ticket[0]=1;ticket[4]=11;ticket[8]=2;
    CHECK(Command(71,ticket,76,response,&size,tick)==COMM_OK&&!Frame_Get(response,4));
    ticket[0]=2;CHECK(Command(71,ticket,76,response,&size,tick)==COMM_OK&&!Frame_Get(response,4));
    {uint8_t key[8]={10,0,0,0,10,0,0,0};
     CHECK(Command(59,key,8,response,&size,tick)==COMM_READONLY);key[0]=11;
     CHECK(Command(59,key,8,response,&size,tick)==COMM_OK&&start_key.phase==START_RELEASE);}
    ticket[0]=4;CHECK(Command(71,ticket,76,response,&size,tick)==COMM_OK&&!Frame_Get(response,4));
    CHECK(!MechPort_Holding()&&start_key.phase==START_IDLE&&fixture_pwm_writes==3);
    printf("same-session PC0 -> micro %.1fmm axis%u -> MECH PASS at %ums, x=%.6f y=%.6f, PWM writes3\n",delta,lateral,tick,executor.odom.pose.x_m,executor.odom.pose.y_m);
    return 0;
}
