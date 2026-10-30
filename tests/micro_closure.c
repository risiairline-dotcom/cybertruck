/* 真实Exec配置和Track_Step复现2026-10-29残差；无设备IO。 */
#include "execution_native.c"
#include <assert.h>
#include <stdio.h>

static TrackConfig Configuration(unsigned version, double radius)
{
    ExecConfig config={0};int64_t counts[4]={0};int16_t speeds[4]={0};
    Device_Init();Device_Tick(100,counts,speeds,0,0);
    config.version=version;config.geometry=(KinGeometry){radius,.094,.11,1};
    config.counts_per_rev=65536;config.yaw_sign=1;config.angle_scale_rad=.0000958738f;
    config.gyro_scale_rad_s=.0010652644f;config.max_rpm=6;
    for(unsigned i=0;i<4;i++){config.address[i]=(uint8_t)(i+1);config.command_sign[i]=config.feedback_sign[i]=(i==1||i==3)?-1:1;}
    assert(Exec_Config(&executor,&config,100)>0);return executor.track.config;
}

static void Fresh(TrackObservation *o,unsigned tick)
{
    o->pose.valid=1;o->pose.calibration=10;o->pose.tick_ms=tick;o->pose.steps=tick;
    o->imu_source_epoch=1;
    RxStamp s={tick,tick,tick,tick+1,1};
    for(unsigned i=0;i<4;i++)o->position[i]=o->speed[i]=s;
    o->yaw_stamp=o->gyro_stamp=s;
}

static void Residual(TrackConfig config,double dx,double dy,int legacy)
{
    Track t;TrackObservation o={0};TrackGoal goal={0};
    if(legacy){config.kp_xy=1;config.micro_steps=0;}
    assert(Track_Init(&t,&config));Fresh(&o,100);
    goal.id=1;goal.timeout_ms=10000;goal.reference.kind=1;goal.reference.y_m=.005f;
    goal.reference.linear=(CurveLimit){.01f,.05f,.2f};goal.reference.angular=(CurveLimit){.1f,.3f,1};
    assert(Track_Start(&t,&goal,&o,100)==1);
    unsigned corrective=0;
    for(unsigned ms=110;ms<=2100;ms+=10)
    {
        Fresh(&o,ms);o.pose.x_m=dx;o.pose.y_m=.005+dy;
        int status=Track_Step(&t,&o,ms);
        /* 车体始终停在残差处：位置小步得不到编码器进展，只能超时停止，不能报完成。 */
        if(status<0){assert(!legacy&&t.output.fault==TRACK_TIMEOUT);break;}
        if(!legacy)
        {
            /* 位置小步四轮同幅；按圈数方向判断是否朝减小残差的方向走。 */
            if(t.output.micro_hold)
            {
                float wheels[4];KinBody b;
                for(unsigned i=0;i<4;i++){wheels[i]=(float)t.output.micro_rev[i];assert(fabs(wheels[i])==fabs(wheels[0]));}
                assert(t.output.micro_rpm>=1&&t.output.micro_rpm<=2);
                assert(Kin_Forward(config.geometry,wheels,&b));
                if(b.vx_m_s*dx+b.vy_m_s*dy<0)corrective++;
            }
            for(unsigned i=0;i<4;i++)assert(t.output.rpm[i]==0);
            if(Track_Token(&t)) (void)Track_Sent(&t,1,Track_Token(&t),ms);
        }
        if(status==1)break;
    }
    unsigned moving=0;float rpm[4];KinBody body;
    for(unsigned i=0;i<4;i++)
    {rpm[i]=(float)t.output.rpm[i];moving|=rpm[i]!=0;assert(fabs(rpm[i])<=6);}
    assert(Kin_Forward(config.geometry,rpm,&body));
    if(legacy){assert(!moving&&t.position_active&&t.output.state==TRACK_RUNNING);}
    else if(hypot(dx,dy)>.0008)
    {assert(corrective);assert(t.output.state!=TRACK_COMPLETE);}
    else{assert(!moving&&!t.position_active&&t.output.state==TRACK_COMPLETE);}
    printf("radius%.3f residual%.6f/%.6f legacy%d gain%.6f output%.0f/%.0f/%.0f/%.0f PASS\n",
           config.geometry.radius_m,dx,dy,legacy,config.kp_xy,rpm[0],rpm[1],rpm[2],rpm[3]);
}

int main(void)
{
    TrackConfig old=Configuration(2,.05);assert(old.kp_xy==1&&old.position_tol_m==.005&&!old.micro_steps);
    for(unsigned r=0;r<3;r++)
    {
        TrackConfig config=Configuration(10,.04+r*.01);
        assert(config.position_tol_m==.0008&&config.max_rpm==6&&config.correction_m_s==.015&&config.slew_rpm_s==30);
        Residual(config,0,.0009262503768165483,1);
        Residual(config,0,.0009262503768165483,0);
        Residual(config,.00092625,0,0);Residual(config,-.00092625,0,0);Residual(config,0,-.00092625,0);
        Residual(config,.0006,.0006,0);Residual(config,0,.00081,0);Residual(config,0,.00079,0);
    }
    return 0;
}
