/* 真实Track：位置小步圈数/转速、TXOK前不计时、未静止不进入停车取样；航向先修且不被新目标重定义。 */
#define main closure_main
#include "micro_closure.c"
#undef main

static void Begin(Track *t, TrackObservation *o, double tolerance)
{
    TrackConfig c=Configuration(10,.05);TrackGoal g={0};
    c.yaw_tol_rad=tolerance;assert(Track_Init(t,&c));
    memset(o,0,sizeof(*o));Fresh(o,100);
    g.id=1;g.timeout_ms=10000;g.reference.kind=1;g.reference.y_m=.005f;
    g.reference.linear=(CurveLimit){.01f,.05f,.2f};g.reference.angular=(CurveLimit){.1f,.3f,1};
    assert(Track_Start(t,&g,o,100)==1);
}

static double Turns(double meters) { return meters/(2*3.14159265358979323846*.05); }

static int Rotation(const Track *t)
{
    const double *r=t->output.micro_rev;
    return r[0]==-r[1] && r[0]==r[2] && r[1]==r[3] && r[0]!=0;
}

int main(void)
{
    Track t;TrackObservation o;unsigned tick=100,token=0;
    const double lever=.094+.11;
    /* 转速受配置限幅和纠偏速度上限约束；圈数按剩余误差，单步不超过6mm轮缘弧长。 */
    for(unsigned mode=0;mode<6;mode++)
    {
        Begin(&t,&o,.0034906585);
        if(mode==1)t.config.max_rpm=1;
        if(mode==2)t.config.correction_m_s=.006;
        if(mode==3)o.pose.y_m=.004;
        if(mode==4)o.yaw_rad=.02;
        if(mode==5)o.yaw_rad=.05;
        Fresh(&o,110);assert(Track_Step(&t,&o,110)>=0);
        assert(t.output.micro_hold && t.micro_phase==1);
        unsigned rpm=(mode==0||mode==3)?2:1;
        double turns=mode==3?Turns(.001):mode==4?Turns(.02*lever):mode==5?Turns(.006):Turns(.005);
        assert(t.output.micro_rpm==rpm);
        for(unsigned i=0;i<4;i++){assert(fabs(fabs(t.output.micro_rev[i])-turns)<1e-7);assert(t.output.rpm[i]==0);}
        assert(mode<4 ? !Rotation(&t) : Rotation(&t));
        assert(fabs(t.micro_duration_ms-fmax(40,turns/(rpm/60.)*1000))<=1);
    }

    /* 票据：错动作/错号/早于请求不算；TXOK前不计时，静止新反馈才进入停车取样。 */
    Begin(&t,&o,.0034906585);
    while(!token){tick+=10;Fresh(&o,tick);assert(Track_Step(&t,&o,tick)>=0);token=Track_Token(&t);}
    unsigned requested=tick;
    assert(!Track_Sent(&t,2,token,tick));
    assert(!Track_Sent(&t,1,token+100,tick));
    assert(!Track_Sent(&t,1,token,requested-1));
    for(;tick<requested+100;){tick+=10;Fresh(&o,tick);assert(Track_Step(&t,&o,tick)>=0);}
    assert(!t.micro_acked && t.micro_phase==1 && Track_Token(&t)==token);
    assert(Track_Sent(&t,1,token,tick));
    assert(!Track_Sent(&t,1,token,tick));
    unsigned sent=tick,duration=t.micro_duration_ms;
    for(;tick+10<sent+duration;){tick+=10;Fresh(&o,tick);assert(Track_Step(&t,&o,tick)>=0);}
    assert(t.micro_phase==1 && t.output.micro_hold);
    tick=sent+duration;Fresh(&o,tick);for(unsigned i=0;i<4;i++)o.rpm[i]=2;
    assert(Track_Step(&t,&o,tick)>=0);assert(t.micro_phase==1); /* 驱动器仍在转，不提前发零速。 */
    tick+=10;Fresh(&o,tick);for(unsigned i=0;i<4;i++)o.rpm[i]=0;assert(Track_Step(&t,&o,tick)>=0);
    assert(t.micro_phase==1); /* 静止但编码器没有走过一半，可能是驱动器起步迟滞，继续等待。 */
    tick+=10;Fresh(&o,tick);o.pose.y_m=.003;assert(Track_Step(&t,&o,tick)>=0);
    assert(t.micro_phase==2 && !t.output.micro_hold && Track_Token(&t)!=token);
    Track_Cancel(&t);assert(!Track_Sent(&t,1,token,tick));

    /* 驱动器未确认：250ms内无TXOK即停止，不重发。 */
    Begin(&t,&o,.0034906585);token=0;
    for(tick=110;!token;tick+=10){Fresh(&o,tick);assert(Track_Step(&t,&o,tick)>=0);token=Track_Token(&t);}
    {
        int status=0;
        for(;status>=0 && tick<1000;tick+=10){Fresh(&o,tick);status=Track_Step(&t,&o,tick);}
        assert(status<0 && t.output.fault==TRACK_TIMEOUT && !t.output.micro_hold);
    }

    /* 驱动器一直未静止：超过估算3倍+500ms即停止。 */
    Begin(&t,&o,.0034906585);token=0;
    for(tick=110;!token;tick+=10){Fresh(&o,tick);assert(Track_Step(&t,&o,tick)>=0);token=Track_Token(&t);}
    assert(Track_Sent(&t,1,token,tick));sent=tick;duration=t.micro_duration_ms;
    {
        int status=0;
        for(;status>=0;tick+=10){Fresh(&o,tick);for(unsigned i=0;i<4;i++)o.rpm[i]=2;status=Track_Step(&t,&o,tick);}
        assert(t.output.fault==TRACK_TIMEOUT && tick-sent>duration*3+500 && tick-sent<=duration*3+530);
    }

    /* 平移目标先修航向：残余0.02rad时先出纯转向小步，宽容差下直接完成。 */
    for(unsigned legacy=0;legacy<2;legacy++)
    {
        unsigned rotated=0;
        Begin(&t,&o,legacy?.035:.0034906585);
        for(tick=110;tick<2200;tick+=10)
        {
            Fresh(&o,tick);o.pose.y_m=.005;o.yaw_rad=o.pose.yaw_rad=.02;
            if(Track_Step(&t,&o,tick)<0){assert(!legacy&&t.output.fault==TRACK_TIMEOUT);break;}
            if(t.output.micro_hold)rotated|=Rotation(&t);
            token=Track_Token(&t);if(token)(void)Track_Sent(&t,1,token,tick);
            if(t.output.state==TRACK_COMPLETE)break;
        }
        if(legacy)assert(t.output.state==TRACK_COMPLETE && !rotated);
        else assert(t.output.state!=TRACK_COMPLETE && rotated);
    }
    /* 新平移目标不能把原来的偏航重定义为零误差。 */
    TrackGoal g=t.goal;g.id=2;Fresh(&o,tick+10);
    assert(Track_Start(&t,&g,&o,tick+10)==1);
    assert(t.micro_heading==0);
    Fresh(&o,tick+20);assert(Track_Step(&t,&o,tick+20)>=0);
    assert(fabs(t.output.yaw_error_rad-.02)<1e-6);

    /* 纯转向目标以当前航向加转角重设目标航向；超出0.1°～3°或带平移的转向被拒绝。 */
    {
        TrackConfig c=t.config;TrackGoal turn=g,bad;
        c.yaw_tol_rad=.0034906585;
        turn.id=3;turn.reference.kind=2;turn.reference.x_m=turn.reference.y_m=0;turn.reference.yaw_rad=-.02f;
        assert(Track_Init(&t,&c));o.yaw_rad=.02;Fresh(&o,tick+30);
        assert(Track_Start(&t,&turn,&o,tick+30)==1 && fabs(t.micro_heading-(.02+(double)-.02f))<1e-12);
        Fresh(&o,tick+40);assert(Track_Step(&t,&o,tick+40)>=0);
        assert(t.output.micro_hold && Rotation(&t));
        assert(Track_Init(&t,&c));bad=turn;bad.id=4;
        bad.reference.yaw_rad=.06f;assert(Track_Start(&t,&bad,&o,tick+50)==-1);
        bad.reference.yaw_rad=.001f;assert(Track_Start(&t,&bad,&o,tick+50)==-1);
        bad.reference.yaw_rad=.02f;bad.reference.x_m=.001f;assert(Track_Start(&t,&bad,&o,tick+50)==-1);
    }
    puts("micro position steps / stale ACK / not-still timeout / yaw first / fixed heading / yaw goal PASS");
    return 0;
}
