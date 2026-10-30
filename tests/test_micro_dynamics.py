"""毫米微调：真实C跟踪/里程计，合成异步反馈与驱动器位置模式；不是本车物理证明。"""
import math
import json
import subprocess
import sys
import time
import unittest
from pathlib import Path
from robot.track_sim import ClosedLoop, settings
from robot.odom_sim import fixture
from robot.device.tracking import Goal, Spec, Limit

ROOT=Path(__file__).resolve().parents[1]


class DelayedMicro(ClosedLoop):
    """axis为x/y时amount是毫米，为yaw时是度。"""
    def __init__(self, library, axis='y', amount=5, period=120, tau=.2,
                 gain=1., ack=True, stalled=False, send_delay_ms=10):
        calibration=fixture();calibration.calibration=10;calibration.gap_ms=250;calibration.margin_counts=64
        config=settings(calibration)
        config.kp_xy=1
        config.kp_yaw=1;config.kd_gyro=.1
        config.correction_m_s=.015;config.correction_rad_s=.1
        config.max_rpm=6;config.slew_rpm_s=30
        config.position_tol_m=.0008;config.yaw_tol_rad=math.radians(.2)
        config.still_rpm=1;config.still_m_s=.005;config.still_rad_s=.035
        config.count_drift[:]=[32]*4
        config.feedback_gap_ms=250;config.micro_steps=1
        self.period=period
        super().__init__(library,config=config,calibration=calibration,auto_start=False,
                         disturbance=False,ack=ack,stalled=stalled,send_delay_ms=send_delay_ms)
        self.plant.tau=[tau*.7,tau,tau*1.2,tau*.9]
        self.plant.gains=[v*gain for v in (.85,1.15,.95,1.05)]
        self.moves=[];self.peak_wheel_rpm=0.
        move,integrate=self.plant.move,self.plant.integrate
        def record(revolutions,rpm):
            self.moves.append((self.plant.ms,list(revolutions),rpm));move(revolutions,rpm)
        def advance(disturbance=False):
            integrate(disturbance);self.peak_wheel_rpm=max(self.peak_wheel_rpm,*map(abs,self.plant.rpm))
        self.plant.move,self.plant.integrate=record,advance
        for _ in range(40):self.advance(False)
        spec=Spec()
        if axis=='yaw':spec.kind=2;spec.yaw_rad=math.radians(amount)
        else:spec.kind=1;setattr(spec,axis+'_m',amount/1000)
        spec.linear=Limit(.01,.05,.2);spec.angular=Limit(.1,.3,1)
        self.begin(Goal(1,10000,spec))
        self.commands=[]

    def deliver(self,kind,address,values,tick,epoch,source_epoch):
        # 轮询每40ms才轮到同一轮；选定120/160/200ms更新，接收时间仍是真实新样本时间。
        if kind in (1,2) and (tick//40)%(self.period//40)!=0:return
        super().deliver(kind,address,values,tick,epoch,source_epoch)

    def advance(self,report=True):
        value=super().advance(report)
        if hasattr(self,'commands'):
            self.commands.append((self.elapsed_ms,self.tracker.output()))
        return value


def duration_ms(move):
    _,revolutions,rpm=move
    return max(40,abs(revolutions[0])/(rpm/60)*1000)


class MicroDynamicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.library=ROOT/'output/tests'/f'micro-dynamics-{time.time_ns()}.dll'
        subprocess.run([sys.executable,str(ROOT/'tools/build_odom.py'),'--output',str(cls.library)],
                       check=True,capture_output=True)

    def test_delayed_asymmetric_axes_and_signs(self):
        reports=[]
        for period in (120,160,200):
            for tau in (.08,.2,.35):
                for axis,mm in (('x',5),('x',-5),('y',5),('y',-5),('x',1),('y',-1)):
                    with self.subTest(period=period,tau=tau,axis=axis,mm=mm):
                        model=DelayedMicro(self.library,axis,mm,period,tau)
                        result=model.run(11000)
                        self.assertEqual(result['state'],'COMPLETE')
                        self.assertLessEqual(result['output']['position_error_m'],.0008)
                        self.assertLessEqual(abs(getattr(model.plant,axis)-mm/1000),.001)
                        self.assertTrue(model.acknowledged)
                        self.assertLessEqual(model.peak_wheel_rpm,2+1e-9)
                        # 每步四轮同幅、只走一个方向；全程不发非零速度命令。
                        for _,revolutions,rpm in model.moves:
                            self.assertEqual(len({round(abs(v),12) for v in revolutions}),1)
                            self.assertLessEqual(rpm,2)
                        for _,out in model.commands:self.assertEqual(out['rpm'],[0]*4)
                        reports.append(dict(period_ms=period,tau_s=tau,axis=axis,mm=mm,moves=len(model.moves),
                                            elapsed_ms=result['elapsed_ms'],error_m=result['output']['position_error_m']))
        (ROOT/'output/tests/micro_dynamics_0352.json').write_text(json.dumps(reports,indent=2),encoding='utf-8')

    def test_turn_goal_reaches_heading_in_capped_steps(self):
        for degrees in (3,-3,.5):
            with self.subTest(degrees=degrees):
                model=DelayedMicro(self.library,'yaw',degrees)
                result=model.run(11000)
                self.assertEqual(result['state'],'COMPLETE',result['output'])
                self.assertLessEqual(abs(model.plant.yaw-math.radians(degrees)),math.radians(.2))
                self.assertLessEqual(math.hypot(model.plant.x,model.plant.y),.0008)
                # 3°在0.204m杠杆上约10.7mm轮缘弧长，按单步6mm上限至少分两步。
                self.assertGreaterEqual(len(model.moves),2 if abs(degrees)==3 else 1)
                for _,revolutions,rpm in model.moves:
                    self.assertEqual(rpm,1)
                    self.assertLessEqual(abs(revolutions[0])*2*math.pi*.05,.006+1e-12)

    def test_missing_motion_ack_stops_without_resend(self):
        model=DelayedMicro(self.library,ack=False)
        result=model.run(11000)
        self.assertEqual(result['state'],'FAILED')
        self.assertEqual(result['output']['fault'],7)
        self.assertEqual(len(model.moves),1)

    def test_heading_bias_stays_within_tolerance(self):
        reports=[]
        for delay in (10,80,160):
            for gain in (.2,.4,.6):
                model=DelayedMicro(self.library,send_delay_ms=delay)
                model.plant.gains=[1+gain,1+gain,1-gain,1-gain]
                result=model.run(11000)
                self.assertEqual(result['state'],'COMPLETE',(delay,gain,result['output']))
                self.assertLessEqual(abs(result['plant']['yaw_rad']),math.radians(.2))
                self.assertLessEqual(result['output']['position_error_m'],.0008)
                reports.append(dict(delay_ms=delay,gain_difference=gain,result=result['output']))
        (ROOT/'output/tests/micro_heading_0352.json').write_text(json.dumps(
            dict(synthetic=True,cases=reports),indent=2),encoding='utf-8')

    def test_stuck_wheel_is_not_reported_as_accurate_completion(self):
        model=DelayedMicro(self.library)
        integrate=model.plant.integrate
        def stuck(disturbance=False):
            if model.plant.position_goal is not None:model.plant.position_goal[0]=model.plant.turns[0]
            model.plant.targets[0]=0;integrate(disturbance)
        model.plant.integrate=stuck
        result=model.run(11000)
        if result['state']=='COMPLETE':
            self.assertLessEqual(abs(model.plant.y-.005),.001)
            self.assertLessEqual(abs(model.plant.yaw),math.radians(.2))
        else:
            self.assertEqual(result['state'],'FAILED')
            self.assertEqual(result['output']['rpm'],[0]*4)

    def test_motion_ack_without_zero_ack_never_launches_second_step(self):
        model=DelayedMicro(self.library)
        original=model.tracker.sent
        model.tracker.sent=lambda action,token,tick: original(action,token,tick) if model.plant.position_goal is not None else 0
        result=model.run(11000)
        self.assertEqual(result['state'],'FAILED')
        self.assertEqual(result['output']['fault'],7)
        self.assertEqual(len(model.moves),1)

    def test_stall_still_times_out_and_zeroes(self):
        model=DelayedMicro(self.library,stalled=True)
        result=model.run(11000)
        self.assertEqual(result['state'],'FAILED')
        self.assertEqual(result['output']['fault'],7)
        self.assertEqual(result['output']['rpm'],[0]*4)
        # 车轮始终未动：首步得不到编码器进展即超时，不追加第二步。
        self.assertEqual(len(model.moves),1)

    def test_lost_feedback_stops_without_another_step(self):
        model=DelayedMicro(self.library)
        for _ in range(30):model.advance(False)
        model.drop.add((1,model.calibration.address[2]))
        result=model.run(11000)
        self.assertEqual(result['state'],'FAILED')
        self.assertEqual(result['output']['fault'],2)
        self.assertEqual(result['output']['rpm'],[0]*4)

    def test_steps_wait_for_execution_zero_tx_and_stable_feedback(self):
        model=DelayedMicro(self.library,'yaw',3,period=160)
        result=model.run(11000)
        self.assertEqual(result['state'],'COMPLETE')
        self.assertGreater(len(model.moves),1)
        for previous,next_ in zip(model.moves,model.moves[1:]):
            self.assertGreaterEqual(next_[0]-previous[0],duration_ms(previous)+150)
        self.assertGreaterEqual(len(model.acknowledged),2*len(model.moves))

    def test_late_drive_start_is_not_cut_short_by_zero_command(self):
        """驱动器起步迟滞超过估算执行时间：未走过一半前不能发零速打断小步。"""
        for delay_ms in (100,300,600):
            with self.subTest(delay_ms=delay_ms):
                model=DelayedMicro(self.library,'y',1)
                plant=model.plant;integrate=plant.integrate;state=dict(goal=None,since=0)
                def late(disturbance=False):
                    goal=plant.position_goal
                    if goal is not state['goal']:state.update(goal=goal,since=plant.ms)
                    if goal is not None and plant.ms-state['since']<delay_ms:
                        plant.position_goal=None;saved=plant.targets[:];plant.targets[:]=[0]*4
                        integrate(disturbance);plant.position_goal=goal;plant.targets[:]=saved
                    else:integrate(disturbance)
                plant.integrate=late
                result=model.run(11000)
                self.assertEqual(len(model.moves),1)
                if delay_ms<=300:
                    self.assertEqual(result['state'],'COMPLETE',result['output'])
                    self.assertLessEqual(abs(model.plant.y-.001),.0008)
                else:
                    # 超出1mm小步的3倍+500ms执行期限：按超时停止，不提前发零速、不补第二步。
                    self.assertEqual((result['state'],result['output']['fault']),('FAILED',7))


if __name__=='__main__':unittest.main()
