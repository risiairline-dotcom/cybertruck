"""毫米微调：真实C跟踪/里程计，合成异步反馈与电机滞后；不是本车物理证明。"""
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
    def __init__(self, library, axis='y', mm=5, period=120, tau=.2,
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
        for _ in range(40):self.advance(False)
        spec=Spec();spec.kind=1
        setattr(spec,axis+'_m',mm/1000)
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
                        self.assertLessEqual(result['peak_rpm'],2)
                        # 每笔移动指令为四轮等幅，不再有独立取整造成的半组停转。
                        for _,out in model.commands:
                            self.assertEqual(len(set(map(abs,out['rpm']))),1)
                        reports.append(dict(period_ms=period,tau_s=tau,axis=axis,mm=mm,
                                            elapsed_ms=result['elapsed_ms'],error_m=result['output']['position_error_m']))
        (ROOT/'output/tests/micro_dynamics_0349.json').write_text(json.dumps(reports,indent=2),encoding='utf-8')

    def test_missing_motion_ack_stops_within_feedback_budget(self):
        model=DelayedMicro(self.library,ack=False)
        result=model.run(11000)
        self.assertEqual(result['state'],'FAILED')
        self.assertEqual(result['output']['fault'],7)
        nonzero=[at for at,out in model.commands if any(out['rpm'])]
        self.assertLess(max(nonzero)-min(nonzero),250)

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
        (ROOT/'output/tests/micro_heading_0349.json').write_text(json.dumps(
            dict(synthetic=True,cases=reports),indent=2),encoding='utf-8')

    def test_excessive_imbalance_is_not_reported_as_accurate_completion(self):
        model=DelayedMicro(self.library)
        model.plant.gains=[1.8,1.8,.2,.2]
        result=model.run(11000)
        self.assertEqual(result['state'],'FAILED')
        self.assertEqual(result['output']['fault'],7)
        self.assertEqual(result['output']['rpm'],[0]*4)

    def test_motion_ack_without_zero_ack_never_launches_second_step(self):
        model=DelayedMicro(self.library)
        original=model.tracker.sent
        model.tracker.sent=lambda action,token,tick: original(action,token,tick) if any(model.plant.targets) else 0
        result=model.run(11000)
        self.assertEqual(result['state'],'FAILED')
        nonzero=[at for at,out in model.commands if any(out['rpm'])]
        self.assertLess(max(nonzero)-min(nonzero),600)
        self.assertEqual(result['output']['fault'],7)

    def test_stall_still_times_out_and_zeroes(self):
        model=DelayedMicro(self.library,stalled=True)
        result=model.run(11000)
        self.assertEqual(result['state'],'FAILED')
        self.assertEqual(result['output']['fault'],7)
        self.assertEqual(result['output']['rpm'],[0]*4)

    def test_lost_feedback_stops_without_another_step(self):
        model=DelayedMicro(self.library)
        for _ in range(30):model.advance(False)
        model.drop.add((1,model.calibration.address[2]))
        result=model.run(11000)
        self.assertEqual(result['state'],'FAILED')
        self.assertEqual(result['output']['fault'],2)
        self.assertEqual(result['output']['rpm'],[0]*4)

    def test_steps_wait_for_zero_tx_and_new_stable_feedback(self):
        model=DelayedMicro(self.library,period=160)
        model.run(11000)
        bursts=[];active=False
        for at,out in model.commands:
            if any(out['rpm']):
                if not active:bursts.append([at,at])
                bursts[-1][1]=at
                active=True
            else:active=False
        self.assertGreater(len(bursts),1)
        for previous,next_ in zip(bursts,bursts[1:]):
            self.assertGreaterEqual(next_[0]-previous[1],150)
        self.assertGreaterEqual(len(model.acknowledged),len(bursts))


if __name__=='__main__':unittest.main()

