"""圆环对准画面中心：合成相机/主机状态，不访问实际相机或电机。"""
import copy
import json
import math
import struct
import tempfile
import unittest
from pathlib import Path

from robot.device.arrival import Arrival,contract
from robot.device.execution import SPEC_DATA
from robot.vision.centering import Centering,load_calibration
from robot.vision.centering_fit import fit_records
from robot.vision.legacy.alignment_vision.core import fingerprint
from test_arrival import observation_record,link


CAMERA=dict(device_id='/dev/v4l/by-id/test-material',image_size=[1280,720],profile_sha256='a'*64)


def calibration(binding):
    value=dict(schema_version=1,goal='camera_image_center',station='rough',target=2,
        camera_signature=CAMERA,profile_fingerprint=fingerprint(CAMERA),
        scope='fixed_heading_empty_ring_experimental_return',inverse_mm_per_px=[[0,.5],[-.5,0]],
        observation_binding_sha256=binding,calibration_id=1,max_error_px=80,
        deadband_px=4,min_step_mm=1,max_step_mm=5,gain=.5,
        experimental_reviewed=True,ruler_checked=True,repeatability_checked=True,physical_directions_checked=True)
    value['sha256']=fingerprint(value)
    return value


def frame(now,sequence,error=(8,-8),**extra):
    return dict(CAMERA,domain='camera_live',camera='material',recognition_mode='ring',
        stream_id='stream-1',capture_epoch=0,frame_id=sequence,captured_s=now,age_ms=0,
        items=[dict(position_id=2,center_px=[640+error[0],360+error[1]],
                    frame_clipped=False,roi_clipped=False,digit_similarity=.9,digit_margin=.3)],**extra)


def fixture(mode='auto'):
    a=Arrival();l=link();a.phase='READY';a.identity=(123,456,1)
    a.body,a.binding=contract(observation_record(),1,1,0,1)
    l.base['mode']=3;l.execution.update(x_m=0,y_m=0,yaw_rad=0,pose_valid=True)
    c=Centering();c.set_calibration(calibration(a.body[36:68].hex()));c.begin(a,mode,0)
    return c,a,l


def stable(c,a,l,start=0,error=(8,-8)):
    saved=[]
    for i in range(8):
        now=start+i*.16
        l.actual_at=l.base_at=l.group_at=now
        value=c.advance(a,l,(frame(now,i+int(start*100)+1,error),now),now)
        if value:saved.append(value)
    return saved


class CenteringTests(unittest.TestCase):
    def test_target_is_image_center_and_requires_post_stop_stable_frames(self):
        c,a,l=fixture();stable(c,a,l)
        self.assertEqual(c.status,'MOVING');self.assertEqual(len(l.commands),1)
        self.assertEqual(c.last_observation['image_center_px'],[640,360])
        self.assertEqual(c.last_observation['delta_px'],[8,-8])
        wire=l.commands[0];self.assertEqual(wire[0],34)
        spec=SPEC_DATA.unpack(wire[1][8:]);self.assertAlmostEqual(spec[1],.002)
        self.assertEqual(spec[2],0)

    def test_no_calibration_or_wrong_pose_cannot_start_auto(self):
        c,a,l=fixture();c.calibration=None
        with self.assertRaises(ValueError):c.begin(a,'auto',0)
        c.set_calibration(calibration('b'*64))
        with self.assertRaises(ValueError):c.begin(a,'auto',0)
        self.assertFalse(l.commands)

    def test_duplicate_clipped_missing_old_and_uncertain_frames_never_move(self):
        for case in ('duplicate','clipped','missing','old','uncertain','barrier'):
            c,a,l=fixture()
            for i in range(9):
                now=i*.16;f=frame(now,i+1)
                if case=='duplicate':f['frame_id']=1
                if case=='clipped':f['items'][0]['frame_clipped']=True
                if case=='missing':f['items']=[]
                if case=='old':f['captured_s']=now-.3
                if case=='uncertain':f['items'][0]['digit_margin']=.01
                if case=='barrier':f['captured_s']=0
                c.advance(a,l,(f,now),now)
            with self.subTest(case=case):self.assertFalse(l.commands)

    def test_camera_restart_and_profile_change_pause_without_motor_request(self):
        for key,value in (('stream_id','stream-2'),('capture_epoch',1),('profile_sha256','b'*64)):
            c,a,l=fixture()
            c.advance(a,l,None,0)
            c.advance(a,l,(frame(.2,1),.2),.2)
            f=frame(.4,2);f[key]=value
            c.advance(a,l,(f,.4),.4)
            self.assertEqual(c.status,'PAUSED');self.assertFalse(l.commands)

    def test_centered_requires_return_ack_and_never_sends_mechanism_target(self):
        c,a,l=fixture();stable(c,a,l,error=(1,-2))
        self.assertEqual(c.status,'RETURNING');self.assertEqual(a.phase,'RETURN')
        self.assertFalse(l.commands)
        a.phase='DONE';c.advance(a,l,None,2)
        self.assertEqual(c.status,'CENTERED')
        self.assertFalse(c.completed['mechanical_actions_sent'])

    def test_minimum_step_can_finish_near_deadband_without_false_done(self):
        c,a,l=fixture();c.calibration['deadband_px']=3
        result=c._step([3.2,0]);self.assertEqual(result['state'],'STEP')
        self.assertEqual(result['command_mm'],[0,1])
        self.assertFalse(l.commands)

    def test_four_direction_capture_fits_draft_but_cannot_authorize_auto(self):
        c,a,l=fixture('manual');seq=0;now=0.
        error=[20.,-20.]
        for axis,mm in (('x',5),('x',-5),('y',5),('y',-5)):
            for _ in range(8):
                now+=.16;seq+=1
                c.advance(a,l,(frame(now,seq,error),now),now)
            c.note_move(l,axis,mm,now)
            self.assertIsNotNone(c.pending_measure)
            a.steps+=1;a.action_id+=1;a.phase='MOVING'
            c.advance(a,l,None,now+.01)
            if axis=='x':error[1]+=2*mm;l.execution['x_m']+=mm/1000
            else:error[0]-=2*mm;l.execution['y_m']+=mm/1000
            a.phase='READY';l.execution['action_id']=a.action_id
            for _ in range(8):
                now+=.16;seq+=1
                saved=c.advance(a,l,(frame(now,seq,error),now),now)
                if saved:snapshot=saved
        self.assertEqual(len(c.records),4)
        value=fit_records(snapshot,[1,2,3,4])
        self.assertEqual(value['inverse_mm_per_px'],[[0,.5],[-.5,0]])
        self.assertFalse(value['experimental_reviewed'])
        with self.assertRaises(ValueError):load_calibration(value)

    def test_missing_camera_pauses_instead_of_blind_motion_or_auto_resume(self):
        c,a,l=fixture();c.advance(a,l,None,0);c.advance(a,l,None,3.01)
        self.assertEqual(c.status,'PAUSED');stable(c,a,l,start=3.2)
        self.assertFalse(l.commands);self.assertEqual(c.status,'PAUSED')

    def test_yaw_change_pauses_without_guessing_rotation(self):
        c,a,l=fixture();c.advance(a,l,None,0)
        l.execution['yaw_rad']=math.radians(2)
        c.advance(a,l,(frame(.2,1),.2),.2)
        self.assertEqual(c.status,'PAUSED');self.assertFalse(l.commands)

    def test_auto_turns_to_corrected_heading_before_looking_at_ring(self):
        c,a,l=fixture();l.firmware='GC-BASE-0.3.52';c.begin(a,'auto',0,yaw_correction_deg=2)
        c.advance(a,l,None,0)
        self.assertEqual(c.status,'MOVING');self.assertEqual(len(l.commands),1)
        spec=SPEC_DATA.unpack(l.commands[0][1][8:])
        self.assertEqual(spec[:3],(2,0,0));self.assertAlmostEqual(spec[3],math.radians(2),places=6)
        # 航向进入0.3°内才按圆环像素走平移小步。
        a.phase='READY';a.waiting=None;l.commands.clear();l.execution['yaw_rad']=math.radians(1.9)
        stable(c,a,l,start=1)
        self.assertEqual(len(l.commands),1);self.assertEqual(SPEC_DATA.unpack(l.commands[0][1][8:])[0],1)
        # 偏差超出3°修正能力即暂停，不连续转向追赶。
        c,a,l=fixture();l.firmware='GC-BASE-0.3.52';c.begin(a,'auto',0,yaw_correction_deg=3)
        c.advance(a,l,None,0);self.assertEqual(len(l.commands),1)
        a.phase='READY';a.waiting=None;l.commands.clear();l.execution['yaw_rad']=math.radians(-.5)
        c.advance(a,l,None,.1)
        self.assertEqual(c.status,'PAUSED');self.assertFalse(l.commands)
        with self.assertRaises(ValueError):c.begin(a,'auto',0,yaw_correction_deg=3.5)

    def test_unreviewed_bad_hash_and_wrong_goal_rejected(self):
        c,a,l=fixture()
        for key,value in (('ruler_checked',False),('goal','old_hand_placed_point'),('sha256','b'*64)):
            v=copy.deepcopy(c.calibration);v[key]=value
            if key!='sha256':v['sha256']=fingerprint({k:v for k,v in v.items() if k!='sha256'})
            with self.subTest(key=key),self.assertRaises(ValueError):load_calibration(v)


class CenteringCoreTests(unittest.TestCase):
    def core(self, runtime=None):
        from robot.mech_core import DeviceCore
        from test_mech_sequence import SequenceTests
        core=DeviceCore(clock=lambda:1.01,runtime=runtime,site_ready=True,commissioning=True)
        core.link=SequenceTests().ready();core.link.firmware='GC-BASE-0.3.45'
        core.link.handoff=dict(id=0,epoch=0,phase=0);core.link.handoff_at=1.
        core.sequences=[observation_record()]
        response=core.command(dict(op='claim',client='operator',request_id='claim'))
        self.token=response['token']
        return core

    def command(self,core,action,**extra):
        return core.command(dict(op='mechanism',client='operator',token=self.token,
            request_id=action,args=dict(action=action,**extra)))

    def test_auto_start_without_reviewed_matrix_has_no_uart_side_effect(self):
        core=self.core();r=core.sequences[0]
        with self.assertRaises(ValueError):self.command(core,'alignment_start',id=r['id'],revision=1,
            sha256=r['sha256'],order=1,mode='auto',empty_path_confirmed=True,actual_pose_confirmed=True)
        self.assertFalse(core.link.commands)
        self.assertFalse(core.arrival.active)
        self.assertFalse(core.link.latched)

    def test_browser_cannot_supply_pixels_or_matrix_instead_of_pi_camera(self):
        core=self.core();r=core.sequences[0]
        self.command(core,'alignment_start',id=r['id'],revision=1,sha256=r['sha256'],order=1,
            empty_path_confirmed=True,actual_pose_confirmed=True,delta_px=[8,8],matrix=[[1,0],[0,1]])
        self.assertIsNone(core.centering.calibration)
        self.assertIsNone(core.centering.last_observation)
        self.assertFalse(core.link.commands)

    def test_profile_is_resolved_in_local_pi_and_flat_remote_layout(self):
        from unittest.mock import patch
        root=Path(__file__).resolve().parents[1]
        for package in ('pi','deployment'):
            with tempfile.TemporaryDirectory() as folder:
                project=Path(folder)/'checkout'
                module=project/package/'robot/mech_core.py';module.parent.mkdir(parents=True);module.touch()
                configs=(project if package=='pi' else project/package)/'config';configs.mkdir()
                (configs/'base_trials.json').write_bytes((root/'config/base_trials.json').read_bytes())
                core=self.core();r=core.sequences[0]
                with patch('robot.mech_core.__file__',str(module)):
                    value=self.command(core,'alignment_start',id=r['id'],revision=1,sha256=r['sha256'],order=1,
                        empty_path_confirmed=True,actual_pose_confirmed=True)
                self.assertEqual(value['status'],'ALIGNMENT_PREPARING');self.assertFalse(core.link.commands)

    def test_stop_cancels_auto_and_does_not_restore_it_on_claim(self):
        core=self.core();core.arrival.phase='READY';core.centering.mode='auto';core.centering.status='MOVING'
        core.stop_control('operator','test','operator')
        self.assertEqual(core.centering.status,'CANCELLED')
        self.assertEqual(core.arrival.phase,'FAILED');self.assertFalse(core.token)

    def test_corrupt_history_is_preserved_and_writes_disabled(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'rough_centering_samples.json';original='{"sha256":"wrong"}'
            path.write_text(original,encoding='utf-8');core=self.core(folder)
            self.assertFalse(core.centering_records_writable)
            self.assertEqual(path.read_text(encoding='utf-8'),original)

    def test_idle_reader_never_opens_http_and_drains_pending_record_on_close(self):
        from unittest.mock import patch
        from robot.vision.ring_feed import RingFeed
        with tempfile.TemporaryDirectory() as folder,patch('robot.vision.ring_feed.build_opener') as opener:
            opener.return_value.open.side_effect=AssertionError('no HTTP when idle')
            feed=RingFeed(Path(folder));feed.save(dict(schema_version=1,records=[]));feed.start();feed.close()
            self.assertEqual(json.loads((Path(folder)/'rough_centering_samples.json').read_text(encoding='utf-8'))['records'],[])
            opener.return_value.open.assert_not_called()


class CenteringNativeLoopTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import test_execution as native
        native.ExecutionTests.setUpClass();cls.lib=native.ExecutionTests.lib

    def test_pixels_through_real_c_controller_to_center_then_return(self):
        import test_execution as native
        for initial in ((18,-14),(-16,12)):
            d=native.SimTransport(self.lib);d.open()
            profile=native.profile();profile.update(version=10,max_rpm=6)
            d.assert_ok(d.call(native.CONFIG,native.config_payload(profile)))
            d.sleep(.1);d.assert_ok(d.call(native.ARM));d.sleep(.1)
            c,a,l=fixture();done=False;sequence=0;now=0.
            for i in range(5000):
                now=i*.01;d.sleep(.01)
                values=native.STATUS_DATA.unpack(d.call(native.STATUS).payload[1:])
                self.assertEqual(values[1],0)
                l.base.update(mode=values[0],fault=values[1]);l.execution.update(action_id=values[2],
                    action_state=values[3],x_m=values[11],y_m=values[12],yaw_rad=values[13],pose_valid=bool(values[10]))
                l.actual_at=l.base_at=l.group_at=now
                a.advance(l,now)
                snapshot=None
                if i%16==0:
                    sequence+=1;error=[initial[0]-2000*d.plant.y,initial[1]+2000*d.plant.x]
                    snapshot=(frame(now,sequence,error),now)
                c.advance(a,l,snapshot,now)
                if l.commands:
                    kind,data,request=l.commands.popleft()
                    if kind==34:d.assert_ok(d.call(kind,data))
                    else:self.assertEqual(kind,71) # 交接ACK由独立完整Board往返测试覆盖。
                    l.results.append(dict(request_id=request,result=0,status='ACCEPTED'))
                if i%10==0:d.call(native.p.HEART,struct.pack('<I',i+1))
                if c.status=='CENTERED':done=True;break
                self.assertNotEqual(c.status,'PAUSED',c.reason)
            self.assertTrue(done)
            error=[initial[0]-2000*d.plant.y,initial[1]+2000*d.plant.x]
            self.assertLessEqual(math.hypot(*error),4)
            self.assertGreater(c.auto_steps,1);self.assertLessEqual(c.auto_steps,12)
