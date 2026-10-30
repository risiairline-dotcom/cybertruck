import math
import struct
import unittest
from collections import deque
from types import SimpleNamespace

from robot.device.arrival import Arrival, contract, micro_goal
from robot.device.mech_sequence import record
from robot.device.execution import SPEC_DATA


def observation_record():
    rows=[dict(order=1,axis=k,target_m=0,rpm=10) for k in ('lift','reach')]
    rows += [dict(order=1,axis=k,target_rad=v,speed_rad_s=1) for k,v in
             (('tray',0),('arm',math.radians(185)),('gripper',0))]
    return record('rough2_observe',rows,1)


def link():
    return SimpleNamespace(firmware='GC-BASE-0.3.45',commissioning=True,latched=False,
        action_locked=False,bound=True,boot=123,session=456,actual_at=0,base_at=0,group_at=0,
        handoff_at=0,handoff=dict(phase=0,id=0,epoch=0),
        actual=dict(fault=0,zero_valid=True,zero_version=1),base=dict(fault=0,stopped=True,mode=0),
        execution=dict(config_version=0),commands=deque(),operation=None,results=deque(),start_event={})


class ArrivalTests(unittest.TestCase):
    def test_0346_only_prepares_existing_handoff_without_motion(self):
        import json
        from pathlib import Path
        a=Arrival();l=link();l.firmware='GC-BASE-0.3.46'
        profile=json.loads((Path(__file__).resolve().parents[1]/'config/base_trials.json').read_text(encoding='utf-8'))
        a.start(l,observation_record(),1,dict(empty_path_confirmed=True,actual_pose_confirmed=True,profile=profile),0)
        self.assertEqual(a.phase,'PREPARE');self.assertEqual(a.steps,0);self.assertFalse(l.commands)
        a.advance(l,0);self.assertEqual(l.commands[0][0],71)

    def test_wire_contract_and_changed_revision(self):
        r=observation_record();data,binding=contract(r,1,1,0,5)
        self.assertEqual(len(data),72)
        self.assertEqual(struct.unpack_from('<4I',data),(5,0,1,1))
        self.assertEqual(struct.unpack_from('<5i',data,16),(0,0,0,185000,0))
        r['rows'][0]['target_m']=.1
        with self.assertRaises(ValueError):contract(r,1,1,0,6)

    def test_four_directions_and_bounds(self):
        for axis in ('x','y'):
            for mm in (-5,-1,1,5):
                wire=micro_goal(1,axis,mm);self.assertEqual(len(wire),96)
                spec=SPEC_DATA.unpack(wire[8:]);self.assertEqual(spec[0],1)
                self.assertAlmostEqual(spec[1 if axis=='x' else 2],mm/1000)
        for v in (0,.5,5.01,float('nan'),float('inf')):
            with self.assertRaises(ValueError):micro_goal(1,'x',v)

    def test_turn_goal_and_firmware_gate(self):
        for degrees in (-3,-.1,.1,3):
            spec=SPEC_DATA.unpack(micro_goal(1,'yaw',degrees)[8:])
            self.assertEqual(spec[:3],(2,0,0))
            self.assertAlmostEqual(spec[3],math.radians(degrees),places=6)
        for v in (0,.05,3.01,float('nan')):
            with self.assertRaises(ValueError):micro_goal(1,'yaw',v)
        a=Arrival();l=link();a.phase='READY';a.identity=(123,456,1)
        l.firmware='GC-BASE-0.3.51'
        with self.assertRaises(ValueError):a.move(l,'yaw',1,0)
        self.assertFalse(l.commands)
        l.firmware='GC-BASE-0.3.52';a.move(l,'yaw',1,0)
        self.assertEqual(l.commands[0][0],34);self.assertEqual(a.phase,'GOAL')

    def test_old_board_and_stale_feedback_refused(self):
        a=Arrival();l=link();args=dict(empty_path_confirmed=True,actual_pose_confirmed=True,profile={})
        l.firmware='GC-BASE-0.3.44'
        with self.assertRaises(ValueError):a.start(l,observation_record(),1,args,0)
        l.firmware='GC-BASE-0.3.45'
        with self.assertRaises(ValueError):a.start(l,observation_record(),1,args,.251)

    def test_session_change_stops_without_replaying(self):
        a=Arrival();l=link();a.phase='READY';a.identity=(123,456,1);l.session=789
        self.assertIsNotNone(a.advance(l,0));self.assertEqual(a.phase,'FAILED');self.assertFalse(l.commands)

    def test_missing_motion_ack_is_never_replayed(self):
        a=Arrival();l=link();a.phase='READY';a.identity=(123,456,1)
        a.move(l,'y',-5,0);l.commands.clear()
        l.actual_at=l.base_at=l.group_at=2.01
        self.assertIn('未确认',a.advance(l,2.01))
        self.assertEqual(a.phase,'FAILED');self.assertFalse(l.commands)

    def test_explicit_stop_cancels_handoff_without_restarting_it(self):
        from robot.mech_core import DeviceCore
        core=DeviceCore();core.arrival.phase='RETURN';core.arrival.waiting=('one',0,71)
        core.stop_control('operator stop','test','operator')
        self.assertEqual(core.arrival.phase,'FAILED')
        self.assertIsNone(core.arrival.waiting)
        self.assertIsNone(core.arrival.advance(core.link,1))

    def test_rejected_return_retries_contract_but_not_motion(self):
        a=Arrival();l=link();a.phase='RETURN';a.identity=(123,456,1);a.body=b'c'*72
        a.advance(l,0);wire=l.commands.popleft()
        l.results.append(dict(request_id=wire[2],result=3,status='REJECTED'))
        a.advance(l,0);a.advance(l,0);retry=l.commands.popleft()
        self.assertEqual(wire[1],retry[1]);self.assertEqual(retry[0],71)
        l.results.append(dict(request_id=retry[2],result=7,status='REJECTED'))
        self.assertIsNotNone(a.advance(l,0));self.assertFalse(l.commands)

    def test_request_ack_flow_then_explicit_step(self):
        import json
        from pathlib import Path
        a=Arrival();l=link()
        profile=json.loads((Path(__file__).resolve().parents[1]/'config/base_trials.json').read_text(encoding='utf-8'))
        a.start(l,observation_record(),1,dict(empty_path_confirmed=True,actual_pose_confirmed=True,profile=profile),0)
        l.execution['pose_valid']=True;now=0
        for kind in (71,71,32,59):
            if kind==59:
                # 配置10生效后先等0.4秒（板端补读四轮细分），期间不请求PC0准备。
                a.advance(l,now);self.assertEqual(a.phase,'KEY_PREPARE');self.assertFalse(l.commands)
                now=.39;l.actual_at=l.base_at=l.group_at=now
                a.advance(l,now);self.assertFalse(l.commands)
                now=.8;l.actual_at=l.base_at=l.group_at=now
            a.advance(l,now);wire=l.commands.popleft();self.assertEqual(wire[0],kind)
            if kind==59:
                l.start_event=dict(ticket=struct.unpack_from('<I',a.body)[0],version=10,phase=4,fired=True,at=now)
            else:l.results.append(dict(request_id=wire[2],result=0,status='ACCEPTED'))
        a.advance(l,now)
        a.advance(l,now);wire=l.commands.popleft();self.assertEqual(wire[0],33)
        l.results.append(dict(request_id=wire[2],result=0,status='ACCEPTED'));l.base['mode']=3
        a.advance(l,now);self.assertEqual(a.phase,'READY')
        a.move(l,'x',5,now);wire=l.commands.popleft();self.assertEqual(wire[0],34)
        l.results.append(dict(request_id=wire[2],result=0,status='ACCEPTED'))
        a.advance(l,now);self.assertEqual(a.phase,'MOVING')
        l.execution.update(action_id=a.action_id,action_state=4)
        a.advance(l,now);self.assertEqual(a.phase,'READY')
        a.finish();a.advance(l,now);wire=l.commands.popleft();self.assertEqual(struct.unpack_from('<I',wire[1])[0],4)
        l.results.append(dict(request_id=wire[2],result=0,status='ACCEPTED'))
        a.advance(l,now);self.assertEqual(a.phase,'DONE')

    def test_board_timeout_reports_actual_reason_not_generic_fault(self):
        a=Arrival();l=link();a.phase='WAIT_PC0';a.identity=(123,456,1)
        a.body=struct.pack('<I',1);a.started=0
        l.actual_at=l.base_at=l.group_at=60
        l.start_event=dict(ticket=1,version=10,phase=5,fired=False,at=60,reason=3,guard_mask=0)
        self.assertIn('满60秒超时',a.advance(l,60))
        self.assertEqual(a.phase,'FAILED');self.assertEqual(a.steps,0);self.assertFalse(l.commands)

    def test_invalid_pose_wait_has_stage_deadline(self):
        a=Arrival();l=link();a.phase='KEY_PREPARE';a.identity=(123,456,1);a.started=0
        l.execution['pose_valid']=False
        for now in (0,2,5):
            l.actual_at=l.base_at=l.group_at=now
            self.assertIsNone(a.advance(l,now))
        l.actual_at=l.base_at=l.group_at=5.01
        self.assertIn('位姿持续无效',a.advance(l,5.01))
        self.assertEqual(a.phase,'FAILED');self.assertFalse(l.commands)

    def test_arming_and_moving_waits_are_bounded(self):
        for phase,limit in (('ARMING',3),('MOVING',12)):
            a=Arrival();l=link();a.phase=phase;a.identity=(123,456,1)
            self.assertIsNone(a.advance(l,0))
            l.actual_at=l.base_at=l.group_at=limit+.01
            self.assertIsNotNone(a.advance(l,limit+.01))
            self.assertEqual(a.phase,'FAILED');self.assertFalse(l.commands)

    def test_old_ticket_or_old_start_frame_cannot_authorize_arm(self):
        for ticket,stamp in ((2,1),(1,.749)):
            a=Arrival();l=link();a.phase='WAIT_PC0';a.identity=(123,456,1)
            a.body=struct.pack('<I',1);l.actual_at=l.base_at=l.group_at=1
            l.start_event=dict(ticket=ticket,version=10,phase=4,fired=True,at=stamp)
            self.assertIn('反馈身份不符或过期',a.advance(l,1))
            self.assertFalse(l.commands)
