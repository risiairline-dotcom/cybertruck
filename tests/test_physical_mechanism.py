"""实车协议/唯一核心离线联调；不创建Serial对象。"""
import math
import struct
import tempfile
import unittest
from pathlib import Path
from fastapi.testclient import TestClient
from test_execution import ROOT
from robot.device import protocol as p
from robot.device.mechanism import MechanismLink,payload,FIELDS
from robot.mech_core import DeviceCore,create_app
from robot.web.workbench import device_router
from fastapi import FastAPI

class PhysicalMechanismTests(unittest.TestCase):
    def test_0324_commissioning_is_explicit_and_keeps_protection(self):
        with self.assertRaises(ValueError): DeviceCore(commissioning=True)
        for change in ('normal', 'fault', 'stale', 'zero', 'wrong_firmware'):
            link=self.ready();link.firmware='GC-BASE-0.3.24';link.commissioning=True
            if change=='fault':link.base['fault']=2048
            if change=='stale':link.actual_at=0
            if change=='zero':link.actual['zero_version']=2
            if change=='wrong_firmware':link.firmware='GC-BASE-0.3.23'
            args=dict(axis='lift',target_m=.002,zero_version=1,rpm=5)
            if change=='normal':
                result=link.submit('trial', 'move', args, 1.01)
                self.assertEqual(result['status'],'QUEUED')
            else:
                with self.subTest(change=change),self.assertRaises(ValueError):
                    link.submit('trial','move',args,1.01)
                self.assertFalse(link.commands)

    def test_0325_requires_explicit_commissioning(self):
        link=self.readonly_current('GC-BASE-0.3.25')
        with self.assertRaises(ValueError):link.submit('blocked','zero',dict(confirmed=True),1.01)
        link=self.ready();link.firmware='GC-BASE-0.3.25';link.commissioning=True
        self.assertEqual(link.submit('candidate','move',dict(axis='tray',target_rad=0,zero_version=1),1.01)['status'],'QUEUED')

    def readonly_current(self, firmware='GC-BASE-0.3.23'):
        link=MechanismLink();link.boot=123;link.bound=True;link.firmware=firmware
        link.actual_at=link.base_at=1.;link.actual=dict.fromkeys(FIELDS,0)
        link.actual['schema']=1;link.base=dict(mode=0,fault=0,stopped=True)
        return link

    def test_current_firmware_only_allows_readonly_attach(self):
        link=self.readonly_current()
        link.submit('read-only','attach',{},1.01)
        kind,data,_=link.commands[0]
        self.assertEqual((kind,struct.unpack('<II',data)),(48,(1,0x4D454348)))
        for action in ('enable','zero','anchor','move','confirm','next','pose','invalidate'):
            link=self.readonly_current()
            with self.subTest(action=action),self.assertRaises(ValueError):
                link.submit('blocked',action,{},1.01)
            self.assertFalse(link.commands)

    def test_0324_candidate_is_readonly_and_keeps_fault_gate(self):
        core=DeviceCore(clock=lambda:1.01);core.link=self.readonly_current('GC-BASE-0.3.24')
        core.command(dict(op='observe',client='offline',request_id='once'))
        self.assertEqual(core.link.commands[0][0],48)
        self.assertFalse(core.token);self.assertFalse(core.owner)
        for action in ('enable','zero','anchor','move','confirm','next','pose','invalidate'):
            link=self.readonly_current('GC-BASE-0.3.24')
            with self.subTest(action=action),self.assertRaises(ValueError):
                link.submit('blocked',action,{},1.01)
            self.assertFalse(link.commands)
        link=self.readonly_current('GC-BASE-0.3.24');link.base['fault']=2048
        with self.assertRaises(ValueError):link.submit('fault','attach',{},1.01)
        self.assertFalse(link.commands)

    def test_current_attach_rejects_fault_motion_pwm_and_stale_state(self):
        for field in ('attached','fault','zero_valid','pwm_mask','axis_state','servo_state','pose_state','motion_uncertain'):
            link=self.readonly_current();link.actual[field]=1
            with self.subTest(field=field),self.assertRaises(ValueError):link.submit('r','attach',{},1.01)
            self.assertFalse(link.commands)
        for field,value in [('mode',1),('fault',2048),('stopped',False)]:
            link=self.readonly_current();link.base[field]=value
            with self.subTest(field=field),self.assertRaises(ValueError):link.submit('r','attach',{},1.01)
        link=self.readonly_current();link.actual_at=0
        with self.assertRaises(ValueError):link.submit('r','attach',{},1.01)

    def test_readonly_observe_has_no_motion_lease_and_does_not_requeue_duplicates(self):
        core=DeviceCore(clock=lambda:1.01);core.link=self.readonly_current()
        command=dict(op='observe',client='acceptance',request_id='once')
        result=core.command(command)
        self.assertEqual(core.command(command),result)
        self.assertEqual(len(core.link.commands),1)
        self.assertFalse(core.token);self.assertFalse(core.owner);self.assertEqual(core.until,0)
        for args in ({'action':'enable'},{'action':'zero'},{'action':'move'}):
            other=DeviceCore(clock=lambda:1.01);other.link=self.readonly_current()
            with self.subTest(args=args),self.assertRaises(ValueError):
                other.command(dict(command,args=args))
            self.assertFalse(other.link.commands)

    def test_readonly_observe_refuses_other_owner_or_site_ready(self):
        for flag in ('owner','site'):
            core=DeviceCore(clock=lambda:1.01);core.link=self.readonly_current()
            if flag=='owner':core.token='someone'
            else:core.site_ready=True
            with self.subTest(flag=flag),self.assertRaises(ValueError):
                core.command(dict(op='observe',client='a',request_id='b'))
            self.assertFalse(core.link.commands)

    def ready(self):
        link=MechanismLink();link.boot=123;link.bound=True;link.firmware='GC-BASE-0.3.14'
        link.actual_at=link.base_at=1.;link.actual=dict.fromkeys(FIELDS,0)
        link.actual.update(base_stopped=1,zero_version=1,servo_id=8,axis_id=2,pose_id=3)
        link.base=dict(fault=0,stopped=True)
        return link

    def test_si_wire_layout_and_explicit_actions(self):
        state=dict(axis_id=3,servo_id=4,pose_id=0)
        kind,data=payload('move',dict(axis='lift',target_m=.002,zero_version=1,rpm=5),state,1)
        self.assertEqual(kind,52);self.assertEqual(len(data),23)
        self.assertEqual(struct.unpack('<IIIiIHB',data),(1,1,5,2000,15000,5,0))
        kind,data=payload('move',dict(axis='tray',target_rad=math.pi/6,zero_version=1),state,2)
        self.assertEqual((kind,len(data)),(53,22));self.assertEqual(struct.unpack_from('<i',data,12)[0],30000)
        with self.assertRaises(ValueError):payload('jog',{},state,1)
        with self.assertRaises(ValueError):payload('anchor',dict(targets={}),state,1)
        with self.assertRaises(ValueError):payload('pose',dict(revision=1),state,1)

    def test_queue_acceptance_is_not_completion_and_no_replay(self):
        link=self.ready()
        result=link.submit('req','move',dict(axis='lift',target_m=.002,zero_version=1),1.01)
        self.assertEqual(result['status'],'QUEUED');self.assertFalse(result['action_sent'])
        with self.assertRaises(ValueError):link.submit('second','move',dict(axis='lift',target_m=.002,zero_version=1),1.02)
        kind,data=link.extra_request(1.02)
        request=p.Packet(kind,123,link.session,1,250,data)
        reply=p.Packet(kind|128,123,link.session,1,0,bytes([0])+struct.pack('<I',0))
        self.assertTrue(link.custom_reply(reply,request,1.03))
        self.assertEqual(link.results[0]['status'],'ACCEPTED')
        link.reset('测试超时');self.assertTrue(link.latched)
        self.assertTrue(link.step(1.04));self.assertEqual(link.step(9),b'')
        self.assertFalse(link.commands)

    def test_fault_stale_wrong_firmware_zero_and_model_rejected(self):
        for change in ('fault','stale','firmware','zero'):
            link=self.ready();args=dict(axis='lift',target_m=.002,zero_version=1)
            if change=='fault':link.base['fault']=2048
            if change=='stale':link.actual_at=0
            if change=='firmware':link.firmware='GC-BASE-0.3.12'
            if change=='zero':args['zero_version']=2
            with self.subTest(change=change),self.assertRaises(ValueError):link.submit('r','move',args,1.01)
            self.assertFalse(link.commands)

    def test_lease_expiry_queues_stop_not_motion(self):
        core=DeviceCore(clock=lambda:2.)
        core.link=self.ready();core.owner='a';core.token='b';core.until=1.
        class Transport:
            writes=[]
            def write(self,data):self.writes.append(data);return len(data)
            def read(self,size):return b''
        transport=Transport();core.tick(transport)
        self.assertTrue(core.link.action_locked);self.assertFalse(core.link.latched);self.assertEqual(core.token,'')
        packet=p.Decoder().feed(transport.writes[0])[0]
        self.assertEqual(packet.kind,p.STOP)

    def test_reply_after_deadline_does_not_refresh_state(self):
        for received,late in ((1.249,False),(1.25,True),(1.3,True)):
            link=self.ready()
            request=p.Packet(p.STOP,123,link.session,7,250,b'')
            link.pending=(request,1.)
            link.feed(p.Packet(p.STOP|128,123,link.session,7,0,bytes([p.OK])).encode(),received)
            with self.subTest(received=received):
                self.assertIsNone(link.pending)
                self.assertEqual(link.latched,late)
                self.assertEqual(link.stop_acknowledged,not late)

    def test_transport_failure_ends_sequence_and_control(self):
        core=DeviceCore(clock=lambda:2.);core.link=self.ready()
        core.owner='page';core.token='secret';core.until=3.;core.sequence.status='RUNNING'
        wire=core.halt(OSError('串口断开'))
        self.assertEqual(p.Decoder().feed(wire)[0].kind,p.STOP)
        self.assertEqual(core.sequence.status,'CANCELLED')
        self.assertEqual((core.owner,core.token),('',''))
        self.assertTrue(core.link.latched);self.assertFalse(core.link.stop_acknowledged)
        self.assertEqual(core.state()['transport_halted'],'串口断开')
        with self.assertRaises(ValueError):core.command(dict(op='claim',client='page',request_id='again'))

    def test_teach_save_failure_keeps_previous_versions(self):
        from unittest import mock
        with tempfile.TemporaryDirectory() as folder:
            core=DeviceCore(clock=lambda:1.01,runtime=folder,site_ready=True);core.link=self.ready()
            core.command(dict(op='claim',client='test',request_id='claim'))
            targets=dict(lift=.002,reach=0.,arm=math.pi,tray=math.pi/6,gripper=98*math.pi/180)
            teach=dict(op='mechanism',client='test',token=core.token,request_id='teach',
                       args=dict(action='teach',pose_id='test_pose',targets=targets))
            with mock.patch('robot.workbench.storage.write_json',side_effect=OSError('磁盘失败')):
                for _ in range(2):
                    with self.assertRaises(OSError):core.command(teach)
            self.assertEqual(core.poses,[])
            self.assertFalse((Path(folder)/'physical_poses.json').exists())
            self.assertEqual(core.command(teach)['revision'],1)
            self.assertFalse(core.link.commands)

    def test_core_saved_versions_frozen_and_duplicate_dedup(self):
        with tempfile.TemporaryDirectory() as folder:
            core=DeviceCore(clock=lambda:1.01,runtime=folder,site_ready=True);core.link=self.ready()
            def cmd(op,args=None,request='r'):
                return dict(op=op,client='test',token=core.token,request_id=request,args=args or {})
            core.command(cmd('claim'))
            targets=dict(lift=.002,reach=0.,arm=math.pi,tray=math.pi/6,gripper=98*math.pi/180)
            teach=cmd('mechanism',dict(action='teach',pose_id='test_pose',targets=targets))
            self.assertEqual(core.command(teach)['status'],'SAVED_DRAFT')
            core.command(teach);self.assertEqual(len(core.poses),1);self.assertFalse(core.link.commands)
            self.assertTrue((Path(folder)/'physical_poses.json').exists())
            run=cmd('mechanism',dict(action='pose',pose_id='test_pose',revision=1,zero_version=1,physical_taught=True),'run')
            self.assertEqual(core.command(run)['status'],'QUEUED')
            core.command(run);self.assertEqual(len(core.link.commands),1)
            run['args']['targets']=dict(targets,lift=.04)
            with self.assertRaises(ValueError):core.command(run)

    def test_gateway_same_origin_and_unavailable_does_not_open_device(self):
        class Client:
            def request(self,path,body=None):return dict(domain='physical',available=True,received=body)
        app=FastAPI();app.include_router(device_router(Client()))
        with TestClient(app) as web:
            self.assertEqual(web.get('/api/device/state').json()['domain'],'physical')
            body=dict(op='stop',client='test',request_id='r')
            self.assertEqual(web.post('/api/device/command',json=body).status_code,403)
            self.assertEqual(web.post('/api/device/command',json=body,headers={'X-Workbench':'1','Origin':'https://evil.example'}).status_code,403)
            self.assertEqual(web.post('/api/device/command',json=body,headers={'X-Workbench':'1'}).status_code,200)

    def test_no_motion_permission_after_flash_defaults_readonly(self):
        core=DeviceCore(clock=lambda:1.01);core.link=self.ready()
        claim=dict(op='claim',client='test',request_id='claim')
        core.command(claim)
        command=dict(op='mechanism',client='test',request_id='move',token=core.token,
                     args=dict(action='move',axis='lift',target_m=.002,zero_version=1))
        with self.assertRaises(ValueError):core.command(command)
        self.assertFalse(core.link.commands)
        self.assertFalse(core.state()['site_ready'])

    def test_manual_confirmation_requires_fresh_command_done(self):
        link=self.ready();link.actual.update(servo_state=5,uptime_ms=1200,channel=2,target_mdeg=180000)
        kind,data=payload('confirm',dict(zero_version=1,confirmed=True),link.actual,9)
        self.assertEqual((kind,len(data)),(54,29))
        link.actual['servo_state']=4
        with self.assertRaises(ValueError):payload('confirm',dict(confirmed=True),link.actual,10)

if __name__=='__main__':unittest.main()
