"""新调试参数全链离线验证，不连接任何设备。"""
from pathlib import Path
import math,struct,subprocess,unittest
from test_execution import ROOT,compile_c
from robot.device.mechanism import payload
from robot.mech_core import DeviceCore
from robot.web.workbench import device_router
from fastapi import FastAPI
from fastapi.testclient import TestClient
import test_physical_mechanism as fixture

class TuningTests(unittest.TestCase):
    def test_native_tuning(self):
        program=ROOT/'output/tests/mech-tuning.exe';control=ROOT/'firmware/User/Control'
        compile_c(program,[ROOT/'tests/mech_tuning.c',*(control/n for n in ('mech_guard.c','mech_axis.c','mech_servo.c')),ROOT/'firmware/User/Device/drive.c'])
        for case in range(9):
            with self.subTest(case=case):
                result=subprocess.run([str(program),str(case)],capture_output=True)
                self.assertEqual(result.returncode,0,result.stderr.decode(errors='replace'))

    def ready(self):
        link=fixture.PhysicalMechanismTests().ready();link.firmware='GC-BASE-0.3.31'
        link.commissioning=True;link.manual_debug=True;link.actual['zero_valid']=1
        return link

    def test_speed_units_and_timeout(self):
        link=self.ready();link.submit('arm1','move',dict(axis='arm',target_rad=math.pi/2,speed_rad_s=math.pi/6,zero_version=1),1.01)
        kind,data,_=link.commands.popleft();self.assertEqual((kind,len(data)),(53,26))
        self.assertEqual(struct.unpack_from('<I',data,22)[0],30000)
        for speed in (0,361*math.pi/180,float('nan')):
            with self.assertRaises(ValueError):payload('move',dict(axis='arm',target_rad=math.pi,speed_rad_s=speed),{},1)

    def test_arm_direct_zero_and_270_without_offset(self):
        for angle in (0,30,83,180,270):
            link=self.ready()
            link.submit('angle','move',dict(axis='arm',target_rad=angle*math.pi/180,zero_version=1),1.01)
            kind,data,_=link.commands[0]
            self.assertEqual((kind,struct.unpack_from('<i',data,12)[0]),(53,angle*1000))
        for angle in (-1,271):
            link=self.ready()
            with self.assertRaises(ValueError):
                link.submit('bad','move',dict(axis='arm',target_rad=angle*math.pi/180,zero_version=1),1.01)

    def test_manual_axis_extended_but_legacy_keeps_speed_limit(self):
        link=self.ready();link.submit('axis1','move',dict(axis='lift',target_m=.14,rpm=1000,zero_version=1),1.01)
        kind,data,_=link.commands.popleft();self.assertEqual(kind,52)
        self.assertEqual(struct.unpack_from('<i',data,12)[0],140000)
        self.assertEqual(struct.unpack_from('<H',data,20)[0],1000)
        self.assertGreater(struct.unpack_from('<I',data,16)[0],5000) # 计入旧驱动加减速，不只按匀速估算。
        with self.assertRaises(ValueError):payload('move',dict(axis='lift',target_m=.15,rpm=1000),{},1)
        with self.assertRaises(ValueError):payload('move',dict(axis='lift',target_m=-.001,rpm=5,_manual_tuning=True),{},1)
        payload('move',dict(axis='reach',target_m=.107,rpm=1000,_manual_tuning=True),{},1)
        for axis,target in (('lift',.140001),('reach',.107001)):
            with self.subTest(axis=axis),self.assertRaises(ValueError):
                payload('move',dict(axis=axis,target_m=target,rpm=1000,_manual_tuning=True),{},1)

    def test_no_release_checkbox_required_and_mode_is_not_spoofable(self):
        link=self.ready();link.submit('claw1','move',dict(axis='gripper',target_rad=0,zero_version=1),1.01)
        self.assertEqual(link.commands[0][1][21],1)
        link=self.ready();link.manual_debug=False
        with self.assertRaises(ValueError):link.submit('bad','move',dict(axis='arm',target_rad=math.pi,_manual_tuning=True,zero_version=1),1.01)

    def test_web_explicit_target_and_duplicate_request(self):
        core=DeviceCore(clock=lambda:1.01,site_ready=True,commissioning=True);core.link=self.ready()
        class Client:
            def request(self,path,body=None):
                return core.state() if path=='/state' else core.command(body)
        app=FastAPI();app.include_router(device_router(Client()))
        with TestClient(app) as client:
            headers={'X-Workbench':'1'}
            token=client.post('/api/device/command',headers=headers,json=dict(op='claim',client='offline',request_id='claim')).json()['token']
            body=dict(op='mechanism',client='offline',token=token,request_id='servo1',
                      args=dict(action='move',axis='gripper',target_rad=0,speed_rad_s=math.pi/2,zero_version=1))
            response=client.post('/api/device/command',headers=headers,json=body)
            self.assertEqual(response.status_code,200);self.assertEqual(response.json()['status'],'QUEUED')
            self.assertEqual(len(core.link.commands),1)
            self.assertEqual(client.post('/api/device/command',headers=headers,json=body).json(),response.json())
            self.assertEqual(len(core.link.commands),1)
            kind,data,_=core.link.commands[0]
            self.assertEqual((kind,len(data),data[21]),(53,26,1))
            self.assertEqual(struct.unpack_from('<I',data,22)[0],90000)
