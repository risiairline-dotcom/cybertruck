"""单件转存正式模型闭环，不创建真实设备或物料计数。"""
import json
import math
import os
import subprocess
import sys
import tempfile
import unittest
from test_execution import ROOT
from robot.workbench.mechanism import Mechanism
from robot.workbench.transfer import Transfer
from robot.workbench.contracts import MechGoal,TransferGoal

class TransferTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.now=100.
        self.mech=Mechanism(self.temp.name,lambda:self.now)
        self.mech.command(MechGoal(action='zero',confirmed=True))
        self.mech.command(MechGoal(action='anchor',zero_version=1,confirmed=True))
        self.flow=Transfer(self.mech)
    def tearDown(self):self.temp.cleanup()
    def command(self,action,**args):return self.flow.command(TransferGoal(action=action,**args))
    def tick(self):
        self.now+=.02;self.mech.tick(.02);self.flow.tick()
    def finish(self,manual=False):
        for _ in range(50001):
            self.tick()
            if self.flow.mode=='WAIT_OBSERVATION' and manual:
                self.command('confirm',action_id=self.flow.id,step_index=self.flow.index,verdict=self.flow.steps[self.flow.index]['expected'])
            if not self.flow.busy:return
        self.fail('model did not finish before bounded deadline')
    def test_preview_does_not_move_or_save_pose(self):
        before=dict(self.mech.positions);plan=self.command('preview',slot=2)
        self.assertEqual(self.mech.positions,before);self.assertEqual(self.mech.action_id,0)
        self.assertEqual(plan['slot_angle_deg'],270);self.assertFalse(self.mech.pose_versions)
        self.assertEqual([s['expected'] for s in plan['steps'] if s['kind']=='observe'],['PICKED','STORED','RETRIEVED','PLACED'])
    def test_complete_three_slots_and_six_colors(self):
        for slot in range(3):
            for color in ('red','green','blue','yellow','cyan','black'):
                with self.subTest(slot=slot,color=color):
                    self.command('run',confirmed=True,auto_model=True,slot=slot,color=color)
                    self.finish();self.assertEqual(self.flow.mode,'COMPLETE',self.flow.reason)
                    self.assertEqual(self.flow.ledger,dict(source='EMPTY',gripper='EMPTY',slots=['EMPTY']*3,destination=color))
                    self.assertEqual(len(self.flow.events),4)
                    self.assertTrue(all(e['evidence']=='synthetic_fixture' for e in self.flow.events))
                    self.assertTrue(self.mech.transport_ready())
                    self.assertFalse(self.mech.hold_possible)
    def test_manual_observation_and_duplicate_identity(self):
        self.command('run',confirmed=True)
        for _ in range(5000):
            self.tick()
            if self.flow.mode=='WAIT_OBSERVATION':break
        self.assertEqual(self.flow.mode,'WAIT_OBSERVATION')
        index=self.flow.index
        self.assertEqual(self.flow.ledger['gripper'],'EMPTY')
        with self.assertRaises(ValueError):self.command('confirm',action_id=999,step_index=index,verdict='PICKED')
        with self.assertRaises(ValueError):self.command('confirm',action_id=self.flow.id,step_index=index,verdict='STORED')
        self.command('confirm',action_id=self.flow.id,step_index=index,verdict='PICKED')
        with self.assertRaises(ValueError):self.command('confirm',action_id=self.flow.id,step_index=index,verdict='PICKED')
        self.finish(manual=True);self.assertEqual(self.flow.mode,'COMPLETE')
        self.assertTrue(all(e['evidence']=='operator_model' for e in self.flow.events))
    def test_unknown_is_not_success_and_cannot_rerun(self):
        self.command('run',confirmed=True)
        for _ in range(5000):
            self.tick()
            if self.flow.mode=='WAIT_OBSERVATION':break
        self.command('confirm',action_id=self.flow.id,step_index=self.flow.index,verdict='UNKNOWN')
        self.assertEqual(self.flow.mode,'FAILED');self.assertEqual(self.flow.ledger['gripper'],'UNKNOWN')
        self.assertFalse(self.flow.events)
        with self.assertRaises(ValueError):self.command('run',confirmed=True)
    def test_cancel_freezes_gripper_and_invalidates_material(self):
        self.command('run',confirmed=True,auto_model=True);self.tick()
        position=self.mech.positions['gripper'];self.command('stop')
        self.assertEqual(self.mech.positions['gripper'],position)
        self.assertEqual(self.flow.mode,'CANCELLED');self.assertEqual(self.flow.ledger['slots'][0],'UNKNOWN')
        self.assertFalse(self.mech.busy)
    def test_zero_change_and_base_fault_cancel(self):
        for kind in ('zero','base','config','timeout'):
            with self.subTest(kind=kind):
                self.flow=Transfer(self.mech)
                if self.flow.mode=='INTERRUPTED':
                    self.mech.hold_possible=False;self.command('reset',confirmed=True)
                self.mech.zero_valid=True;self.mech.zero_version=1;self.mech.config_hash='candidate';self.mech.updated=self.now
                self.command('run',confirmed=True,auto_model=True)
                if kind=='zero':self.mech.zero_version+=1
                if kind=='config':self.mech.config_hash='different'
                if kind=='timeout':self.now+=1001
                self.flow.tick(kind!='base')
                self.assertEqual(self.flow.mode,'FAILED')
    def test_frozen_recipe_and_restart_no_replay(self):
        self.command('run',confirmed=True,auto_model=True,slot=1)
        frozen=self.flow.view();frozen['plan']['steps'].clear()
        self.assertTrue(self.flow.steps)
        restarted=Transfer(self.mech)
        self.assertEqual(restarted.mode,'INTERRUPTED')
        self.assertEqual(restarted.ledger['gripper'],'UNKNOWN')
    def test_invalid_geometry_no_partial_action(self):
        with self.assertRaises(ValueError):self.command('run',confirmed=True,raw_lift_m=.04,raw_reach_m=.05)
        self.assertEqual(self.mech.action_id,0);self.assertFalse(self.flow.busy)


class TransferCoreTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.library=ROOT/'output/tests'/('transfer.dll' if os.name=='nt' else 'transfer.so')
        subprocess.run([sys.executable,'-B',str(ROOT/'tools/build_odom.py'),'--output',str(cls.library)],
                       check=True,capture_output=True)

    def setUp(self):
        from robot.workbench.core import create_core
        self.temp=tempfile.TemporaryDirectory();self.now=100.;self.seq=0
        self.app=create_core(self.library,self.temp.name,background=False)
        self.core=self.app.state.core;self.core.clock=lambda:self.now;self.core.mech.clock=lambda:self.now;self.core.mech.updated=self.now
        self.token=''
        self.send('claim')
        self.send('mechanism',dict(action='zero',confirmed=True))
        self.send('mechanism',dict(action='anchor',zero_version=1,confirmed=True))
    def tearDown(self):self.core.close();self.temp.cleanup()
    def send(self,op,args=None,packet=None):
        self.seq+=1
        packet=packet or dict(op=op,args=args or {},client='transfer_test',token=self.token,request_id=f'r{self.seq}',sequence=self.seq)
        result=self.core.command(packet)
        if 'token' in result:self.token=result['token']
        return result
    def advance(self,count=1):
        for i in range(count):
            self.now+=.01
            if i%30==0:self.send('renew')
            self.core.tick(.01)
    def test_full_core_accelerated_cycle_and_manual_hijack_rejected(self):
        from robot.workbench.service import Rejected
        self.send('transfer',dict(action='run',confirmed=True,auto_model=True,pace=20,slot=2,color='cyan'))
        with self.assertRaises(Rejected):self.send('mechanism',dict(action='move',axis='lift',target_m=.002,zero_version=1))
        with self.assertRaises(Rejected):self.send('move',dict(x_m=.1))
        self.advance(3000)
        value=self.core.snapshot()
        self.assertEqual(value['transfer']['mode'],'COMPLETE',value['transfer']['reason'])
        self.assertEqual(value['transfer']['ledger']['destination'],'cyan')
        self.assertFalse(value['transfer']['physical_validated'])
        self.assertEqual(len(value['transfer']['events']),4)
    def test_lease_expires_during_observation_and_no_automatic_resume(self):
        self.send('transfer',dict(action='run',confirmed=True,pace=20))
        self.advance(400)
        self.assertEqual(self.core.transfer.mode,'WAIT_OBSERVATION')
        self.now=self.core.lease_until+.01
        self.core.snapshot()
        self.assertEqual(self.core.transfer.mode,'CANCELLED')
        self.assertEqual(self.core.transfer.ledger['gripper'],'UNKNOWN')
    def test_core_gap_protection_is_not_accelerated_away(self):
        self.send('transfer',dict(action='run',confirmed=True,auto_model=True,pace=20))
        self.advance(2);self.now+=.3;self.core.tick(.3)
        self.assertEqual(self.core.transfer.mode,'FAILED')
    def test_gateway_routes_to_offline_transfer_and_deduplicates(self):
        from fastapi.testclient import TestClient
        from robot.web.app import create_app
        from fastapi import HTTPException
        device=TestClient(self.app)
        class Client:
            def request(_,path,body=None):
                r=device.get(path) if body is None else device.post(path,json=body)
                if r.status_code!=200:raise HTTPException(r.status_code,r.json()['detail'])
                return r.json()
        web=TestClient(create_app(core_client=Client()))
        self.seq+=1
        packet=dict(op='transfer',args=dict(action='run',confirmed=True,auto_model=True,pace=20),client='transfer_test',token=self.token,request_id='once',sequence=self.seq)
        a=web.post('/api/workbench/command',json=packet,headers={'X-Workbench':'1'})
        b=web.post('/api/workbench/command',json=packet,headers={'X-Workbench':'1'})
        self.assertEqual(a.status_code,200);self.assertEqual(a.json()['id'],b.json()['id'])
        self.assertEqual(self.core.transfer.id,1)
        self.advance(3000)
        self.assertEqual(web.get('/api/workbench/state').json()['transfer']['mode'],'COMPLETE')

if __name__=='__main__':unittest.main()
