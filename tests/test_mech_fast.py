"""高速机械联调：物理容差、首次快照及请求速度的离线负对照。"""
import copy,math,struct,subprocess,tempfile,unittest
from pathlib import Path
from test_execution import ROOT,compile_c
import test_mech_port as port_fixture
import test_mech_sequence as seq_fixture
from robot.device.mech_sequence import Sequence,record
from robot.device.mechanism import payload
from robot.device import protocol as p


class FastMechanismTests(unittest.TestCase):
    def test_native_tolerance_handoff_snapshot_and_faults(self):
        control=ROOT/'firmware/User/Control';program=ROOT/'output/tests/mech-fast.exe'
        compile_c(program,[ROOT/'tests/mech_fast.c',control/'mech_guard.c',control/'mech_axis.c',control/'mech_servo.c',control/'mech_group.c',ROOT/'firmware/User/Device/drive.c'])
        for case in range(12):
            with self.subTest(case=case):
                result=subprocess.run([str(program),str(case)],capture_output=True)
                self.assertEqual(result.returncode,0,result.stderr.decode(errors='replace'))

    def test_real_adapter_snapshot_is_readonly_and_frozen(self):
        program=ROOT/'output/tests/mech-failure-port.exe'
        port_fixture.MechPortTests.build(program,ROOT/'tests/mech_failure_port.c')
        result=subprocess.run([str(program)],capture_output=True)
        self.assertEqual(result.returncode,0,result.stderr.decode(errors='replace'))

    def test_new_return_keeps_requested_rate_and_group_without_editing_record(self):
        rows=[dict(order=1,axis='reach',target_m=0,rpm=100),dict(order=1,axis='arm',target_rad=math.pi,speed_rad_s=math.pi)]
        original=copy.deepcopy(rows)
        saved=record('fast_return',rows,1)
        link=seq_fixture.SequenceTests().ready();link.firmware='GC-BASE-0.3.39';link.actual['reach_um']=84000
        seq=Sequence();seq.start(saved,'ordered',link,1.01);seq.advance(link,1.02)
        self.assertEqual(len(seq.plan),1)
        _,wire,_=link.commands[0]
        self.assertEqual(struct.unpack_from('<I',wire,44)[0],100)
        self.assertEqual(rows,original)
        self.assertEqual(seq.saved,saved)

    def test_zero_single_rate_is_versioned_and_deadline_uses_actual_rate(self):
        for firmware,expected in (('GC-BASE-0.3.38',30),('GC-BASE-0.3.39',100)):
            _,wire=payload('move',dict(axis='reach',target_m=0,rpm=100,_manual_tuning=True,_firmware=firmware),dict(reach_um=84000),1)
            self.assertEqual(struct.unpack_from('<H',wire,20)[0],expected)

    def test_new_firmware_uses_two_rounds_without_changing_legacy(self):
        fixture=seq_fixture.SequenceTests();link=fixture.ready();link.firmware='GC-BASE-0.3.39';link.actual['stable_samples']=2
        seq=Sequence();saved=record('fast',fixture.rows(),1);seq.start(saved,'ordered',link,1.01)
        self.assertFalse(seq.advance(link,1.02));self.assertEqual(len(link.commands),1)
        self.assertEqual(seq.saved,saved)
        old=fixture.ready();old.actual['stable_samples']=2
        seq=Sequence();seq.start(saved,'ordered',old,1.01);seq.advance(old,1.02)
        self.assertFalse(old.commands)

    def test_snapshot_protocol_version_and_length(self):
        link=seq_fixture.SequenceTests().ready();link.firmware='GC-BASE-0.3.39'
        data=struct.pack('<7I8q2q2i2I',1,1,215418,104,2,1,3,1,-43905,2,-44131,1,249,2,249,1,83538,0,0,20,30)
        request=p.Packet(70,link.boot,link.session,1,250,b'')
        reply=p.Packet(198,link.boot,link.session,1,0,bytes([p.OK])+data)
        self.assertTrue(link.custom_reply(reply,request,1.1))
        self.assertEqual(link.failure['actual_raw'][1],-43905)
        self.assertEqual(link.failure['position_um'][1],83538)
        self.assertEqual(link.failure['reason'],104)
        bad=seq_fixture.SequenceTests().ready();bad.firmware='GC-BASE-0.3.39'
        bad.custom_reply(p.Packet(198,bad.boot,bad.session,1,0,bytes([p.OK])+data[:-1]),request,1.1)
        self.assertTrue(bad.latched)

    def test_snapshot_poll_keeps_critical_status_and_legacy_boundary(self):
        link=seq_fixture.SequenceTests().ready();link.firmware='GC-BASE-0.3.39'
        kinds=[link.extra_request(i*.04)[0] for i in range(30)]
        self.assertIn(70,kinds)
        for kind in (35,58,64,68):self.assertGreaterEqual(kinds.count(kind),5)
        old=seq_fixture.SequenceTests().ready();old.firmware='GC-BASE-0.3.38'
        self.assertNotIn(70,[old.extra_request(i*.04)[0] for i in range(30)])
