"""联调段间残差回放，既验证误停修复也保留真实越界。"""
import subprocess,unittest,tempfile,re
from pathlib import Path
from test_execution import ROOT,compile_c

class EndpointTests(unittest.TestCase):
    def test_last_stop_survives_disconnect_without_authorizing(self):
        from robot.mech_core import DeviceCore
        from test_mech_sequence import SequenceTests
        core=DeviceCore(clock=lambda:1.02)
        core.link=SequenceTests().ready();core.link.actual['fault']=106
        core.capture_stop('机构106','program-failed')
        core.link.reset('USB断开')
        state=core.state()
        self.assertFalse(state['connected'])
        self.assertIsNone(state['mechanism'])
        self.assertEqual(state['stop_evidence'][-1]['mechanism']['fault'],106)
        self.assertTrue(state['stop_evidence'][-1]['historical'])
        for _ in range(20):core.capture_stop('停止','test')
        self.assertEqual(len(core.stop_evidence),12)

    def test_successive_endpoints(self):
        control=ROOT/'firmware/User/Control'
        program=ROOT/'output/tests/mech-endpoint.exe'
        compile_c(program,[ROOT/'tests/mech_endpoint.c',control/'mech_guard.c',control/'mech_axis.c',ROOT/'firmware/User/Device/drive.c'])
        for case in range(5):
            with self.subTest(case=case):
                result=subprocess.run([str(program),str(case)],capture_output=True)
                self.assertEqual(result.returncode,0,result.stderr.decode(errors='replace'))
