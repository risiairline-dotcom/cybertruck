"""六节点正式调度、延迟回包及ID5故障保持；不访问设备。"""
import subprocess
import tempfile
import unittest
from pathlib import Path
from test_execution import ROOT
import test_base_startup


class MechQueryScheduleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.binary = ROOT / 'output/tests/mech-query-schedule.exe'
        test_base_startup.BaseStartupTests.build(cls.binary, ROOT / 'tests/mech_query_schedule.c', mechanism=True)

    def test_six_node_schedule_and_faults(self):
        for case in range(9):
            with self.subTest(case=case):
                result = subprocess.run([str(self.binary), str(case)], capture_output=True)
                self.assertEqual(result.returncode, 0, result.stderr.decode(errors='replace'))
                print(result.stdout.decode().strip())

    def test_stop_at_every_query_phase_is_recoverable(self):
        for case in range(9,49):
            with self.subTest(case=case):
                result=subprocess.run([str(self.binary),str(case)],capture_output=True)
                self.assertEqual(result.returncode,0,result.stderr.decode(errors='replace'))

    def test_real_error_during_stop_still_latches(self):
        result=subprocess.run([str(self.binary),'49'],capture_output=True)
        self.assertEqual(result.returncode,0,result.stderr.decode(errors='replace'))

if __name__ == '__main__':
    unittest.main()
