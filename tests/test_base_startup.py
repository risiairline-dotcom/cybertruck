"""真实板级启动循环的准备期、保护和旧逻辑反例；不连接设备。"""
from pathlib import Path
import subprocess
import tempfile
import unittest
from test_execution import ROOT, compile_c


class BaseStartupTests(unittest.TestCase):
    @classmethod
    def build(cls, output, fixture, mechanism=False):
        user = ROOT / 'firmware/User'
        sources = [fixture, *(user / 'Device' / name for name in ('can_tx.c', 'bus.c', 'drive.c', 'stamp.c'))]
        sources += [user / 'Control' / name for name in ('exec.c', 'track.c', 'ref.c', 'odom.c', 'kin.c',
                    'yaw.c', 'curve.c', 'line.c', 'path.c', 'start.c')]
        sources += [user / 'Comm' / name for name in ('frame.c', 'comm.c', 'execute.c', 'feedback.c')]
        if mechanism:
            sources += [user / 'Board/mech_port.c']
            sources += [user / 'Control' / name for name in
                        ('mech_guard.c', 'mech_axis.c', 'mech_servo.c', 'mech_pose.c', 'mech_group.c')]
        compile_c(output, sources, extra=['-I', str(ROOT / 'tests'), '-I', str(user / 'Board'),
                  '-I', str(ROOT / 'tests/mech_hal'),
                  '-I', str(ROOT / 'firmware/Core/Inc'), '-include', str(ROOT / 'tests/base_hal_stub.h')])

    @classmethod
    def setUpClass(cls):
        cls.binary = ROOT / 'output/tests/base-startup.exe'
        cls.build(cls.binary, ROOT / 'tests/base_startup.c')

    def run_case(self, case):
        result = subprocess.run([str(self.binary), str(case)], capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr.decode('utf-8', errors='replace'))

    def test_driver_delayed_ready_does_not_create_startup_fault(self): self.run_case(0)
    def test_missing_reply_still_latches_and_recovery_does_not_clear(self): self.run_case(1)
    def test_tick_wrap(self): self.run_case(2)
    def test_held_pc0_does_not_arm_or_beep(self): self.run_case(3)
    def test_missing_reply_after_startup_still_uses_100ms_deadline(self): self.run_case(4)
    def test_real_can_error_during_preparation_is_not_masked(self): self.run_case(5)

if __name__ == '__main__':
    unittest.main()
