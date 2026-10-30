"""正式板级机构适配器：合成CAN/PWM验证启动、去重和故障链。"""
from pathlib import Path
import subprocess
import tempfile
import unittest
from test_execution import ROOT, compile_c

class MechPortTests(unittest.TestCase):
    @staticmethod
    def build(program, fixture):
        sources=[fixture,ROOT/'firmware/User/Comm/frame.c',ROOT/'firmware/User/Device/drive.c']
        sources += [ROOT/'firmware/User/Control'/name for name in ('mech_guard.c','mech_axis.c','mech_servo.c','mech_pose.c','mech_group.c')]
        compile_c(program,sources,extra=('-I',str(ROOT/'firmware/User/Board'),'-I',str(ROOT/'tests/mech_hal')))

    def test_native_board_adapter(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'output/tests') as folder:
            program=Path(folder)/'mech-port.exe'
            self.build(program, ROOT/'tests/mech_port.c')
            for case in range(72):
                with self.subTest(case=case):
                    result=subprocess.run([str(program),str(case)],capture_output=True)
                    self.assertEqual(result.returncode,0,result.stderr.decode(errors='replace'))

