import subprocess,tempfile,unittest
from pathlib import Path
from test_base_startup import BaseStartupTests,ROOT

class MicroAckTests(unittest.TestCase):
    def test_real_bus_replacement(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'output/tests') as folder:
            binary=Path(folder)/'micro-ack.exe'
            BaseStartupTests.build(binary,ROOT/'tests/micro_ack.c')
            result=subprocess.run([str(binary)],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)

    def test_real_base_position_step(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'output/tests') as folder:
            binary=Path(folder)/'micro-position.exe'
            BaseStartupTests.build(binary,ROOT/'tests/micro_position.c')
            result=subprocess.run([str(binary)],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertIn('PASS',result.stdout)
