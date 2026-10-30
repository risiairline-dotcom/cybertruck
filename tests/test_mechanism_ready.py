"""实车准备模块的软件证据；不连接设备、不调用串口或HAL。"""

import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pi'))
from robot.device.integration import preparation


class MechanismReadyTests(unittest.TestCase):
    def test_passive_native_guards_and_reused_protocol(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'output/tests') as folder:
            executable = Path(folder)/'mech_guard.exe'
            subprocess.run([sys.executable,'-m','ziglang','cc','-std=c99','-O2','-Wall','-Wextra','-Werror',
                            '-I',str(ROOT/'firmware/User/Device'),'-I',str(ROOT/'firmware/User/Config'),
                            '-I',str(ROOT/'firmware/User/Control'),str(ROOT/'tests/mech_guard.c'),
                            str(ROOT/'firmware/User/Control/mech_guard.c'),str(ROOT/'firmware/User/Device/drive.c'),
                            '-o',str(executable)],check=True,capture_output=True)
            result = subprocess.run([str(executable)],capture_output=True,text=True,encoding='utf-8',errors='replace')
            self.assertEqual(result.returncode,0,result.stdout + result.stderr)
            self.assertIn('13 passive mechanism groups: PASS',result.stdout)
            (ROOT/'output/tests/mechanism_ready.log').write_text(result.stdout,encoding='utf-8')

    def test_mapping_matches_ioc_and_user_slot_definition(self):
        view=preparation()
        self.assertEqual([a['address'] for a in view['axes']],[5,6])
        self.assertEqual([a['feedback_sign'] for a in view['axes']],[1,-1])
        self.assertEqual([(s['pin'],s['channel']) for s in view['servos']],[('PE9',1),('PE11',2),('PE13',3)])
        self.assertEqual(view['tray_reference_deg'],0)
        self.assertEqual(view['tray_slot_deg'],[30,150,270])
        self.assertFalse(view['real_connected'])
        self.assertFalse(view['real_motion'])
        self.assertFalse(view['physical_revalidated'])

if __name__ == '__main__':
    unittest.main()
