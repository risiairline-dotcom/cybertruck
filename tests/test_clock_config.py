"""由 IOC 时钟树独立重算 CAN、PWM 与 SysTick 频率。"""
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import check_project as project


class ClockTests(unittest.TestCase):
    def test_current_project_passes_static_check(self):
        project.ERRORS.clear()
        data = project.load_ioc()
        project.verify_ioc(data)
        project.verify_sources()
        project.verify_owners()
        project.verify_keil()
        self.assertEqual(project.ERRORS, [])

    def test_derived_rates_preserve_can_pwm_and_systick(self):
        d = project.load_ioc()
        vco = int(d['RCC.HSE_VALUE']) / int(d['RCC.PLLM']) * int(d['RCC.PLLN'])
        sysclk, apb1, apb2 = vco / 2, vco / 8, vco / 4
        self.assertEqual((sysclk, apb1, apb2, vco / int(d['RCC.PLLQ'])),
                         (120000000, 30000000, 60000000, 48000000))
        self.assertEqual(apb1 / int(d['CAN1.Prescaler']) / (1 + 12 + 2), 500000)
        self.assertEqual(apb2 * 2 / (int(d['TIM1.Prescaler']) + 1) /
                         (int(d['TIM1.Period']) + 1), 50)


if __name__ == '__main__':
    unittest.main(verbosity=2)
