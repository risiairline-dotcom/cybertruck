"""正式传输的回调先后、接收静默与STOP优先级；不连接设备。"""
from pathlib import Path
import subprocess
import tempfile
import unittest

from test_execution import ROOT, compile_c


class CanTurnaroundTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.folder = tempfile.TemporaryDirectory(dir=ROOT / 'output/tests')
        cls.output = Path(cls.folder.name)
        header = (ROOT / 'tests/can_hal_stub.h').read_text(encoding='utf-8')
        header = header.replace('#define __disable_irq() ((void)0)',
                                'void Test_Disable(void);\n#define __disable_irq() Test_Disable()')
        cls.header = cls.output / 'callback_stub.h'
        cls.header.write_text(header, encoding='utf-8')
        cls.binary = cls.output / 'turnaround.exe'
        cls.build(cls.binary, ROOT / 'firmware/User/Device/can_tx.c')

    @classmethod
    def tearDownClass(cls):
        cls.folder.cleanup()

    @classmethod
    def build(cls, binary, source):
        compile_c(binary, [source, ROOT / 'tests/can_turnaround.c'],
                  extra=['-I', str(ROOT / 'tests'), '-include', str(cls.header)])

    def run_case(self, index):
        result = subprocess.run([str(self.binary), str(index)], capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr.decode('utf-8', errors='replace'))

    def test_tx_callback_crossing_millisecond_keeps_turnaround(self): self.run_case(0)
    def test_arbitration_callback_crossing_millisecond_keeps_backoff(self): self.run_case(1)
    def test_rx_callback_crossing_millisecond_keeps_quiet_time(self): self.run_case(2)
    def test_last_rx_extends_quiet_time(self): self.run_case(3)
    def test_stop_bypasses_rx_quiet_time(self): self.run_case(4)
    def test_rx_flood_does_not_extend_50ms_tx_deadline(self): self.run_case(5)
    def test_rx_quiet_time_crosses_uint32_wrap(self): self.run_case(6)
    def test_pending_fifo_blocks_non_stop_send(self): self.run_case(7)
    def test_stop_bypasses_pending_fifo(self): self.run_case(8)
    def test_pending_fifo_does_not_extend_50ms_deadline(self): self.run_case(9)

if __name__ == '__main__':
    unittest.main()
