"""实际TXOK与回调先后的反例；不连接设备。"""
import subprocess
import tempfile
from pathlib import Path
import unittest
from test_execution import ROOT, compile_c


class CanReplyOrderTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.binary = ROOT / 'output/tests/can-reply-order.exe'
        cls.device = ROOT / 'firmware/User/Device'
        cls.build(cls.binary, cls.device / 'bus.c')

    @classmethod
    def build(cls, binary, bus):
        compile_c(binary, [ROOT / 'tests/can_reply_order.c', cls.device / 'can_tx.c',
                           bus, cls.device / 'drive.c'],
                  extra=['-I', str(ROOT / 'tests'), '-include', str(ROOT / 'tests/can_hal_stub.h')])

    def run_case(self, case):
        result = subprocess.run([str(self.binary), str(case)], capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr.decode('utf-8', errors='replace'))

    def test_hardware_txok_reply_before_tx_callback(self): self.run_case(0)
    def test_no_hardware_txok_is_rejected(self): self.run_case(1)
    def test_hardware_terr_cannot_unlock_reply(self): self.run_case(2)
    def test_millisecond_wrap(self): self.run_case(3)
    def test_queued_old_frame_not_retagged_by_later_callback(self): self.run_case(4)
    def test_wrong_identity_still_rejected_with_hardware_txok(self): self.run_case(5)
    def test_real_error_during_receive_cannot_complete_current_query(self): self.run_case(6)
    def test_ring_overflow_during_receive_cannot_complete_current_query(self): self.run_case(7)

    def test_error_then_late_complete_never_publishes_success(self): self.run_case(12)
    def test_simultaneous_can_and_rx_errors_preserve_both_causes(self): self.run_case(13)

    def test_id6_target_query_reply_order_and_real_error_lock(self):
        for case in range(8, 12):
            with self.subTest(case=case): self.run_case(case)

if __name__ == '__main__':
    unittest.main()
