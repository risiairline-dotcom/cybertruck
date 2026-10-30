"""正式CAN、总线与解析器联合验证；只运行主机可执行文件。"""
import subprocess
import tempfile
from pathlib import Path
import unittest
from test_execution import ROOT, compile_c


class CanQueryCaptureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.binary = ROOT / 'output/tests/can-query-capture.exe'
        device = ROOT / 'firmware/User/Device'
        compile_c(cls.binary, [ROOT / 'tests/can_query_capture.c', device / 'can_tx.c',
                               device / 'bus.c', device / 'drive.c'],
                  extra=['-I', str(ROOT / 'tests'), '-include', str(ROOT / 'tests/can_hal_stub.h')])

    def run_case(self, index):
        result = subprocess.run([str(self.binary), str(index)], capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr.decode('utf-8', errors='replace'))

    def test_first_fault_and_wrapped_history_are_immutable(self): self.run_case(0)
    def test_txok_waits_for_reply_and_turnaround(self): self.run_case(1)
    def test_wrong_address_code_and_bad_checksum_do_not_unlock(self): self.run_case(2)
    def test_missing_reply_latches_without_retry(self): self.run_case(3)
    def test_stop_preempts_reply_wait(self): self.run_case(4)
    def test_pre_txok_reply_is_not_reused(self): self.run_case(5)
    def test_millisecond_wrap(self): self.run_case(6)
    def test_deadline_reply_is_rejected(self): self.run_case(7)
    def test_real_can_error_still_fails_without_replay(self): self.run_case(8)
    def test_on_time_reply_can_finish_turnaround_after_deadline(self): self.run_case(9)
    def test_delayed_replies_keep_both_motion_and_wheel_feedback_live(self): self.run_case(10)
    def test_missing_active_reply_retains_four_stops(self): self.run_case(11)
    def test_first_query_timeout_is_frozen_independently(self): self.run_case(13)
    def test_timeout_record_preserves_millisecond_wrap(self): self.run_case(14)
    def test_timeout_record_includes_last_unmatched_reply(self): self.run_case(15)

if __name__ == '__main__': unittest.main()
