"""底盘协议演进与实车许可分离；不导入或打开设备传输。"""

from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pi'))
from robot.device.capabilities import LATEST_MINOR, firmware_capabilities
from robot.device.execution import MotionLink, STATUS, START_STATE, START_PREPARE, ARM, GOAL
from robot.device.can_capture import CAN_CAPTURE
from robot.device.feedback import FEEDBACK
from robot.device import protocol as p


class CapabilityTests(unittest.TestCase):
    def test_original_motion_entry_matrix_is_unchanged(self):
        # 原drive_core条件的独立真值表；新诊断能力不得扩大任一运动入口。
        for version in range(0, LATEST_MINOR + 2):
            caps = firmware_capabilities(f'GC-BASE-0.3.{version}')
            for simulation in (False, True):
                candidate = simulation and (version in (22, 23, 24, 35, 36, 37, 38, 39, 40, 41, 42) or 45 <= version <= LATEST_MINOR)
                offline = simulation and version in (12, 13, 14, 17, 18, 19, 20, 21)
                keyed = version in (17, 18, 19, 20, 21)
                old_routes = dict(short=version in (6, 7, 8, 9, 10, 11) or (version == 52 and not simulation),
                                  north=version in (7, 8, 9, 10, 11),
                                  raw=version in (8, 9, 10, 11),
                                  tour=version in (10, 11), batch=version == 11)
                expected = {route: bool(candidate or offline or keyed or accepted)
                            for route, accepted in old_routes.items()}
                expected.update(start2=bool(candidate or version in (19, 20, 21)),
                                switch=bool(candidate or version in (20, 21)))
                for route, accepted in expected.items():
                    with self.subTest(version=version, simulation=simulation, route=route):
                        self.assertEqual(caps.permits_motion(route, simulation=simulation), accepted)
                self.assertEqual(caps.permits_key(simulation=simulation), bool(candidate or keyed))

    def test_protocol_release_boundaries_are_bounded(self):
        for version in range(0, LATEST_MINOR + 2):
            caps = firmware_capabilities(f'GC-BASE-0.3.{version}')
            with self.subTest(version=version):
                self.assertEqual(caps.odom_fault, 9 <= version <= LATEST_MINOR)
                self.assertEqual(caps.can_history, 12 <= version <= LATEST_MINOR)
                self.assertEqual(caps.start_schema, 1 if version in (15, 16) else
                                 2 if version in (17, 18, 19) else 3 if 20 <= version <= LATEST_MINOR else 0)

    def test_unknown_or_malformed_versions_have_no_extension_or_motion(self):
        for value in (None, '', 42, {}, 'GC-BASE-0.3.043', 'GC-BASE-0.3.42-dev',
                      'GC-BASE-0.4.0', f'GC-BASE-0.3.{LATEST_MINOR + 1}', 'GC-BASE-0.3.42 ', 'gc-base-0.3.42'):
            with self.subTest(firmware=value):
                caps = firmware_capabilities(value)
                self.assertFalse(caps.odom_fault or caps.can_history or caps.start_schema)
                self.assertFalse(caps.physical_motion or caps.simulated_motion)
                self.assertFalse(caps.permits_key() or caps.permits_key(simulation=True))

    @staticmethod
    def link(version, *, mode=0, errors=1):
        link = MotionLink()
        link.firmware = f'GC-BASE-0.3.{version}'
        link.execution = dict(mode=mode, can_errors=errors)
        link.last_execution = link.last_fault_query = link.last_start_query = link.last_sample = 1.
        return link

    def test_known_versions_query_only_supported_readonly_extensions(self):
        for version in range(6, LATEST_MINOR + 1):
            with self.subTest(version=version):
                link = self.link(version)
                link.last_fault_query = -1.
                self.assertEqual(link.extra_request(1.), (37, b'') if 9 <= version <= LATEST_MINOR else
                                 (CAN_CAPTURE, b'\0') if 12 <= version <= LATEST_MINOR else None)
                link = self.link(version)
                link.last_start_query = -1.
                self.assertEqual(link.extra_request(1.), (START_STATE, b'') if 20 <= version <= LATEST_MINOR else
                                 (CAN_CAPTURE, b'\0') if 12 <= version <= LATEST_MINOR else None)
                link = self.link(version)
                self.assertEqual(link.extra_request(1.), (CAN_CAPTURE, b'\0') if 12 <= version <= LATEST_MINOR else None)

    def test_new_diagnostics_do_not_run_can_pages_during_motion(self):
        for version in (24, 25, 34, 41, 42, 43):
            for mode in range(7):
                with self.subTest(version=version, mode=mode):
                    link = self.link(version, mode=mode)
                    self.assertEqual(link.extra_request(1.), (CAN_CAPTURE, b'\0') if mode in (0, 1, 6) else None)
                    link = self.link(version, mode=mode)
                    link.last_start_query = -1.
                    expected = (START_STATE, b'') if mode in (0, 1) else (CAN_CAPTURE, b'\0') if mode == 6 else None
                    self.assertEqual(link.extra_request(1.), expected)
        self.assertIsNone(self.link(42, errors=0).extra_request(1.))

    def test_diagnostic_polling_preserves_status_and_imu_priority(self):
        link = self.link(42)
        link.last_execution = link.last_sample = -1.
        self.assertEqual(link.extra_request(1.), (STATUS, b''))
        self.assertEqual(link.extra_request(1.), (FEEDBACK, bytes((5, 0))))
        self.assertEqual(link.extra_request(1.), (CAN_CAPTURE, b'\0'))
        self.assertIsNone(link.extra_request(1.))
        self.assertFalse(link.command)

    def test_stop_still_preempts_readonly_and_pending_start(self):
        link = self.link(42)
        link.boot, link.bound = 1, True
        link.command = (START_PREPARE, b'pending')
        link.pending = (p.Packet(CAN_CAPTURE, 1, link.session, 1, 250, b'\0'), .9)
        link.stop()
        packet = p.Decoder().feed(link.step(1.))[0]
        self.assertEqual(packet.kind, p.STOP)
        self.assertIsNone(link.command)
        self.assertFalse(link.result)
        self.assertNotIn(packet.kind, (ARM, GOAL, START_PREPARE))

    def test_extensions_wait_for_hello_and_open(self):
        link = self.link(42)
        link.last_execution = link.last_fault_query = link.last_start_query = -1.
        self.assertEqual(p.Decoder().feed(link.step(1.))[0].kind, p.HELLO)
        link.pending = None
        link.boot, link.generation = 123, 1
        self.assertEqual(p.Decoder().feed(link.step(1.))[0].kind, p.OPEN)
        link.pending = None
        link.bound = True
        self.assertEqual(p.Decoder().feed(link.step(1.))[0].kind, p.HEART)


if __name__ == '__main__':
    unittest.main()
