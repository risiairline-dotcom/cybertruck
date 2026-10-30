"""正式GC与C闭环的同会话多段验证；不打开硬件。"""

import json
from pathlib import Path
import tempfile
import unittest
import time
from dataclasses import replace
from types import SimpleNamespace

import test_execution as fixture
from robot.drive_core import run_trial
from robot.nav.route import validate, digest, export_navigation
from robot.device import protocol as p
from robot.device.execution import CONFIG, ARM, GOAL, STATUS, trial
from robot.device.tracking import Spec
from robot.web.app import create_app
from robot.runtime import save_snapshot
from fastapi.testclient import TestClient

ROOT = Path(__file__).resolve().parents[1]


class RouteLinkTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fixture.ExecutionTests.setUpClass()
        cls.lib = fixture.ExecutionTests.lib

    def plan(self):
        return json.loads((ROOT / 'config/short_route.json').read_text(encoding='utf-8'))

    def run_route(self, transport):
        records = []
        with tempfile.TemporaryDirectory() as directory:
            result = run_trial(transport, fixture.profile(), 'straight', directory,
                               lambda kind, **v: records.append(dict(kind=kind, **v)),
                               clock=lambda: transport.now, sleep=transport.sleep,
                               route=self.plan(), simulation=True)
            snapshot = json.loads((Path(directory) / 'state.json').read_text(encoding='utf-8'))
        return result, records, snapshot

    def test_one_session_one_configuration_continuous_odometry(self):
        transport = fixture.SimTransport(self.lib)
        result, records, snapshot = self.run_route(transport)
        self.assertTrue(result['controller_complete'])
        for kind in (p.OPEN, CONFIG, ARM, p.STOP):
            self.assertEqual(sum(v.kind == kind for v in transport.requests), 1)
        goals = [v for v in transport.requests if v.kind == GOAL]
        self.assertEqual([int.from_bytes(v.payload[:4], 'little') for v in goals], [1, 2, 3, 4])
        self.assertEqual(len({(v.boot, v.session) for v in goals}), 1)
        ticks = [v['execution']['board_ms'] for v in records if v['kind'] == 'state' and v.get('execution')]
        self.assertTrue(all(a <= b for a, b in zip(ticks, ticks[1:])))
        route = snapshot['route_execution']
        self.assertEqual(route['completed'], 4)
        self.assertEqual(route['status'], 'complete')
        self.assertGreater(transport.plant.x, .28)
        self.assertGreater(transport.plant.y, .12)
        self.assertTrue(snapshot['execution']['stop_confirmed'])
        self.assertFalse(route['map_registered'])

    def test_lost_second_goal_ack_stops_without_third_goal(self):
        class LostAck(fixture.SimTransport):
            def read(self, count=4096):
                data = super().read(count)
                if sum(v.kind == GOAL for v in self.requests) == 2:
                    if any(v.kind == GOAL | 0x80 for v in p.Decoder().feed(data)):
                        return b''
                return data
        transport = LostAck(self.lib)
        with self.assertRaises(ValueError):
            self.run_route(transport)
        self.assertEqual(sum(v.kind == GOAL for v in transport.requests), 2)
        self.assertEqual(sum(v.kind == ARM for v in transport.requests), 1)

    def test_asymmetric_wheel_gain_and_lag_complete_same_route(self):
        transport = fixture.SimTransport(self.lib)
        transport.plant.gains = [.96, 1.04, .98, 1.02]
        transport.plant.tau = [.07, .09, .11, .08]
        result, _, state = self.run_route(transport)
        self.assertTrue(result['controller_complete'])
        self.assertTrue(state['execution']['stop_confirmed'])
        self.assertEqual(sum(v.kind == GOAL for v in transport.requests), 4)

    def test_reboot_during_second_leg_never_resumes(self):
        class Reboot(fixture.SimTransport):
            def read(self, count=4096):
                data = super().read(count)
                packets = p.Decoder().feed(data)
                if sum(v.kind == GOAL for v in self.requests) >= 2:
                    return b''.join(replace(v, boot=124).encode() if v.kind == STATUS | 0x80
                                    else v.encode() for v in packets)
                return data
        transport = Reboot(self.lib)
        with self.assertRaises((ValueError, RuntimeError)):
            self.run_route(transport)
        self.assertEqual(sum(v.kind == GOAL for v in transport.requests), 2)
        self.assertEqual(sum(v.kind == ARM for v in transport.requests), 1)

    def test_field_plan_or_modified_targets_rejected_before_any_write(self):
        for change in ('frame', 'payload', 'hash'):
            plan = self.plan()
            if change == 'frame':
                plan.update(frame_id='map', domain='simulation')
                plan['sha256'] = digest(plan)
            elif change == 'payload':
                plan['goals'][0] = '00' * 96
                plan['sha256'] = digest(plan)
            else:
                plan['sha256'] = 'wrong'
            transport = fixture.SimTransport(self.lib)
            with tempfile.TemporaryDirectory() as directory, self.assertRaises(ValueError):
                run_trial(transport, fixture.profile(), 'straight', directory,
                          lambda *a, **k: None, route=plan)
            self.assertEqual(transport.requests, [])

    def test_web_preview_has_no_motion_endpoint(self):
        with tempfile.TemporaryDirectory() as directory, TestClient(create_app(directory)) as client:
            route = client.get('/api/routes/short').json()
            validate(route)
            self.assertFalse(route['executable'])
            self.assertEqual(client.post('/api/routes/short', json={}).status_code, 405)
            self.assertEqual(client.post('/api/routes/run', json={}).status_code, 404)
            replay = client.get('/api/routes/short/replay').json()
            if replay['available']:
                self.assertEqual(replay['domain'], 'simulation_replay')
                self.assertEqual(replay['plan_sha256'], route['sha256'])

    def test_field_export_reuses_original_spec_but_stays_blocked(self):
        spec = Spec.from_buffer_copy(trial('pose')[8:])
        model = SimpleNamespace(scope='two_batch', actions=[(0, SimpleNamespace(
            reference=SimpleNamespace(spec=spec)))])
        package = export_navigation(model)
        self.assertEqual(bytes.fromhex(package['goals'][0]), bytes(spec))
        self.assertEqual(package['goal_encoding'], 'RefSpec88')
        self.assertFalse(package['executable'])
        with self.assertRaises(ValueError):
            validate(package)

    def test_web_never_exposes_stale_route_pose_as_current(self):
        with tempfile.TemporaryDirectory() as directory, TestClient(create_app(directory)) as client:
            state = dict(mode='serial', readonly=True, connected=False,
                         snapshot_at_s=time.monotonic(), snapshot_wall_s=time.time(),
                         route_execution=dict(domain='commissioning', pose=[1, 2, 0], status='running'))
            save_snapshot(Path(directory) / 'state.json', state)
            view = client.get('/api/state').json()['route_execution']
            self.assertIsNone(view['pose'])
            self.assertEqual(view['status'], 'stale')


if __name__ == '__main__':
    unittest.main()
