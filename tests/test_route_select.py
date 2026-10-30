"""PC1选区、PC0锁定和两区50mm停靠的正式C闭环；无设备IO。"""

import copy
import ctypes as c
import json
import math
import struct
import tempfile
import unittest
from unittest.mock import patch

import test_execution as native
from robot.device.execution import START_PREPARE, START_STATE, ARM, GOAL, CONFIG, MotionLink, config_payload
from robot.device import protocol as p
from robot.drive_core import run_trial
from robot.nav.selection import routes, choice
from robot.nav.field import Field
from robot.nav.frames import Pose, compose
from robot.track_sim import Plant


class SwitchTransport(native.KeyTransport):
    def __init__(self, library, level=lambda now: 0, key=lambda now: now >= 2.):
        super().__init__(library)
        self.level, self.key = level, key
        self.lib.Device_SelectLevel.argtypes = [c.c_uint32, c.c_uint8]
        self.lib.Device_SwitchPrepare.argtypes = [c.c_uint32] * 4 + [c.c_uint8]
        self.lib.Device_SwitchRead.argtypes = [c.POINTER(c.c_uint32)]

    def selection(self):
        values = (c.c_uint32 * 27)()
        self.lib.Device_SwitchRead(values)
        return list(values)

    def write(self, wire):
        packet = p.Decoder().feed(wire)[0]
        if packet.kind in (START_PREPARE, START_STATE):
            self.requests.append(packet)
            accepted = True
            if packet.kind == START_PREPARE:
                accepted = self.lib.Device_SwitchPrepare(*struct.unpack('<4I', packet.payload), int(self.key(self.now)))
            data = bytes([p.OK if accepted else p.READONLY]) + struct.pack('<27I', *self.selection())
            self.replies.append(p.Packet(packet.kind | 0x80, packet.boot, packet.session, packet.request, 0, data).encode())
            return len(wire)
        if packet.kind in (ARM, GOAL):
            assert self.selection()[4] == 1 and not self.selection()[5]
        return native.SimTransport.write(self, wire)

    def sleep(self, seconds):
        native.SimTransport.sleep(self, seconds)
        if self.sweep:
            self.sweep(self.plant)
        self.lib.Device_SelectLevel(self.plant.ms, self.level(self.now))
        self.lib.Device_KeyTick(self.plant.ms, self.key(self.now))


class SwitchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        native.ExecutionTests.setUpClass.__func__(cls)

    def profile(self):
        return json.loads((native.ROOT/'config/switch_trials.json').read_text(encoding='utf-8'))

    def run_route(self, zone, level=None, key=None, sweep=True):
        plans, profile = routes(), self.profile()
        transport = SwitchTransport(self.lib, level or (lambda now: zone-1), key or (lambda now: now >= 2.))
        transport.plant = Plant(ideal=True, radius_m=profile['wheel_radius_m'],
                                k_m=profile['half_length_m']+profile['half_width_m'])
        transport.sensor_sign, transport.sensor_offset_rad = -1, .7
        field, origin = Field(data=plans[zone]['field']), Pose(*plans[zone]['map_origin'])
        checked = [-1]
        def check(plant):
            speed = max(abs(rpm) for sample in plant.history for rpm in sample[1]) * 2*math.pi*plant.radius_m/60
            margin = (math.sqrt(2)*speed+(field.radius+.04)*speed/plant.k_m)*.002
            for tick, x, y, yaw in plant.positions:
                if tick <= checked[0]:
                    continue
                self.assertEqual(field.errors(compose(origin, Pose(x,y,yaw)), f'start_{zone}', margin, tracking=False), [])
                checked[0] = tick
        transport.sweep = check if sweep else None
        records = []
        with tempfile.TemporaryDirectory() as directory:
            result = run_trial(transport, profile, 'straight', directory,
                lambda kind, **v: records.append(dict(kind=kind, **v)),
                clock=lambda: transport.now, sleep=transport.sleep, simulation=True,
                wait_start=True, switch_routes=plans)
        self.assertTrue(result['controller_complete'])
        self.assertEqual(len(result['completed_segments']), 9)
        self.assertEqual(sum(v.kind == ARM for v in transport.requests), 1)
        self.assertEqual(sum(v.kind == CONFIG for v in transport.requests), 1)
        goals = [v for v in transport.requests if v.kind == GOAL]
        self.assertEqual(len({(v.boot,v.session) for v in goals}),1)
        self.assertEqual([int.from_bytes(v.payload[:4],'little') for v in goals],list(range(1,10)))
        self.assertTrue(next(v for v in records if v['kind']=='stop_result')['confirmed'])
        self.assertTrue(field.parked(compose(origin,Pose(transport.plant.x,transport.plant.y,transport.plant.yaw)),f'start_{zone}'))
        self.assertEqual(self.lib.Device_Value(7),0)
        return records, transport

    def test_zone1_complete_with_50mm_docks_and_sweep(self):
        route = routes()[1]
        nodes = route['field']['nodes']
        body = route['field']['body']['left_m']
        self.assertAlmostEqual(nodes['raw'][0]-body-.085,.05)
        self.assertAlmostEqual(2.25-nodes['rough'][0]-body,.05)
        self.assertAlmostEqual(nodes['staging'][1]-body-.15,.05)
        self.run_route(1)

    def test_zone2_complete_and_runtime_switch_ignored(self):
        records, transport = self.run_route(2, level=lambda now: 1 if now < 5 else 0)
        selected = [v['zone'] for v in records if v['kind']=='route_selected']
        self.assertEqual(selected,[2])
        self.assertEqual(transport.selection()[25],2)

    def test_wait_switch_requires_release_then_new_press(self):
        records, _ = self.run_route(2, level=lambda now: int(now >= .9),
            key=lambda now: .85 <= now < 3. or now >= 4., sweep=False)
        self.assertEqual([v['zone'] for v in records if v['kind']=='route_selected'],[1,2])
        cancelled = [v for v in records if v['kind']=='physical_key_cancelled']
        self.assertEqual(cancelled[0]['event']['reason'],5)
        self.assertEqual(sum(v['kind']=='physical_key_started' for v in records),1)

    def test_no_key_times_out_without_motion(self):
        transport = SwitchTransport(self.lib,key=lambda now:False)
        with tempfile.TemporaryDirectory() as directory, self.assertRaisesRegex(ValueError,'取消'):
            run_trial(transport,self.profile(),'straight',directory,lambda *a,**k:None,
                clock=lambda:transport.now,sleep=transport.sleep,simulation=True,
                wait_start=True,switch_routes=routes())
        self.assertFalse(any(v.kind in (ARM,GOAL) for v in transport.requests))

    def test_old_firmware_refused_before_config(self):
        class OldLink(MotionLink):
            def feed(self, wire, now):
                super().feed(wire,now)
                if self.firmware:self.firmware='GC-BASE-0.3.19'
        transport = SwitchTransport(self.lib)
        with patch('robot.drive_core.MotionLink',OldLink),tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError,'PC1选区要求'):
                run_trial(transport,self.profile(),'straight',directory,lambda *a,**k:None,
                    clock=lambda:transport.now,sleep=transport.sleep,simulation=True,
                    wait_start=True,switch_routes=routes())
        self.assertFalse(any(v.kind in (CONFIG,ARM,GOAL) for v in transport.requests))

    def test_invalid_or_stale_selection_no_default(self):
        good=dict(schema=3,select_zone=1,select_epoch=1,select_valid=1)
        self.assertEqual(choice(good,.1),(1,1))
        for change in ({'schema':2},{'select_valid':0},{'select_zone':0},{'select_epoch':0}):
            self.assertIsNone(choice(dict(good,**change),.1))
        self.assertIsNone(choice(good,.25))

    def test_wrong_zone_goals_and_runtime_reselect_rejected(self):
        device = native.Device(self.lib)
        self.lib.Device_Select.argtypes=[c.c_uint32]
        device.open(); device.assert_ok(device.call(CONFIG,config_payload(self.profile())))
        for _ in range(8):device.step()
        self.assertNotEqual(device.call(ARM).payload[0],p.OK)
        self.assertEqual(self.lib.Device_Select(1),1)
        device.assert_ok(device.call(ARM))
        for _ in range(8):device.step()
        self.assertEqual(self.lib.Device_Select(2),0)
        self.assertNotEqual(device.call(GOAL,bytes.fromhex(routes()[2]['goals'][0])).payload[0],p.OK)
        device.assert_ok(device.call(GOAL,bytes.fromhex(routes()[1]['goals'][0])))

    def test_preview_never_opens_a_device(self):
        import contextlib
        import io
        from robot.drive_core import main
        output = io.StringIO()
        with patch('sys.argv',['drive_core','--switch-zone']),patch('robot.drive_core.open_transport') as opening:
            with contextlib.redirect_stdout(output):main()
        value = json.loads(output.getvalue())
        self.assertFalse(value['command_sent'])
        self.assertTrue(value['physical_key_required'])
        self.assertEqual(value['profile']['version'],9)
        self.assertEqual(set(value['switch_routes']),{'1','2'})
        opening.assert_not_called()

    def test_route_tampering_is_not_accepted_as_a_new_plan(self):
        from robot.nav.route import validate,digest
        value = copy.deepcopy(routes()[1])
        value['parameters']['docking']['raw_m'] = .06
        value['sha256'] = digest(value)
        with self.assertRaises(ValueError):validate(value)

    def test_switch_state_wire_schema_and_invalid_level(self):
        link=MotionLink();link.boot=123
        request=p.Packet(START_STATE,123,link.session,7,250)
        fields=[3,0,0,0,0,0]+[0]*15+[0,1,1,1,0,0]
        response=p.Packet(START_STATE|0x80,123,link.session,7,0,bytes([p.OK])+struct.pack('<27I',*fields))
        self.assertTrue(link.custom_reply(response,request,.1))
        self.assertFalse(link.failed)
        self.assertEqual(link.snapshot(.2)['physical_start']['select_zone'],1)
        self.assertIsNone(link.snapshot(.36)['physical_start'])
        fields[21]=2
        response=p.Packet(START_STATE|0x80,123,link.session,7,0,bytes([p.OK])+struct.pack('<27I',*fields))
        link.custom_reply(response,request,.4)
        self.assertTrue(link.failed)
