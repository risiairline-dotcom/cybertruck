"""正式GC/执行器/跟踪器的主机闭环；不打开真实串口。"""

import ctypes as c
import json
import math
import os
import struct
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import time
from fastapi.testclient import TestClient

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pi'))
# 主机夹具与编译产物统一放在已忽略的 output/tests。
(ROOT / 'output/tests').mkdir(parents=True, exist_ok=True)
from robot.device import protocol as p
from robot.device.execution import MotionLink, CONFIG, ARM, GOAL, STATUS, STATUS_DATA, config_payload, trial
from robot.device.execution import START_PREPARE, START_STATE
from robot.drive_core import run_trial, require_owner_stopped, observe
from robot.track_sim import Plant
from robot.web.app import create_app
from robot.runtime import save_snapshot, load_snapshot
from robot.device.feedback import Feedback, FEEDBACK
from robot.device.tracking import Reference, Goal, Spec


def profile():
    value = json.loads((ROOT / 'config/base_trials.json').read_text(encoding='utf-8'))
    # 合成车体的已知参数；不能回写实车候选或声称本车方向已确认。
    value.update(yaw_sign=1, half_length_m=.10, half_width_m=.12)
    return value


def compile_c(output, sources, extra=()):
    sources = list(sources)
    # 舵机复用正式底盘曲线，独立夹具也链接同一实现。
    curve = ROOT / 'firmware/User/Control/curve.c'
    if any(Path(p).name == 'mech_servo.c' for p in sources) and curve not in sources:
        sources.append(curve)
    command = [sys.executable, '-m', 'ziglang', 'cc', '-std=c99', '-O2',
               '-Wall', '-Wextra', '-Werror', '-UNDEBUG', '-Wno-misleading-indentation']
    for name in ('Control', 'Device', 'Comm', 'Config'):
        command += ['-I', str(ROOT / 'firmware/User' / name)]
    command += [*extra, *map(str, sources), '-o', str(output)]
    result = subprocess.run(command, capture_output=True, text=True, encoding='utf-8', errors='replace')
    if result.returncode:
        raise RuntimeError(result.stderr)


class Device:
    def __init__(self, library):
        self.lib = library
        self.lib.Device_Init()
        self.tick = self.request = 0
        self.lib.Device_Tick(0, (c.c_int64 * 4)(), (c.c_int16 * 4)(), 0, 0)

    def write(self, wire):
        self.lib.Device_Push(wire, len(wire))
        return len(wire)

    def read(self, _=4096):
        output = c.create_string_buffer(160)
        size = self.lib.Device_Read(output)
        return output.raw[:size]

    def call(self, kind, payload=b'', session=456):
        self.request += 1
        self.write(p.Packet(kind, 123, session, self.request, 250, payload).encode())
        return p.Decoder().feed(self.read())[0]

    def open(self):
        hello = self.call(p.HELLO, session=0)
        self.call(p.OPEN, hello.payload[1:5])

    def step(self, counts=None, rpm=None, yaw=0, gyro=0, heart=True):
        self.tick += 10
        if heart and self.tick % 100 == 0:
            self.call(p.HEART, self.tick.to_bytes(4, 'little'))
        self.lib.Device_Tick(self.tick, (c.c_int64 * 4)(*(counts or [0] * 4)),
                             (c.c_int16 * 4)(*(rpm or [0] * 4)), yaw, gyro)

    def arm(self):
        self.open()
        self.call(CONFIG, config_payload(profile()))
        for _ in range(8):
            self.step()
        self.assert_ok(self.call(ARM))
        for _ in range(8):
            self.step()

    @staticmethod
    def assert_ok(packet):
        if packet.payload[0] != p.OK:
            raise AssertionError(packet.payload)


class SimTransport(Device):
    def __init__(self, library):
        super().__init__(library)
        self.plant = Plant(ideal=True)
        self.now = 0.
        self.requests = []
        self.signs = [1, -1, 1, -1]
        self.sensor_sign = 1
        self.sensor_offset_rad = 0.
        self.previous_raw_yaw = None
        self.crossed_yaw_wrap = False

    def write(self, wire):
        self.requests += p.Decoder().feed(wire)
        return super().write(wire)

    def sleep(self, seconds):
        steps = max(1, math.ceil(seconds / .002))
        for _ in range(steps):
            target = (c.c_int16 * 4)()
            self.lib.Device_Target(target)
            self.plant.targets = [v * s for v, s in zip(target, self.signs)]
            self.plant.integrate()
            tick = self.plant.ms
            if tick % 10 == 0:
                counts = (c.c_int64 * 4)(*[round(v * 65536) * s for v, s in zip(self.plant.turns, self.signs)])
                rpm = (c.c_int16 * 4)(*[round(v) * s for v, s in zip(self.plant.rpm, self.signs)])
                yaw = round((self.sensor_sign * self.plant.yaw + self.sensor_offset_rad) / profile()['angle_scale_rad'])
                yaw = ((yaw + 32768) & 0xffff) - 32768
                if self.previous_raw_yaw is not None and abs(yaw - self.previous_raw_yaw) > 32768:
                    self.crossed_yaw_wrap = True
                self.previous_raw_yaw = yaw
                gyro = round(self.sensor_sign * self.plant.wz / profile()['gyro_scale_rad_s'])
                self.lib.Device_Tick(tick, counts, rpm, yaw, gyro)
            self.now = tick / 1000


class KeyTransport(SimTransport):
    """复用正式C按键和闭环；仅按键电平、传输和车辆由模型提供。"""

    def __init__(self, library, press_at=2., drop_gyro_at=None):
        super().__init__(library)
        self.press_at = press_at
        self.drop_gyro_at = drop_gyro_at
        self.replies = []
        self.sweep = None
        self.lib.Device_KeyPrepare.argtypes = [c.c_uint32, c.c_uint32]
        self.lib.Device_KeyTick.argtypes = [c.c_uint32, c.c_uint8]
        self.lib.Device_SwitchRead.argtypes = [c.POINTER(c.c_uint32)]

    def key_state(self):
        values = (c.c_uint32 * 27)()
        self.lib.Device_SwitchRead(values)
        return list(values)

    def write(self, wire):
        packet = p.Decoder().feed(wire)[0]
        if packet.kind in (START_PREPARE, START_STATE):
            self.requests.append(packet)
            accepted = True
            if packet.kind == START_PREPARE:
                ticket, version = struct.unpack('<II', packet.payload)
                accepted = bool(self.lib.Device_KeyPrepare(ticket, version))
            data = bytes([p.OK if accepted else p.READONLY]) + struct.pack('<27I', *self.key_state())
            self.replies.append(p.Packet(packet.kind | 0x80, packet.boot, packet.session,
                                         packet.request, 0, data).encode())
            return len(wire)
        if packet.kind in (ARM, GOAL):
            assert self.key_state()[4] and not self.key_state()[5], '按键短鸣结束前禁止使能或目标'
        return super().write(wire)

    def read(self, size=4096):
        return self.replies.pop(0) if self.replies else super().read(size)

    def sleep(self, seconds):
        super().sleep(seconds)
        if self.sweep:
            self.sweep(self.plant)
        if self.drop_gyro_at is not None and self.now >= self.drop_gyro_at:
            self.lib.Device_Drop(8)
        # 按住电平可跨越路线时长；正式Start只能消费一次事件。
        pressed = self.press_at is not None and self.now >= self.press_at
        self.lib.Device_KeyTick(self.plant.ms, int(pressed))


class ExecutionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        output = ROOT / 'output/tests'
        output.mkdir(exist_ok=True, parents=True)
        control = ROOT / 'firmware/User/Control'
        comm = ROOT / 'firmware/User/Comm'
        # 同进程聚合运行时可能再次构建；Windows不能覆盖已被ctypes加载的DLL。
        library = output / f'execution-{os.getpid()}-{time.time_ns()}.dll'
        compile_c(library, [*(control / name for name in ('exec.c', 'track.c', 'ref.c', 'odom.c',
                    'kin.c', 'yaw.c', 'curve.c', 'line.c', 'path.c', 'start.c')),
                    ROOT / 'firmware/User/Device/stamp.c', comm / 'frame.c', comm / 'comm.c',
                    comm / 'execute.c', comm / 'feedback.c', ROOT / 'tests/execution_native.c'], extra=['-shared'])
        cls.lib = c.CDLL(str(library))
        cls.lib.Device_Push.argtypes = [c.c_char_p, c.c_uint]
        cls.lib.Device_Read.argtypes = [c.c_void_p]
        cls.lib.Device_Read.restype = c.c_uint16
        cls.lib.Device_Tick.argtypes = [c.c_uint32, c.POINTER(c.c_int64), c.POINTER(c.c_int16), c.c_int16, c.c_int16]
        cls.lib.Device_Target.argtypes = [c.POINTER(c.c_int16)]
        cls.lib.Device_Value.argtypes = [c.c_uint]
        cls.lib.Device_Value.restype = c.c_uint32
        cls.lib.Device_Ref.argtypes = [c.c_double, c.POINTER(Reference)]

    def setUp(self):
        self.device = Device(self.lib)

    def test_boot_and_configuration_never_enable(self):
        self.device.open()
        self.device.assert_ok(self.device.call(CONFIG, config_payload(profile())))
        for _ in range(20):
            self.device.step()
        self.assertEqual(self.lib.Device_Value(4), 0)
        self.assertEqual(self.lib.Device_Value(1), 0)
        self.assertEqual(self.lib.Device_Value(3), 0)

    def test_goal_requires_separate_arm(self):
        self.device.open()
        self.device.call(CONFIG, config_payload(profile()))
        self.assertNotEqual(self.device.call(GOAL, trial('straight')).payload[0], p.OK)
        self.assertEqual(self.lib.Device_Value(1), 0)

    def test_key_preparation_reads_fresh_feedback_before_any_arm(self):
        self.device.open()
        self.device.call(CONFIG, config_payload(profile()))
        for _ in range(8):
            self.device.step()
        self.assertEqual(self.lib.Device_ObservationValid(), 0)
        self.assertEqual(self.lib.Device_StartReady(self.device.tick), 1)
        self.assertEqual(self.lib.Device_ObservationValid(), 1)
        self.assertEqual(self.lib.Device_Value(0), 1)
        self.assertEqual(self.lib.Device_Value(4), 0)
        self.assertEqual(self.lib.Device_StartReady(self.device.tick + 251), 0)

    def test_physical_start_event_precedes_arm_and_goal(self):
        import struct
        from robot.device.execution import START_PREPARE, START_STATE

        class KeyTransport(SimTransport):
            def __init__(self, library):
                super().__init__(library)
                self.key_replies = []
                self.key_polls = 0
                self.key_ticket = self.key_version = 0

            def write(self, wire):
                packet = p.Decoder().feed(wire)[0]
                if packet.kind in (START_PREPARE, START_STATE):
                    self.requests.append(packet)
                    if packet.kind == START_PREPARE:
                        self.key_ticket, self.key_version = struct.unpack('<II', packet.payload)
                    else:
                        self.key_polls += 1
                    phase = 4 if self.key_polls >= 3 else 1
                    data = bytes([p.OK]) + struct.pack('<6I', 1, phase, self.key_ticket,
                                                       self.key_version, int(phase == 4), 0)
                    self.key_replies.append(p.Packet(packet.kind | 0x80, packet.boot, packet.session,
                                                     packet.request, 0, data).encode())
                    return len(wire)
                if packet.kind in (ARM, GOAL):
                    assert self.key_polls >= 3, '按键完成前不得使能或提交目标'
                return super().write(wire)

            def read(self, size=4096):
                return self.key_replies.pop(0) if self.key_replies else super().read(size)

        transport = KeyTransport(self.lib)
        with tempfile.TemporaryDirectory() as directory:
            result = run_trial(transport, profile(), 'straight', directory, lambda *a, **k: None,
                               clock=lambda: transport.now, sleep=transport.sleep,
                               simulation=True, wait_start=True)
        self.assertTrue(result['controller_complete'])
        self.assertGreaterEqual(transport.key_polls, 3)

    def test_start_cancel_record_schema_and_identity(self):
        import struct
        from robot.device.execution import START_STATE
        link = MotionLink()
        link.boot = 123
        request = p.Packet(START_STATE, 123, link.session, 7, 250)
        fields = [2, 5, 11, 6, 0, 0, 1, 3020, 516, 3020,
                  2980, 3010, 2769, 2990, 2991, 2992, 2993, 3000, 3001, 3002, 3003]
        reply = p.Packet(START_STATE | 0x80, 123, link.session, 7, 0,
                         bytes([p.OK]) + struct.pack('<21I', *fields))
        self.assertTrue(link.custom_reply(reply, request, 1.))
        self.assertEqual(link.start_event['reason'], 1)
        self.assertEqual(link.start_event['guard_mask'], 516)
        self.assertEqual(link.start_event['gyro_ms'], 2769)
        malformed = p.Packet(START_STATE | 0x80, 123, link.session, 7, 0,
                             bytes([p.OK]) + struct.pack('<6I', 2, 5, 11, 6, 0, 0))
        link.custom_reply(malformed, request, 2.)
        self.assertTrue(link.failed)

    def test_key_gate_keeps_250ms_gyro_freshness_without_arming(self):
        self.device.open()
        self.device.call(CONFIG, config_payload(profile()))
        for _ in range(8):
            self.device.step()
        self.assertEqual(self.lib.Device_StartReady(self.device.tick), 1)
        self.lib.Device_Drop(8)
        for _ in range(24):
            self.device.step()
        self.assertEqual(self.lib.Device_StartReady(self.device.tick), 1)
        self.device.step()
        self.assertEqual(self.lib.Device_StartReady(self.device.tick), 0)
        self.assertEqual(self.lib.Device_StartMask() & 512, 512)
        self.assertEqual(self.lib.Device_Value(4), 0)
        self.assertEqual(self.lib.Device_Value(3), 0)

    def test_one_meter_calibration_profiles_are_isolated_and_close_loop(self):
        value = json.loads((ROOT / 'config/calibration_trials.json').read_text(encoding='utf-8'))
        for name in ('cal_slow', 'cal_medium', 'cal_fast'):
            transport = SimTransport(self.lib)
            with tempfile.TemporaryDirectory() as directory:
                result = run_trial(transport, value, name, directory, lambda *a, **k: None,
                                   clock=lambda: transport.now, sleep=transport.sleep, simulation=True)
            self.assertTrue(result['controller_complete'], name)
            self.assertAlmostEqual(transport.plant.x, 1., delta=.01)
            self.assertEqual(sum(packet.kind == GOAL for packet in transport.requests), 1)
        with self.assertRaises(ValueError):
            run_trial(SimTransport(self.lib), profile(), 'cal_fast', '.', lambda *a, **k: None,
                      simulation=True)

    def test_config_binds_current_imu_epoch_and_later_change_stops(self):
        self.lib.Device_Epoch(2)
        self.device.step()
        self.device.arm()
        self.assertEqual(self.lib.Device_Value(0), 3)
        self.lib.Device_Epoch(3)
        for _ in range(30):
            self.device.step()
        self.assertEqual(self.lib.Device_Value(2), 1)
        self.assertEqual(self.lib.Device_Value(0), 6)

    def test_duplicate_goal_and_conflict_do_not_replay(self):
        self.device.arm()
        goal = trial('straight')
        self.device.assert_ok(self.device.call(GOAL, goal))
        self.device.assert_ok(self.device.call(GOAL, goal))
        self.assertNotEqual(self.device.call(GOAL, trial('side')).payload[0], p.OK)
        self.assertEqual(self.lib.Device_Value(1), 1)

    def test_session_expiry_stops_once(self):
        self.device.arm()
        self.device.call(GOAL, trial('straight'))
        for _ in range(90):
            self.device.step(heart=False)
        self.assertEqual(self.lib.Device_Value(2), 1)
        self.assertEqual(self.lib.Device_Value(0), 6)

    def test_can_fault_stops_and_never_rearms(self):
        self.device.arm()
        self.lib.Device_Error(2)
        self.device.step()
        for _ in range(50):
            self.device.step()
        self.assertEqual(self.lib.Device_Value(2), 1)
        self.assertEqual(self.lib.Device_Value(0), 6)
        self.assertNotEqual(self.device.call(ARM).payload[0], p.OK)

    def test_long_goal_and_arc_are_blocked(self):
        self.device.arm()
        goal = bytearray(trial('straight'))
        import struct
        struct.pack_into('<f', goal, 12, 1.)
        self.assertNotEqual(self.device.call(GOAL, bytes(goal)).payload[0], p.OK)
        struct.pack_into('<I', goal, 8, 3)
        self.assertNotEqual(self.device.call(GOAL, bytes(goal)).payload[0], p.OK)

    def test_only_bounded_fixed_heading_arc_is_accepted(self):
        import struct
        self.device.arm()
        # 不扩大总类权限：只接受这一轮已验算的小弧线，禁止任意半径或转姿。
        for offset, code, value in ((8, 'I', 3), (8, 'I', 5), (12, 'f', .01),
                                    (20, 'f', .1), (24, 'f', .02), (32, 'f', .07),
                                    (32, 'f', float('nan')), (40, 'i', -1), (44, 'i', 1),
                                    (48, 'f', .025), (72, 'I', 1), (76, 'f', .1),
                                    (80, 'f', .02), (92, 'f', .001)):
            goal = bytearray(trial('arc'))
            struct.pack_into('<' + code, goal, offset, value)
            self.assertNotEqual(self.device.call(GOAL, bytes(goal)).payload[0], p.OK)
        self.assertEqual(self.device.call(GOAL, trial('arc')).payload[0], p.OK)

    def test_only_low_speed_counterclockwise_quarter_turn_is_accepted(self):
        import struct
        self.device.arm()
        for offset, value in ((20, -math.pi / 2), (20, math.pi), (20, math.pi / 2 + .01),
                              (12, .01), (60, .11), (64, .35), (68, 1.1)):
            goal = bytearray(trial('turn90'))
            struct.pack_into('<f', goal, offset, value)
            self.assertNotEqual(self.device.call(GOAL, bytes(goal)).payload[0], p.OK)
        self.assertEqual(self.device.call(GOAL, trial('turn90')).payload[0], p.OK)

    def test_only_bounded_synchronous_pose_is_accepted(self):
        import struct
        self.device.arm()
        for offset, code, value in ((12, 'f', .08), (16, 'f', .01), (20, 'f', -.1745329),
                                    (24, 'f', .01), (48, 'f', .018), (52, 'f', .08),
                                    (56, 'f', .3), (60, 'f', .1), (64, 'f', .3),
                                    (68, 'f', 1.), (72, 'I', 1), (80, 'f', .01)):
            goal = bytearray(trial('pose'))
            struct.pack_into('<' + code, goal, offset, value)
            self.assertNotEqual(self.device.call(GOAL, bytes(goal)).payload[0], p.OK)
        self.assertEqual(self.device.call(GOAL, trial('pose')).payload[0], p.OK)

    def test_pose_reference_is_concurrent_and_respects_wheel_limit(self):
        self.device.arm()
        self.assertEqual(self.device.call(GOAL, trial('pose')).payload[0], p.OK)
        point = Reference()
        peaks = dict(velocity=0., acceleration=0., jerk=0., angular_velocity=0.,
                     angular_acceleration=0., angular_jerk=0., wheel_rpm=0.)
        concurrent = 0
        for index in range(3001):
            self.assertEqual(self.lib.Device_Ref(index / 3000, c.byref(point)), 1)
            cosine, sine = math.cos(point.yaw_rad), math.sin(point.yaw_rad)
            vx = cosine * point.vx_m_s + sine * point.vy_m_s
            vy = -sine * point.vx_m_s + cosine * point.vy_m_s
            values = dict(velocity=math.hypot(point.vx_m_s, point.vy_m_s),
                          acceleration=math.hypot(point.ax_m_s2, point.ay_m_s2),
                          jerk=math.hypot(point.jx_m_s3, point.jy_m_s3),
                          angular_velocity=abs(point.wz_rad_s),
                          angular_acceleration=abs(point.angular_accel_rad_s2),
                          angular_jerk=abs(point.angular_jerk_rad_s3),
                          wheel_rpm=(abs(vx) + abs(vy) + .22 * abs(point.wz_rad_s)) * 60 / (2 * math.pi * .05))
            peaks = {key: max(peaks[key], value) for key, value in values.items()}
            concurrent += point.vx_m_s > .001 and point.wz_rad_s > .001
            self.assertAlmostEqual(point.y_m, 0., places=6)
            self.assertGreaterEqual(point.x_m, -1e-6)
            self.assertLessEqual(point.x_m, .060001)
            self.assertGreaterEqual(point.yaw_rad, -1e-6)
            self.assertLessEqual(point.yaw_rad, math.radians(10) + 1e-6)
        self.assertGreater(concurrent, 1500)
        limits = dict(velocity=.012, acceleration=.05, jerk=.2, angular_velocity=.06,
                      angular_acceleration=.2, angular_jerk=.6, wheel_rpm=6)
        for key, limit in limits.items():
            self.assertLessEqual(peaks[key], limit * 1.0003, (key, peaks[key]))
        self.assertAlmostEqual(point.x_m, .06, places=6)
        self.assertAlmostEqual(point.yaw_rad, math.radians(10), places=6)
        (ROOT / 'output/tests/pose-reference.json').write_text(json.dumps(peaks, indent=2), encoding='utf-8')

    def test_quarter_turn_crosses_sensor_wrap_without_replay(self):
        transport = SimTransport(self.lib)
        transport.sensor_sign = -1
        transport.sensor_offset_rad = math.radians(-179)
        value = profile()
        value['yaw_sign'] = -1
        records = []
        with tempfile.TemporaryDirectory() as directory:
            result = run_trial(transport, value, 'turn90', directory,
                               lambda kind, **fields: records.append(dict(kind=kind, **fields)),
                               clock=lambda: transport.now, sleep=transport.sleep, simulation=True)
        self.assertTrue(result['controller_complete'])
        self.assertTrue(transport.crossed_yaw_wrap)
        self.assertAlmostEqual(transport.plant.yaw, math.pi / 2, delta=math.radians(2))
        self.assertEqual(sum(v.kind == GOAL for v in transport.requests), 1)
        self.assertTrue(next(v for v in records if v['kind'] == 'stop_result')['confirmed'])

    def test_old_check_time_reproduces_stale_and_freezes_trigger(self):
        import struct
        self.device.arm()
        tick=self.device.tick
        self.lib.Device_PosTime(tick+1,tick+1,tick)
        reply=self.device.call(37)
        self.assertEqual(reply.payload[0],p.OK)
        record=struct.unpack('<24Iqq',reply.payload[1:])
        self.assertEqual(record[:4],(1,4,1,tick))
        self.assertEqual(record[5:10],(0x36,1,tick+1,tick+1,tick+1))
        self.lib.Device_PosTime(tick+2,tick+2,tick+2)
        self.assertEqual(self.device.call(37).payload,reply.payload)
        state=STATUS_DATA.unpack(self.device.call(STATUS).payload[1:])
        self.assertEqual(state[1],1)

    def test_fresh_check_time_accepts_same_cross_millisecond_frame(self):
        import struct
        self.device.arm()
        tick=self.device.tick
        self.lib.Device_PosTime(tick+1,tick+1,tick+1)
        record=struct.unpack('<24Iqq',self.device.call(37).payload[1:])
        self.assertEqual(record[0],0)
        state=STATUS_DATA.unpack(self.device.call(STATUS).payload[1:])
        self.assertEqual(state[1],0)

    def test_true_250ms_age_remains_rejected_with_trigger_record(self):
        import struct
        self.device.arm()
        tick=self.device.tick
        self.lib.Device_PosTime(tick,tick,tick+250)
        record=struct.unpack('<24Iqq',self.device.call(37).payload[1:])
        self.assertEqual(record[:4],(1,4,1,tick+250))

    def test_two_batch_nine_goals_preserve_order_and_stop(self):
        plan=json.loads((ROOT/'config/batch_route.json').read_text())
        high=profile();high.update(version=5,max_rpm=100)
        transport=SimTransport(self.lib)
        records=[]
        with tempfile.TemporaryDirectory() as directory:
            result=run_trial(transport,high,'straight',directory,
                lambda kind,**v:records.append(dict(kind=kind,**v)),clock=lambda:transport.now,
                sleep=transport.sleep,route=plan,simulation=True)
        self.assertTrue(result['controller_complete'])
        goals=[v for v in transport.requests if v.kind==GOAL]
        self.assertEqual([int.from_bytes(v.payload[:4],'little') for v in goals],list(range(1,10)))
        self.assertEqual(len({(v.boot,v.session) for v in goals}),1)
        self.assertEqual(sum(v.kind==CONFIG for v in transport.requests),1)
        self.assertEqual(sum(v.kind==ARM for v in transport.requests),1)
        self.assertTrue(next(v for v in records if v['kind']=='stop_result')['confirmed'])
        end=plan['boundaries'][-1]
        self.assertLess(math.hypot(transport.plant.x-end[0],transport.plant.y-end[1]),.01)
        self.assertLess(abs(transport.plant.yaw-end[2]),.03)

    def test_pc0_two_batch_route_keeps_one_session_and_nine_handoffs(self):
        plan = json.loads((ROOT/'config/batch_route.json').read_text(encoding='utf-8'))
        value = profile()
        value.update(version=5, max_rpm=100)
        transport = KeyTransport(self.lib, press_at=50.)
        records = []
        with tempfile.TemporaryDirectory() as directory:
            result = run_trial(transport, value, 'straight', directory,
                lambda kind, **v: records.append(dict(kind=kind, **v)), clock=lambda: transport.now,
                sleep=transport.sleep, route=plan, simulation=True, wait_start=True)
        self.assertTrue(result['controller_complete'])
        self.assertEqual(len(result['completed_segments']), 9)
        goals = [v for v in transport.requests if v.kind == GOAL]
        self.assertEqual([int.from_bytes(v.payload[:4], 'little') for v in goals], list(range(1, 10)))
        self.assertEqual(len({(v.boot, v.session) for v in goals}), 1)
        for kind in (CONFIG, ARM, START_PREPARE, p.OPEN):
            self.assertEqual(sum(v.kind == kind for v in transport.requests), 1)
        start = next(v for v in records if v['kind'] == 'physical_key_started')
        self.assertGreaterEqual(start['event']['phase'], 4)
        self.assertEqual(start['event']['reason'], 0)
        self.assertTrue(next(v for v in records if v['kind'] == 'stop_result')['confirmed'])
        self.assertEqual(self.lib.Device_Value(7), 0)
        end = plan['boundaries'][-1]
        self.assertLess(math.hypot(transport.plant.x-end[0], transport.plant.y-end[1]), .01)
        self.assertLess(abs(transport.plant.yaw-end[2]), .03)

    def test_pc0_batch_timeout_and_stale_gyro_never_arm(self):
        plan = json.loads((ROOT/'config/batch_route.json').read_text(encoding='utf-8'))
        value = profile()
        value.update(version=5, max_rpm=100)
        for case, press_at, drop_at, reason in [('timeout', None, None, 3), ('gyro', 2., 1., 1)]:
            with self.subTest(case=case), tempfile.TemporaryDirectory() as directory:
                transport = KeyTransport(self.lib, press_at=press_at, drop_gyro_at=drop_at)
                records = []
                with self.assertRaisesRegex(ValueError, '按键准备'):
                    run_trial(transport, value, 'straight', directory,
                        lambda kind, **v: records.append(dict(kind=kind, **v)), clock=lambda: transport.now,
                        sleep=transport.sleep, route=plan, simulation=True, wait_start=True)
                event = next(v['event'] for v in records if v['kind'] == 'physical_key_cancelled')
                self.assertEqual(event['reason'], reason)
                self.assertFalse(any(v.kind in (ARM, GOAL) for v in transport.requests))
                self.assertEqual(self.lib.Device_Value(4), 0)
                self.assertEqual(self.lib.Device_Value(3), 0)
                if case == 'gyro':
                    self.assertEqual(event['guard_mask'] & 512, 512)

    def test_explicit_pc0_entry_rejects_legacy_firmware_before_config(self):
        class LegacyLink(MotionLink):
            def feed(self, wire, now):
                super().feed(wire, now)
                if self.firmware:
                    self.firmware = 'GC-BASE-0.3.11'
        plan = json.loads((ROOT/'config/batch_route.json').read_text(encoding='utf-8'))
        value = profile()
        value.update(version=5, max_rpm=100)
        transport = SimTransport(self.lib)
        with tempfile.TemporaryDirectory() as directory, patch('robot.drive_core.MotionLink', LegacyLink):
            with self.assertRaisesRegex(ValueError, '要求PC0协议'):
                run_trial(transport, value, 'straight', directory, lambda *a, **k: None,
                    clock=lambda: transport.now, sleep=transport.sleep, route=plan,
                    simulation=True, wait_start=True)
        self.assertFalse(any(v.kind in (CONFIG, ARM, GOAL) for v in transport.requests))

    def test_can_candidate_does_not_open_physical_motion_entry(self):
        transport = SimTransport(self.lib)
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, '固件或使能状态不符合'):
                run_trial(transport, profile(), 'straight', directory, lambda *a, **k: None,
                          clock=lambda: transport.now, sleep=transport.sleep, simulation=False)
        self.assertFalse(any(v.kind in (CONFIG, ARM, GOAL, START_PREPARE) for v in transport.requests))

    def test_diagnostic_capabilities_do_not_authorize_new_motion_entries(self):
        from robot.nav.selection import routes
        switched = json.loads((ROOT/'config/switch_trials.json').read_text(encoding='utf-8'))
        for firmware in ('GC-BASE-0.3.15', 'GC-BASE-0.3.16', 'GC-BASE-0.3.25',
                         'GC-BASE-0.3.34', 'GC-BASE-0.3.41', 'GC-BASE-0.3.42',
                         'GC-BASE-0.3.43', 'GC-BASE-0.4.0', 'GC-BASE-0.3.42-dev'):
            class ReportedLink(MotionLink):
                def feed(self, wire, now):
                    super().feed(wire, now)
                    if self.firmware:
                        self.firmware = firmware
            for entry in ('short', 'pc0', 'pc1'):
                kwargs = {} if entry == 'short' else dict(wait_start=True)
                if entry == 'pc1':
                    kwargs['switch_routes'] = routes()
                transport = SimTransport(self.lib)
                with self.subTest(firmware=firmware, entry=entry), tempfile.TemporaryDirectory() as directory:
                    with patch('robot.drive_core.MotionLink', ReportedLink), self.assertRaises(ValueError):
                        run_trial(transport, switched if entry == 'pc1' else profile(), 'straight',
                                  directory, lambda *a, **k: None, clock=lambda: transport.now,
                                  sleep=transport.sleep, simulation=False, **kwargs)
                self.assertFalse(any(v.kind in (CONFIG, ARM, GOAL, START_PREPARE) for v in transport.requests))

    def test_pc0_batch_cli_preview_never_opens_transport(self):
        import contextlib
        import io
        from robot.drive_core import main
        output = io.StringIO()
        args = ['drive_core', '--route', str(ROOT/'config/batch_route.json'),
                '--profile', str(ROOT/'config/batch_trials.json'), '--wait-key']
        with patch('sys.argv', args), patch('robot.drive_core.open_transport') as opened:
            with contextlib.redirect_stdout(output):
                main()
        self.assertFalse(opened.called)
        preview = json.loads(output.getvalue())
        self.assertTrue(preview['physical_key_required'])
        self.assertFalse(preview['command_sent'])
        self.assertEqual(preview['route']['cases'], [f'batch_{i}' for i in range(1, 10)])

    def test_start2_saved_plan_pc0_closed_loop_and_actual_sweep(self):
        from robot.nav.field import Field
        from robot.nav.frames import Pose, compose
        plan = json.loads((ROOT/'config/start2_route.json').read_text(encoding='utf-8'))
        value = json.loads((ROOT/'config/start2_trials.json').read_text(encoding='utf-8'))
        field = Field(data=plan['field'])
        origin = Pose(*plan['map_origin'])
        self.assertEqual(origin.yaw_rad, 0)
        first_spec = Spec.from_buffer_copy(bytes.fromhex(plan['goals'][0])[8:])
        self.assertAlmostEqual(first_spec.x_m, -1.05, places=6)
        self.assertAlmostEqual(first_spec.y_m, -.20, places=6)
        self.assertEqual(first_spec.yaw_rad, 0)
        transport = KeyTransport(self.lib)
        transport.plant = Plant(ideal=True, radius_m=value['wheel_radius_m'],
                                k_m=value['half_length_m']+value['half_width_m'])
        transport.sensor_sign = -1
        transport.sensor_offset_rad = .7
        checked = [-1]
        trace = []
        def sweep(plant):
            speed = max(abs(rpm) for sample in plant.history for rpm in sample[1]) * 2*math.pi*plant.radius_m/60
            movement = (math.sqrt(2)*speed+(field.radius+.04)*speed/plant.k_m)*.002
            for tick, x, y, yaw in plant.positions:
                if tick <= checked[0]:
                    continue
                pose = compose(origin, Pose(x, y, yaw))
                self.assertEqual(field.errors(pose, 'start_2', movement, tracking=False), [],
                                 f'实际模型扫掠失败：{tick}, {pose}')
                checked[0] = tick
                if tick % 100 == 0:
                    trace.append(pose.values())
        transport.sweep = sweep
        records = []
        with tempfile.TemporaryDirectory() as directory:
            result = run_trial(transport, value, 'straight', directory,
                lambda kind, **v: records.append(dict(kind=kind, **v)), clock=lambda: transport.now,
                sleep=transport.sleep, route=plan, simulation=True, wait_start=True)
        self.assertTrue(result['controller_complete'])
        self.assertEqual(len(result['completed_segments']), 9)
        goals = [v for v in transport.requests if v.kind == GOAL]
        self.assertEqual([int.from_bytes(v.payload[:4], 'little') for v in goals], list(range(1, 10)))
        self.assertEqual(len({(v.boot, v.session) for v in goals}), 1)
        self.assertEqual(sum(v.kind == CONFIG for v in transport.requests), 1)
        self.assertEqual(sum(v.kind == ARM for v in transport.requests), 1)
        self.assertTrue(next(v for v in records if v['kind'] == 'stop_result')['confirmed'])
        self.assertEqual(self.lib.Device_Value(7), 0)
        final = compose(origin, Pose(transport.plant.x, transport.plant.y, transport.plant.yaw))
        self.assertTrue(field.parked(final, 'start_2'))
        first_move = next(point for point in trace if math.dist(point[:2], origin.values()[:2]) > .005)
        self.assertLess(first_move[0], origin.x_m)  # 跳过按键等待帧，起步应朝地图-X。

    def test_start2_white_list_rejects_speed_order_and_wrong_version(self):
        from robot.device.tracking import Spec
        plan = json.loads((ROOT/'config/start2_route.json').read_text(encoding='utf-8'))
        value = profile()
        value.update(version=8, max_rpm=100)
        self.device.open()
        self.device.assert_ok(self.device.call(CONFIG, config_payload(value)))
        for _ in range(8):self.device.step()
        self.device.assert_ok(self.device.call(ARM))
        for _ in range(8):self.device.step()
        self.assertNotEqual(self.device.call(GOAL, bytes.fromhex(plan['goals'][1])).payload[0], p.OK)
        raw = bytearray.fromhex(plan['goals'][0])
        spec = Spec.from_buffer_copy(raw[8:])
        spec.linear.velocity = .31
        raw[8:] = bytes(spec)
        self.assertNotEqual(self.device.call(GOAL, bytes(raw)).payload[0], p.OK)
        self.assertEqual(self.device.call(GOAL, bytes.fromhex(plan['goals'][0])).payload[0], p.OK)
        self.device = Device(self.lib)
        self.device.open()
        value['version'] = 5
        self.device.assert_ok(self.device.call(CONFIG, config_payload(value)))
        for _ in range(8):self.device.step()
        self.device.assert_ok(self.device.call(ARM))
        for _ in range(8):self.device.step()
        self.assertNotEqual(self.device.call(GOAL, bytes.fromhex(plan['goals'][0])).payload[0], p.OK)
        with self.assertRaises(ValueError):
            config_payload(dict(value, version=7))
        # 单独冷启动验证旧version7在固件侧被拒绝，避免通过改主机标记复活。
        self.device = Device(self.lib)
        self.device.open()
        config = bytearray(config_payload(dict(value, version=8)))
        config[:4] = (7).to_bytes(4, 'little')
        self.assertNotEqual(self.device.call(CONFIG, bytes(config)).payload[0], p.OK)

    def test_start2_map_origin_tampering_is_rejected(self):
        from robot.nav.route import validate, digest
        plan = json.loads((ROOT/'config/start2_route.json').read_text(encoding='utf-8'))
        validate(plan)
        plan['map_origin'][2] = math.pi
        plan['sha256'] = digest(plan)
        with self.assertRaises(ValueError):
            validate(plan)

    def test_start2_refuses_previous_firmware_before_any_configuration(self):
        class PreviousLink(MotionLink):
            def feed(self, wire, now):
                super().feed(wire, now)
                if self.firmware:
                    self.firmware = previous
        route = json.loads((ROOT/'config/start2_route.json').read_text(encoding='utf-8'))
        value = json.loads((ROOT/'config/start2_trials.json').read_text(encoding='utf-8'))
        for previous in ('GC-BASE-0.3.17', 'GC-BASE-0.3.18'):
            transport = SimTransport(self.lib)
            with self.subTest(firmware=previous), tempfile.TemporaryDirectory() as directory:
                with patch('robot.drive_core.MotionLink', PreviousLink):
                    with self.assertRaisesRegex(ValueError, '要求PC0协议'):
                        run_trial(transport, value, 'straight', directory, lambda *a, **k: None,
                            clock=lambda: transport.now, sleep=transport.sleep, route=route,
                            simulation=True, wait_start=True)
            self.assertFalse(any(v.kind in (CONFIG, ARM, GOAL) for v in transport.requests))

    def test_batch_repeated_geometry_cannot_skip_first_batch(self):
        plan=json.loads((ROOT/'config/batch_route.json').read_text())
        high=profile();high.update(version=5,max_rpm=100)
        self.device.open()
        self.device.assert_ok(self.device.call(CONFIG,config_payload(high)))
        for _ in range(8):self.device.step()
        self.device.assert_ok(self.device.call(ARM))
        for _ in range(8):self.device.step()
        self.assertNotEqual(self.device.call(GOAL,bytes.fromhex(plan['goals'][5])).payload[0],p.OK)
        self.assertEqual(self.device.call(GOAL,bytes.fromhex(plan['goals'][0])).payload[0],p.OK)
        self.assertNotEqual(self.device.call(GOAL,bytes.fromhex(plan['goals'][1+1])).payload[0],p.OK)

    def test_tour_six_goals_complete_with_continuous_odom(self):
        plan=json.loads((ROOT/'config/tour_route.json').read_text())
        high=profile(); high.update(version=4,max_rpm=100)
        transport=SimTransport(self.lib)
        records=[]
        with tempfile.TemporaryDirectory() as directory:
            result=run_trial(transport,high,'straight',directory,
                lambda kind,**v:records.append(dict(kind=kind,**v)),clock=lambda:transport.now,
                sleep=transport.sleep,route=plan,simulation=True)
        self.assertTrue(result['controller_complete'])
        goals=[v for v in transport.requests if v.kind==GOAL]
        self.assertEqual([int.from_bytes(v.payload[:4],'little') for v in goals],list(range(1,7)))
        self.assertEqual(len({(v.boot,v.session) for v in goals}),1)
        self.assertEqual(sum(v.kind==CONFIG for v in transport.requests),1)
        self.assertEqual(sum(v.kind==ARM for v in transport.requests),1)
        self.assertTrue(next(v for v in records if v['kind']=='stop_result')['confirmed'])
        self.assertLess(math.hypot(transport.plant.x-.001,transport.plant.y+.011),.01)
        self.assertLess(abs(transport.plant.yaw-math.pi),.03)

    def test_tour_order_and_adjustment_limits(self):
        plan=json.loads((ROOT/'config/tour_route.json').read_text())
        high=profile();high.update(version=4,max_rpm=100)
        self.device.open()
        self.device.assert_ok(self.device.call(CONFIG,config_payload(high)))
        for _ in range(8):self.device.step()
        self.device.assert_ok(self.device.call(ARM))
        for _ in range(8):self.device.step()
        self.assertNotEqual(self.device.call(GOAL,bytes.fromhex(plan['goals'][-1])).payload[0],p.OK)
        for field,value in (('x_m',1.1),('yaw_rad',float('nan'))):
            target=Goal.from_buffer_copy(bytes.fromhex(plan['goals'][0]))
            setattr(target.reference,field,value)
            self.assertNotEqual(self.device.call(GOAL,bytes(target)).payload[0],p.OK)
        target=Goal.from_buffer_copy(bytes.fromhex(plan['goals'][0]))
        target.reference.linear.velocity=.31
        self.assertNotEqual(self.device.call(GOAL,bytes(target)).payload[0],p.OK)
        self.assertEqual(self.device.call(GOAL,bytes.fromhex(plan['goals'][0])).payload[0],p.OK)
        self.assertNotEqual(self.device.call(GOAL,bytes.fromhex(plan['goals'][2])).payload[0],p.OK)

    def test_tour_anchor_does_not_hide_large_drift(self):
        from robot.nav.route import anchor_spec
        from robot.nav.frames import Pose
        plan=json.loads((ROOT/'config/tour_route.json').read_text())
        spec=Goal.from_buffer_copy(bytes.fromhex(plan['goals'][0])).reference
        with self.assertRaises(ValueError):
            anchor_spec(spec,Pose(0,0,0),Pose(1.05,-.2,0),Pose(.02,0,0))
        with self.assertRaises(ValueError):
            anchor_spec(spec,Pose(0,0,0),Pose(1.05,-.2,0),Pose(0,0,.04))

    def test_tour_cannot_use_raw_profile_or_mutated_goals(self):
        from robot.nav.route import digest
        plan=json.loads((ROOT/'config/tour_route.json').read_text())
        for kind in ('profile','payload'):
            transport=SimTransport(self.lib)
            high=profile();high.update(version=3 if kind=='profile' else 4,max_rpm=100)
            if kind=='payload':
                plan['goals'][-1]='00'*96;plan['sha256']=digest(plan)
            with tempfile.TemporaryDirectory() as directory,self.assertRaises(ValueError):
                run_trial(transport,high,'straight',directory,lambda *a,**k:None,route=plan,simulation=True)
            self.assertEqual(transport.requests,[])

    def test_raw_route_frozen_specs_complete_one_session(self):
        from robot.nav.route import validate
        plan = json.loads((ROOT/'config/raw_route.json').read_text(encoding='utf-8'))
        validate(plan)
        transport = SimTransport(self.lib)
        high = profile()
        high.update(version=3, max_rpm=100)
        records = []
        with tempfile.TemporaryDirectory() as directory:
            result = run_trial(transport, high, 'straight', directory,
                lambda kind, **v: records.append(dict(kind=kind, **v)),
                clock=lambda: transport.now, sleep=transport.sleep, route=plan, simulation=True)
        self.assertTrue(result['controller_complete'])
        self.assertEqual(sum(v.kind==ARM for v in transport.requests), 1)
        self.assertEqual(sum(v.kind==GOAL for v in transport.requests), 2)
        self.assertEqual(sum(v.kind==CONFIG for v in transport.requests), 1)
        self.assertTrue(next(v for v in records if v['kind']=='stop_result')['confirmed'])
        self.assertLess(math.hypot(transport.plant.x-.19, transport.plant.y+1.05), .015)
        self.assertLess(abs(transport.plant.yaw-math.pi/2), .04)

    def arm_raw(self):
        high=profile(); high.update(version=3,max_rpm=100)
        self.device.open()
        self.device.assert_ok(self.device.call(CONFIG,config_payload(high)))
        for _ in range(8): self.device.step()
        self.device.assert_ok(self.device.call(ARM))
        for _ in range(8): self.device.step()
        return json.loads((ROOT/'config/raw_route.json').read_text(encoding='utf-8'))

    def test_raw_order_mutation_and_low_speed_goal_rejected_in_high_session(self):
        plan=self.arm_raw()
        self.assertNotEqual(self.device.call(GOAL,bytes.fromhex(plan['goals'][1])).payload[0],p.OK)
        self.assertNotEqual(self.device.call(GOAL,trial('straight')).payload[0],p.OK)
        target=Goal.from_buffer_copy(bytes.fromhex(plan['goals'][0]))
        target.reference.linear.velocity=.31
        self.assertNotEqual(self.device.call(GOAL,bytes(target)).payload[0],p.OK)
        target=Goal.from_buffer_copy(bytes.fromhex(plan['goals'][0]))
        target.timeout_ms=30001
        self.assertNotEqual(self.device.call(GOAL,bytes(target)).payload[0],p.OK)
        self.assertEqual(self.device.call(GOAL,bytes.fromhex(plan['goals'][0])).payload[0],p.OK)
        self.assertNotEqual(self.device.call(GOAL,bytes.fromhex(plan['goals'][1])).payload[0],p.OK)

    def test_raw_lost_second_ack_stops_without_replay(self):
        class LostAck(SimTransport):
            def read(self,count=4096):
                data=super().read(count)
                if sum(v.kind==GOAL for v in self.requests)==2:
                    if any(v.kind==GOAL|0x80 for v in p.Decoder().feed(data)):return b''
                return data
        plan=json.loads((ROOT/'config/raw_route.json').read_text(encoding='utf-8'))
        high=profile(); high.update(version=3,max_rpm=100)
        transport=LostAck(self.lib)
        records=[]
        with tempfile.TemporaryDirectory() as directory,self.assertRaises(ValueError):
            run_trial(transport,high,'straight',directory,lambda kind,**v:records.append(dict(kind=kind,**v)),
                      clock=lambda:transport.now,sleep=transport.sleep,route=plan,simulation=True)
        self.assertEqual(sum(v.kind==GOAL for v in transport.requests),2)
        self.assertEqual(sum(v.kind==ARM for v in transport.requests),1)
        self.assertTrue(next(v for v in records if v['kind']=='stop_result')['confirmed'])

    def test_raw_goal_cannot_enter_low_speed_session(self):
        self.device.arm()
        plan=json.loads((ROOT/'config/raw_route.json').read_text(encoding='utf-8'))
        self.assertNotEqual(self.device.call(GOAL,bytes.fromhex(plan['goals'][0])).payload[0],p.OK)

    def test_raw_modified_package_rejected_before_write(self):
        from robot.nav.route import digest
        plan=json.loads((ROOT/'config/raw_route.json').read_text(encoding='utf-8'))
        plan['goals'][0]='00'*96
        plan['sha256']=digest(plan)
        transport=SimTransport(self.lib)
        high=profile(); high.update(version=3,max_rpm=100)
        with tempfile.TemporaryDirectory() as directory,self.assertRaises(ValueError):
            run_trial(transport,high,'straight',directory,lambda *a,**k:None,route=plan,simulation=True)
        self.assertEqual(transport.requests,[])

    def test_north_gc_closed_loop_one_session_and_final_stop(self):
        transport = SimTransport(self.lib)
        records = []
        with tempfile.TemporaryDirectory() as directory:
            result = run_trial(transport, profile(), 'north', directory,
                lambda kind, **v: records.append(dict(kind=kind, **v)),
                clock=lambda: transport.now, sleep=transport.sleep, simulation=True)
        self.assertTrue(result['controller_complete'])
        self.assertLess(math.hypot(transport.plant.x-1.05, transport.plant.y+.13), .008)
        self.assertEqual(sum(v.kind == GOAL for v in transport.requests), 1)
        self.assertEqual(sum(v.kind == ARM for v in transport.requests), 1)
        self.assertTrue(next(v for v in records if v['kind']=='stop_result')['confirmed'])

    def test_north_stale_placement_boot_never_arms(self):
        transport = SimTransport(self.lib)
        with tempfile.TemporaryDirectory() as directory, \
                patch('robot.nav.site.require_north', return_value={'placement_boot_id':999}), \
                self.assertRaises(ValueError):
            run_trial(transport, profile(), 'north', directory, lambda *a, **k: None,
                      clock=lambda: transport.now, sleep=transport.sleep)
        self.assertEqual(sum(v.kind == ARM for v in transport.requests), 0)
        self.assertEqual(sum(v.kind == GOAL for v in transport.requests), 0)

    def test_north_cross_track_departure_stops_without_replay(self):
        class Departure(SimTransport):
            def sleep(self, seconds):
                super().sleep(seconds)
                if self.now > 8 and not getattr(self, 'injected', False):
                    self.plant.y += .05
                    # 纯平移跳变不会进入轮里程计，使用四轮编码器模拟侧向滑行。
                    delta = .05 / (2 * math.pi * .05)
                    self.plant.turns[:] = [v+s*delta for v,s in zip(self.plant.turns,[-1,1,1,-1])]
                    self.injected = True
        transport = Departure(self.lib)
        with tempfile.TemporaryDirectory() as directory, self.assertRaises(ValueError):
            run_trial(transport, profile(), 'north', directory, lambda *a, **k: None,
                      clock=lambda: transport.now, sleep=transport.sleep, simulation=True)
        self.assertEqual(sum(v.kind == GOAL for v in transport.requests), 1)
        self.assertEqual(sum(v.kind == ARM for v in transport.requests), 1)

    def test_north_only_frozen_payload_is_accepted(self):
        self.device.arm()
        for change in ('distance','time','speed','unused'):
            candidate = Goal.from_buffer_copy(trial('north'))
            if change == 'distance': candidate.reference.x_m += .01
            if change == 'time': candidate.timeout_ms = 120000
            if change == 'speed': candidate.reference.linear.velocity = .019
            if change == 'unused': candidate.reference.tangent = 1
            self.assertNotEqual(self.device.call(GOAL, bytes(candidate)).payload[0], p.OK)
        self.assertEqual(self.device.call(GOAL, trial('north')).payload[0], p.OK)

    def test_north_real_execution_refuses_unconfirmed_placement_before_any_write(self):
        transport = SimTransport(self.lib)
        with tempfile.TemporaryDirectory() as directory, \
                patch('robot.nav.site.read_site', return_value={}), self.assertRaises(ValueError):
            run_trial(transport, profile(), 'north', directory, lambda *a, **k: None)
        self.assertEqual(transport.requests, [])

    def test_arc_reference_stays_within_geometry_and_motion_limits(self):
        self.device.arm()
        self.assertEqual(self.device.call(GOAL, trial('arc')).payload[0], p.OK)
        point = Reference()
        maximum = dict(velocity=0., acceleration=0., jerk=0., wheel_rpm=0.)
        for index in range(3001):
            self.assertEqual(self.lib.Device_Ref(index / 3000, c.byref(point)), 1)
            self.assertLessEqual(math.hypot(point.x_m, point.y_m), .15)
            self.assertGreaterEqual(point.x_m, -1e-6)
            self.assertGreaterEqual(point.y_m, -1e-6)
            self.assertAlmostEqual(point.yaw_rad, 0.)
            maximum['velocity'] = max(maximum['velocity'], math.hypot(point.vx_m_s, point.vy_m_s))
            maximum['acceleration'] = max(maximum['acceleration'], math.hypot(point.ax_m_s2, point.ay_m_s2))
            maximum['jerk'] = max(maximum['jerk'], math.hypot(point.jx_m_s3, point.jy_m_s3))
            maximum['wheel_rpm'] = max(maximum['wheel_rpm'],
                                       (abs(point.vx_m_s) + abs(point.vy_m_s)) * 60 / (2 * math.pi * .05))
        for key, limit in dict(velocity=.018, acceleration=.08, jerk=.3, wheel_rpm=6).items():
            self.assertLessEqual(maximum[key], limit * 1.0003, (key, maximum[key]))
        report = dict(maximum=maximum, target_m_rad=[point.x_m, point.y_m, point.yaw_rad], samples=3001)
        (ROOT / 'output/tests/arc-reference.json').write_text(json.dumps(report, indent=2), encoding='utf-8')

    def test_imu_sign_missing_blocks_before_serial(self):
        value = profile()
        value['yaw_sign'] = None
        with self.assertRaises(ValueError):
            config_payload(value)

    def test_clockwise_positive_sensor_becomes_counterclockwise_positive(self):
        transport = SimTransport(self.lib)
        transport.sensor_sign = -1
        value = profile()
        value['yaw_sign'] = -1
        with tempfile.TemporaryDirectory() as directory:
            result = run_trial(transport, value, 'turn', directory, lambda *a, **k: None,
                               clock=lambda: transport.now, sleep=transport.sleep, simulation=True)
        self.assertTrue(result['controller_complete'])
        self.assertGreater(transport.plant.yaw, .15)
        self.assertLess(transport.plant.yaw, .20)

    def test_owner_running_with_disconnected_board_still_blocks(self):
        with patch('robot.drive_core.build_opener'):
            with self.assertRaises(ValueError):
                require_owner_stopped('http://172.20.10.4:8080')

    def test_observe_reads_raw_imu_without_arming(self):
        transport = SimTransport(self.lib)
        with tempfile.TemporaryDirectory() as directory:
            observe(transport, directory, 1.5, clock=lambda: transport.now, sleep=transport.sleep)
            state = json.loads((Path(directory) / 'state.json').read_text(encoding='utf-8'))
        self.assertTrue(state['execution']['stop_confirmed'])
        self.assertIn('4', state['raw_imu'])
        self.assertIn('5', state['raw_imu'])
        self.assertTrue(any(v.kind == 37 for v in transport.requests))
        self.assertTrue(any(v.kind == START_STATE for v in transport.requests))
        self.assertFalse(any(v.kind in (CONFIG, ARM, GOAL, p.STOP) for v in transport.requests))
        self.assertEqual(self.lib.Device_Value(4), 0)

    def test_selected_feedback_channel_and_original_stamp(self):
        self.device.open()
        self.device.step(yaw=123, gyro=45)
        reply = self.device.call(FEEDBACK, bytes([4, 0]))
        request = p.Packet(FEEDBACK, 123, 456, self.device.request, 250, bytes([4, 0]))
        sample = Feedback.read(reply, request, .010, .012)
        self.assertEqual(sample.values[2], 123)
        self.assertEqual(sample.last_ms, 10)
        wrong = p.Packet(FEEDBACK, 123, 456, self.device.request, 250, bytes([5, 0]))
        with self.assertRaises(ValueError):
            Feedback.read(reply, wrong, .010, .012)

    def test_motion_stop_preempts_pending_and_never_replays(self):
        link = MotionLink()
        link.bound = True
        link.boot = 123
        link.last_heart = 1
        link.submit(GOAL, trial('straight'))
        first = p.Decoder().feed(link.step(1))[0]
        self.assertEqual(first.kind, GOAL)
        link.stop()
        stopped = p.Decoder().feed(link.step(1.01))[0]
        self.assertEqual(stopped.kind, p.STOP)
        self.assertNotEqual(first.request, stopped.request)
        link.feed(p.Packet(GOAL | 0x80, 123, link.session, first.request, 0, b'\0').encode(), 1.02)
        self.assertEqual(link.pending[0].kind, p.STOP)

    def test_web_trials_are_readonly_and_snapshot_expires(self):
        with tempfile.TemporaryDirectory() as directory:
            client = TestClient(create_app(directory))
            self.assertFalse(client.get('/api/base/trials').json()['executable'])
            self.assertEqual(client.post('/api/base/trials', json={}).status_code, 405)
            state = dict(mode='serial', readonly=True, connected=False, snapshot_at_s=time.monotonic(),
                         snapshot_wall_s=time.time(), execution={'mode': 4}, execution_age_ms=100)
            target = Path(directory) / 'state.json'
            save_snapshot(target, state)
            self.assertIsNone(load_snapshot(target, now=state['snapshot_at_s'] + .2)['execution'])

    def test_stale_wheel_or_imu_stops(self):
        for mask in (1, 2, 4, 8):
            self.device = Device(self.lib)
            self.device.arm()
            self.device.call(GOAL, trial('straight'))
            self.lib.Device_Drop(mask)
            for _ in range(60):
                self.device.step()
            self.assertEqual(self.lib.Device_Value(2), 1)
            self.assertEqual(self.lib.Device_Value(0), 6)

    def test_normal_stop_can_observe_without_automatic_enable(self):
        self.device.arm()
        self.device.call(p.STOP)
        for _ in range(40):
            self.device.step()
        self.assertEqual(self.lib.Device_Value(0), 1)
        enabled = self.lib.Device_Value(4)
        self.device.call(CONFIG, config_payload(profile()))
        for _ in range(20):
            self.device.step()
        self.assertEqual(self.lib.Device_Value(4), enabled)
        self.assertEqual(self.lib.Device_Value(1), 1)

    def test_link_fault_during_normal_stop_cannot_rearm(self):
        self.device.arm()
        self.device.call(p.STOP)
        self.lib.Device_LinkFault(2)
        for _ in range(30):
            self.device.step(heart=False)
        self.assertEqual(self.lib.Device_Value(0), 6)
        self.assertNotEqual(self.lib.Device_Value(7), 0)
        self.assertEqual(self.lib.Device_Value(1), 1)

    def test_stop_reply_precedes_ordinary_queue(self):
        self.device.arm()
        self.device.write(p.Packet(STATUS, 123, 456, 99, 250).encode())
        self.device.write(p.Packet(p.STOP, 123, 456, 100, 250).encode())
        self.assertEqual(p.Decoder().feed(self.device.read())[0].kind, p.STOP | 0x80)
        self.assertEqual(self.lib.Device_Value(2), 1)

    def test_zero_tx_ack_is_required_for_completion(self):
        transport = SimTransport(self.lib)
        self.lib.Device_Ack(0)
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):
                run_trial(transport, profile(), 'straight', directory, lambda *a, **k: None,
                          clock=lambda: transport.now, sleep=transport.sleep, simulation=True)
        self.assertNotEqual(self.lib.Device_Value(5), 4)
        self.assertEqual(sum(v.kind == GOAL for v in transport.requests), 1)

    def test_real_core_protocol_runs_short_cases_offline(self):
        summary = []
        for case in ('straight', 'side', 'turn', 'turn90', 'diagonal', 'arc', 'pose'):
            transport = SimTransport(self.lib)
            records = []
            with tempfile.TemporaryDirectory() as directory:
                result = run_trial(transport, profile(), case, directory,
                                   lambda kind, **values: records.append(dict(kind=kind, **values)),
                                   clock=lambda: transport.now, sleep=transport.sleep, simulation=True)
                final = json.loads((Path(directory) / 'state.json').read_text(encoding='utf-8'))
                stop = next(v for v in reversed(records) if v['kind'] == 'stop_result')
                self.assertTrue(final['execution']['stop_confirmed'])
                self.assertEqual(final['execution'], stop['execution'])
            self.assertTrue(result['controller_complete'], (case, records[-5:]))
            self.assertEqual(sum(v.kind == GOAL for v in transport.requests), 1)
            self.assertEqual(sum(v.kind == p.STOP for v in transport.requests), 1)
            summary.append(dict(case=case, elapsed_ms=transport.plant.ms,
                                true_x_m=transport.plant.x, true_y_m=transport.plant.y,
                                true_yaw_rad=transport.plant.yaw, result=result))
        (ROOT / 'output/tests/execution-summary.json').write_text(
            json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')

    def test_lost_goal_ack_never_replays(self):
        class LostAck(SimTransport):
            def read(self, count=4096):
                data = super().read(count)
                packets = p.Decoder().feed(data)
                return b'' if any(v.kind == GOAL | 0x80 for v in packets) else data
        transport = LostAck(self.lib)
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):
                run_trial(transport, profile(), 'straight', directory, lambda *a, **k: None,
                          clock=lambda: transport.now, sleep=transport.sleep, simulation=True)
        self.assertEqual(sum(v.kind == GOAL for v in transport.requests), 1)
        self.assertEqual(sum(v.kind == p.STOP for v in transport.requests), 1)


class ReusedBusTests(unittest.TestCase):
    def test_carcan13_transport_bus_and_protocol(self):
        output = ROOT / 'output/tests'
        output.mkdir(exist_ok=True, parents=True)
        device = ROOT / 'firmware/User/Device'
        cases = [('can_transport', ['can_tx.c'], ['-include', str(ROOT / 'tests/can_hal_stub.h')]),
                 ('motor_bus', ['bus.c', 'drive.c'], []), ('motor_protocol', ['drive.c'], [])]
        for name, sources, options in cases:
            program = output / f'reused_{name}.exe'
            compile_c(program, [ROOT / f'tests/reused_{name}.c', *(device / name for name in sources)],
                      extra=['-DCAR_CHASSIS_HOST_TEST', *options])
            for args in ([str(i)] for i in range(35)) if name == 'motor_bus' else [[]]:
                subprocess.run([str(program), *args], check=True, capture_output=True)


if __name__ == '__main__':
    unittest.main()
