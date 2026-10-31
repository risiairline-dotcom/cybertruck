"""短段执行协议；复用Link会话/保活，仅显式命令能配置、使能和运动。"""

import math
import struct

from . import protocol as p
from .link import Link
from .feedback import Feedback, FEEDBACK
from .can_capture import CAN_CAPTURE, CaptureReader
from .capabilities import firmware_capabilities
from .tracking import Spec

CONFIG, ARM, GOAL, STATUS = range(32, 36)
START_PREPARE, START_STATE = 59, 60
CONFIG_DATA = struct.Struct('<I3fIfifff4B4b4b')
STATUS_DATA = struct.Struct('<11I9f')
SPEC_DATA = struct.Struct('<I3f4f2i6fI5f')
MODE = ('DISABLED', 'OBSERVING', 'ARMING', 'READY', 'ACTIVE', 'STOPPING', 'LOCKED')
ACTION = ('IDLE', 'ACCEPTED', 'RUNNING', 'SETTLING', 'COMPLETE', 'FAILED', 'CANCELLED')


def config_payload(profile):
    """配置候选必须显式补齐方向；不由模拟值或网页图标生成。"""
    if profile.get('version') == 7:
        raise ValueError('旧启停区2车头-X配置已停用，请使用车头+X的version8')
    if (profile.get('domain') != 'commissioning' or type(profile.get('yaw_sign')) is not int
            or profile['yaw_sign'] not in (-1, 1)):
        raise ValueError('试验配置缺少明确的IMU方向；不得用模拟值补齐')
    geometry = [profile[key] for key in ('wheel_radius_m', 'half_length_m', 'half_width_m')]
    if any(type(v) not in (int, float) or not math.isfinite(v) or not 0 < v <= .5 for v in geometry):
        raise ValueError('试验几何无效')
    counts, maximum = profile['counts_per_rev'], profile['max_rpm']
    allowed = maximum == 150 if profile['version'] == 6 else maximum == 100 if profile['version'] in (3, 4, 5, 8, 9) else 1 <= maximum <= 10
    if counts != 65536 or type(maximum) is not int or not allowed:
        raise ValueError('反馈单位或试验轮速不符合本阶段范围')
    addresses = profile['addresses']
    if (len(addresses) != 4 or len(set(addresses)) != 4
            or any(type(v) is not int or not 1 <= v <= 255 for v in addresses)):
        raise ValueError('轮地址无效')
    signs = profile['command_sign'] + profile['feedback_sign']
    if len(signs) != 8 or any(type(v) is not int or v not in (-1, 1) for v in signs):
        raise ValueError('轮极性无效')
    return CONFIG_DATA.pack(profile['version'], *geometry, 1, counts, profile['yaw_sign'],
                            profile['angle_scale_rad'], profile['gyro_scale_rad_s'], maximum,
                            *addresses, *signs)


def trial(name, action_id=1):
    """目标相对动作开始的车体，X前/Y左/逆时针正；不表示实测通过。"""
    if name in ('cal_slow', 'cal_medium', 'cal_fast'):
        from pathlib import Path
        import json
        profile = json.loads((Path(__file__).resolve().parents[3] / 'config/calibration_trials.json').read_text(encoding='utf-8'))
        if action_id != 1:
            raise ValueError('每次标定只允许单独一个动作')
        speed = profile['calibration_speeds_m_s'][('cal_slow', 'cal_medium', 'cal_fast').index(name)]
        ref = Spec()
        ref.kind, ref.x_m = 1, 1.
        ref.linear.velocity = speed
        ref.linear.acceleration = profile['calibration_acceleration_m_s2']
        ref.linear.jerk = profile['calibration_jerk_m_s3']
        ref.angular.velocity, ref.angular.acceleration, ref.angular.jerk = .7, 2., 12.
        return struct.pack('<II', 1, 10000) + bytes(ref)
    options = {'north': (1, 1.05, -.13, 0.), 'straight': (1, .10, 0., 0.), 'side': (1, 0., .08, 0.),
               'turn': (2, 0., 0., math.radians(10)),
               'diagonal': (1, .06, .06, 0.), 'arc': (4, 0., 0., 0.),
               'turn90': (2, 0., 0., math.pi / 2), 'pose': (5, .06, 0., math.radians(10))}
    if name not in options or type(action_id) is not int or not 0 < action_id < 2 ** 32:
        raise ValueError('目标必须是已定义短段且动作ID非零')
    kind, x, y, yaw = options[name]
    # 45度斜移的两轮线速为车速sqrt(2)倍，保留6RPM上限及闭环修正余量。
    velocity = .018 if name in ('diagonal', 'arc') else .03
    # 同步转姿先保留轮速叠加余量；不修改独立平移/转向用例的参数。
    linear = (.018, .05, .2) if name == 'north' else (.012, .05, .2) if name == 'pose' else (velocity, .08, .3)
    angular = (.06, .2, .6) if name == 'pose' else (.10, .3, 1.0)
    path = (.01, .01, .06, .02, 1, 0) if name == 'arc' else (0, 0, 0, 0, 0, 0)
    # 所有未使用字段清零；二进制96字节布局由M4及主机断言共同检查。
    spec = SPEC_DATA.pack(kind, x, y, yaw, *path,
                          *linear, *angular, 0, 0,
                          .012 if name == 'arc' else 0, 0, 0, 0)
    return struct.pack('<II', action_id, 90000 if name == 'north' else 20000) + spec


def trials():
    return dict(domain='commissioning', executable=False,
                frame='本次动作起点车体：X前、Y左、逆时针正',
                cases=[dict(name=n, target_m_rad=v) for n, v in (
                    ('straight', [.10, 0, 0]), ('side', [0, .08, 0]),
                    ('turn', [0, 0, math.radians(10)]), ('turn90', [0, 0, math.pi / 2]),
                    ('diagonal', [.06, .06, 0]), ('pose', [.06, 0, math.radians(10)]))]
                      + [dict(name='arc', kind='fixed_heading_arc', radius_m=.06, blend_m=.02,
                              lead_m=.01, tail_m=.01, curve_velocity_m_s=.012)],
                next_cases=['tangent_arc', 'whole_route'],
                blockers=['需核对板端GC-BASE-0.3.6与现场条件',
                          '仅开放指定小弧线和同步转姿；随弧转姿及整图仍禁止，网页不下发运动'])


class MotionLink(Link):
    def __init__(self):
        super().__init__(capabilities=5)
        self.command = None
        self.result = None
        self.execution = None
        self.execution_at = None
        self.last_execution = -1.
        self.failed = False
        self.urgent_stop = False
        self.samples = {}
        self.last_sample = -1.
        self.last_fault_query = -1.
        self.first_fault = None
        self.sample_kind = 4
        self.can_capture = CaptureReader()
        self.last_can_capture = -1.
        self.can_capture_error = None
        self.start_event = None
        self.start_event_at = None
        self.last_start_query = -1.

    def reset(self, reason):
        self.start_event = None
        self.start_event_at = None
        self.last_start_query = -1.
        # 重连只恢复观察。未决命令作未知，不排队重放，不自动使能。
        if (self.command is not None or self.execution is not None
                or (self.pending and self.pending[0].kind in (CONFIG, ARM, GOAL))):
            self.failed = True
        self.first_fault = None
        self.can_capture = CaptureReader()
        self.last_can_capture = -1.
        self.can_capture_error = None
        self.command = None
        self.execution = self.execution_at = None
        self.samples.clear()
        super().reset(reason)

    def submit(self, kind, payload=b''):
        if (not self.bound or self.failed or self.command is not None
                or (self.pending and self.pending[0].kind in (CONFIG, ARM, GOAL, START_PREPARE))
                or kind not in (CONFIG, ARM, GOAL, START_PREPARE, START_STATE)):
            raise ValueError('会话未就绪、命令未决或执行已中断')
        self.command = (kind, payload)
        self.result = None
        if kind == START_PREPARE:
            self.start_event = self.start_event_at = None

    def stop(self):
        self.command = None
        self.urgent_stop = True

    def step(self, now):
        if self.urgent_stop:
            self.urgent_stop = False
            self.request = (self.request + 1) & 0xffffffff
            packet = p.Packet(p.STOP, self.boot or 0, self.session, self.request, 250)
            # 丢弃普通等待；迟到应答不再匹配。仅STOP可以抢占。
            self.pending = (packet, now)
            return packet.encode()
        return super().step(now)

    def extra_request(self, now):
        capabilities = firmware_capabilities(self.firmware)
        if self.command:
            command = self.command
            self.command = None
            return command
        if now - self.last_execution >= .05:
            self.last_execution = now
            return STATUS, b''
        if capabilities.odom_fault and now - self.last_fault_query >= .2:
            self.last_fault_query = now
            return 37, b''
        if (capabilities.start_schema == 3 and self.execution
                and self.execution['mode'] in (0, 1) and now-self.last_start_query >= .1):
            self.last_start_query = now
            return START_STATE, b''
        if now - self.last_sample >= .05:
            self.last_sample = now
            self.sample_kind = 5 if self.sample_kind == 4 else 4
            return FEEDBACK, bytes([self.sample_kind, 0])
        # 仅新版本且执行器静止模式才读冻结页，不挤占运动期反馈；不扩大实车运动版本白名单。
        if (capabilities.can_history and self.execution
                and self.execution['mode'] in (0, 1, 6) and self.execution['can_errors']
                and not self.can_capture.result and not self.can_capture_error
                and now - self.last_can_capture >= .05):
            self.last_can_capture = now
            return CAN_CAPTURE, bytes([self.can_capture.next_page])
        return None

    def custom_reply(self, packet, request, now, sent=None):
        if request.kind == CAN_CAPTURE:
            try:
                if packet.boot != self.boot or packet.session != self.session or packet.payload[:1] != bytes([p.OK]):
                    raise ValueError('CAN分页身份或返回状态无效')
                self.can_capture.feed(packet.payload[1:], packet.boot, packet.session)
            except ValueError as exc:
                self.can_capture_error = str(exc)
            return True
        if request.kind == 37:
            # 板端复位后的启动准备期只回BUSY；只读查询按周期再问，不判为会话失效。
            if packet.boot == self.boot and packet.payload == bytes([p.BUSY]):
                return True
            if packet.boot != self.boot or len(packet.payload) != 113 or packet.payload[0] != p.OK:
                self.failed = True
                return True
            raw = struct.unpack('<24Iqq', packet.payload[1:])
            names = ('valid', 'fault', 'source', 'now_ms', 'action_id', 'kind', 'address',
                     'first_ms', 'last_ms', 'handled_ms', 'sequence', 'epoch',
                     'previous_first_ms', 'previous_last_ms', 'previous_handled_ms',
                     'previous_sequence', 'previous_epoch', 'wheel1_ms', 'wheel2_ms',
                     'wheel3_ms', 'wheel4_ms', 'yaw_ms', 'yaw_fault', 'source_epoch',
                     'value', 'previous_value')
            self.first_fault = dict(zip(names, raw), boot_id=packet.boot, session_id=packet.session)
            return True
        if request.kind == FEEDBACK:
            if packet.payload[:1] == bytes([p.OK]):
                try:
                    sample = Feedback.read(packet, request, sent, now)
                    self.samples[sample.kind] = sample
                except ValueError:
                    self.samples.clear()
            return True
        if request.kind not in (CONFIG, ARM, GOAL, STATUS, p.STOP, START_PREPARE, START_STATE):
            return False
        if packet.boot != self.boot or not packet.payload or packet.payload[0] != p.OK:
            self.failed = True
            self.result = dict(kind=request.kind, accepted=False, outcome='rejected_or_unknown')
            return True
        if request.kind != STATUS:
            if request.kind in (START_PREPARE, START_STATE):
                if len(packet.payload) not in (25, 85, 109):
                    self.failed = True
                    return True
                values = struct.unpack('<6I', packet.payload[1:25])
                if (values[0] not in (1, 2, 3) or len(packet.payload) != {1: 25, 2: 85, 3: 109}.get(values[0])
                        or values[1] > 5 or values[4] != int(values[1] == 4)):
                    self.failed = True
                    return True
                self.start_event = dict(zip(('schema', 'phase', 'ticket', 'version', 'fired', 'beep'), values))
                if values[0] >= 2:
                    names = ('reason', 'aborted_ms', 'guard_mask', 'guard_ms', 'pose_ms', 'yaw_ms',
                             'gyro_ms', 'position1_ms', 'position2_ms', 'position3_ms', 'position4_ms',
                             'speed1_ms', 'speed2_ms', 'speed3_ms', 'speed4_ms')
                    self.start_event.update(zip(names, struct.unpack('<15I', packet.payload[25:85])))
                if values[0] == 3:
                    names = ('select_level', 'select_zone', 'select_valid', 'select_epoch', 'latched_zone', 'latched_epoch')
                    self.start_event.update(zip(names, struct.unpack('<6I', packet.payload[85:])))
                    event = self.start_event
                    if (event['select_level'] not in (0, 1) or event['select_valid'] not in (0, 1)
                            or event['select_zone'] != (event['select_level'] + 1 if event['select_valid'] else 0)
                            or not event['select_epoch'] or event['latched_zone'] not in (0, 1, 2)):
                        self.failed = True
                        return True
                self.start_event_at = now
                self.last_start_query = now
            self.result = dict(kind=request.kind, accepted=True)
            return True
        if len(packet.payload) != 1 + STATUS_DATA.size:
            self.reset('执行状态长度不正确')
            self.failed = True
            return True
        raw = STATUS_DATA.unpack(packet.payload[1:])
        if raw[0] >= len(MODE) or raw[3] >= len(ACTION) or any(not math.isfinite(v) for v in raw[11:]):
            self.reset('执行状态数值无效')
            self.failed = True
            return True
        self.execution = dict(zip(('mode', 'fault', 'action_id', 'action_state', 'action_fault',
                                   'stop_confirmed', 'board_ms', 'config_version', 'can_errors',
                                   'can_rx_sequence', 'odom_valid', 'x_m', 'y_m', 'yaw_rad',
                                   'position_error_m', 'yaw_error_rad', 'fl_rpm', 'fr_rpm',
                                   'rl_rpm', 'rr_rpm'), raw))
        self.execution.update(mode_name=MODE[raw[0]], action_name=ACTION[raw[3]])
        self.execution_at = now
        return True

    def snapshot(self, now):
        state = super().snapshot(now)
        age = None if self.execution_at is None else (now - self.execution_at) * 1000
        state.update(execution=self.execution if age is not None and age < 250 else None,
                     physical_start=self.start_event if self.start_event_at is not None and now-self.start_event_at < .25 else None,
                     odom_fault_record=self.first_fault,
                     can_fault_record=self.can_capture.result, can_capture_error=self.can_capture_error,
                     can_capture_status=self.can_capture.header,
                     execution_age_ms=age, control_capable=True,
                     raw_imu={str(kind): dict(values=list(sample.values), age_ms=sample.age_ms(now),
                                             first_ms=sample.first_ms, last_ms=sample.last_ms,
                                             sequence=sample.sequence, epoch=sample.epoch,
                                             source_epoch=sample.source_epoch)
                              for kind, sample in self.samples.items() if sample.age_ms(now) < 250},
                     message='短段执行核心；网页只读，未自动下发目标')
        return state
