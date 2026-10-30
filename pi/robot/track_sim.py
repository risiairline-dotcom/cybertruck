"""合成车体的离线闭环。真值只由轮命令与车体模型产生，不从目标路径回填。"""

import argparse
from collections import deque
import json
import math
from pathlib import Path
import time

from .odom_sim import fixture, MASK
from .device.odometry import Odometry, Stamp
from .device.tracking import Tracker, Config, Goal, Spec, PathSpec, Limit
from .device.feedback import Feedback, FeedbackCache, FEEDBACK
from .device import protocol as p

STATES = ('IDLE', 'ACCEPTED', 'RUNNING', 'SETTLING', 'COMPLETE', 'FAILED', 'CANCELLED')
GYRO_SCALE = math.radians(2000) / 32768  # 仅此合成传感器的已知模型量程


def settings(odometry):
    config = Config()
    config.geometry = odometry.geometry
    config.kp_xy = 2
    config.kp_yaw = 3
    config.kd_gyro = .15
    config.correction_m_s = .12
    config.correction_rad_s = .7
    config.max_rpm = 100
    config.slew_rpm_s = 180
    config.position_tol_m = .005
    config.yaw_tol_rad = .025
    config.still_rpm = .5
    config.still_m_s = .006
    config.still_rad_s = .025
    config.count_drift[:] = [180] * 4
    config.fresh_ms = 250
    config.feedback_gap_ms = 100
    config.control_gap_ms = 50
    config.stable_ms = 150
    config.can_epoch = odometry.can_epoch
    config.imu_epoch = odometry.imu_epoch
    config.imu_source_epoch = odometry.imu_source_epoch
    config.calibration = odometry.calibration
    return config


def goal(kind='diagonal', action=1):
    spec = Spec()
    spec.linear = Limit(.12, .25, .8)
    spec.angular = Limit(.7, 8, 300)
    if kind in ('line', 'diagonal', 'back', 'side'):
        spec.kind = 1
        spec.x_m = -.35 if kind == 'back' else 0 if kind == 'side' else .4
        spec.y_m = .2 if kind == 'diagonal' else -.3 if kind == 'side' else 0
    elif kind in ('turn', 'right_turn', 'multi_turn'):
        spec.kind = 2
        spec.yaw_rad = -math.pi / 2 if kind == 'right_turn' else math.pi / 2 if kind == 'turn' else 2 * math.pi
    elif kind in ('arc', 'right_arc', 'reverse_arc', 'fixed_arc'):
        spec.kind = 3
        spec.path = PathSpec(.08, .08, .18, .08, -1 if kind == 'right_arc' else 1, int(kind == 'reverse_arc'))
        spec.tangent = int(kind != 'fixed_arc')
    else:
        raise ValueError('未知模拟目标')
    return Goal(action, 20000, spec)


class Plant:
    def __init__(self, stalled=False, ideal=False, radius_m=.05, k_m=.22):
        if not math.isfinite(radius_m) or not math.isfinite(k_m) or radius_m <= 0 or k_m <= 0:
            raise ValueError('模型轮径和轮距几何必须为正')
        self.radius_m, self.k_m = radius_m, k_m
        self.targets = [0.] * 4
        self.rpm = [0.] * 4
        self.turns = [0.] * 4
        self.x = self.y = self.yaw = self.wz = 0.
        self.ms = 0
        self.stalled = stalled
        self.gains = [1.] * 4 if ideal else [.96, 1.04, .98, 1.02]
        self.tau = [.08] * 4 if ideal else [.07, .09, .11, .08]
        self.history = deque(maxlen=80)
        self.positions = deque(maxlen=80)
        # 驱动器相对位置模式：按自身编码器追目标圈数，载荷差异不改变位移；新速度命令即取消。
        self.position_goal = None
        self.position_rpm = 0
        self.record()

    def move(self, revolutions, rpm):
        self.position_goal = [turn + value for turn, value in zip(self.turns, revolutions)]
        self.position_rpm = rpm

    def speed(self, targets):
        self.position_goal = None
        self.targets[:] = targets

    def record(self):
        self.history.append((self.ms, tuple(self.rpm), tuple(self.turns), self.yaw, self.wz))
        self.positions.append((self.ms, self.x, self.y, self.yaw))

    def integrate(self, disturbance=False):
        dt = .002
        for index in range(4):
            if self.position_goal is not None:
                remaining = self.position_goal[index] - self.turns[index]
                requested = max(-self.position_rpm, min(self.position_rpm, remaining * 60 / .15))
            else:
                requested = self.targets[index] * self.gains[index]
            requested = 0 if self.stalled else requested
            if disturbance and 1100 <= self.ms < 1250:
                requested += 20
            self.rpm[index] += (requested - self.rpm[index]) * (1 - math.exp(-dt / self.tau[index]))
            self.turns[index] += self.rpm[index] * dt / 60
        # 独立刚体公式；不调用跟踪器或用参考位置修正车体真值。
        travel = [value * (2 * math.pi * self.radius_m / 60) for value in self.rpm]
        vx = sum(travel) / 4
        vy = (-travel[0] + travel[1] + travel[2] - travel[3]) / 4
        self.wz = (-travel[0] + travel[1] - travel[2] + travel[3]) / (4 * self.k_m)
        mid = self.yaw + self.wz * dt / 2
        self.x += (math.cos(mid) * vx - math.sin(mid) * vy) * dt
        self.y += (math.sin(mid) * vx + math.cos(mid) * vy) * dt
        self.yaw += self.wz * dt
        self.ms += 2
        self.record()

    def sample(self, tick):
        return next(sample for sample in reversed(self.history) if sample[0] == tick)


class ClosedLoop:
    def __init__(self, library, kind='diagonal', *, config=None, stalled=False,
                 ack=True, disturbance=True, base_ms=0, ideal=False, auto_start=True, calibration=None, send_delay_ms=10):
        self.calibration = calibration if calibration is not None else fixture()
        self.native = Odometry(library, self.calibration, base_ms)
        self.config = config if config is not None else settings(self.calibration)
        self.tracker = Tracker(self.native, self.config)
        self.plant = Plant(stalled, ideal)
        self.cache = FeedbackCache(17, 19)
        self.decoder = p.Decoder()
        self.goal = goal(kind)
        self.kind = kind
        self.base_ms = base_ms
        self.ack = ack
        self.send_delay_ms = send_delay_ms
        self.disturbance = disturbance
        self.elapsed_ms = -10
        self.sequence = self.request = self.frames = 0
        self.started = False
        # 工作台点动只驱动合成车体；默认None保持原闭环测试行为。
        self.manual_rpm = None
        self.auto_start = auto_start
        self.pending = []
        self.last_output = 0
        self.position_token = 0
        self.acknowledged = set()
        self.trace = []
        self.reference_trace = []
        self.reference_end_ms = None
        self.transitions = []
        self.peak_rpm = 0
        self.drop = set()
        self.pose = self.native.snapshot(base_ms)
        self.observation = None

    def deliver(self, kind, address, values, tick, epoch, source_epoch):
        if kind in self.drop or (kind, address) in self.drop:
            return
        received = (self.base_ms + tick) & MASK
        self.sequence = (self.sequence + 1) & MASK
        raw = self.native.encode(kind, address, Stamp(received, received, (received + 2) & MASK,
                                                     self.sequence, epoch), source_epoch, values, (received + 4) & MASK)
        self.request = (self.request + 1) & MASK
        request = p.Packet(FEEDBACK, 17, 19, self.request, 250)
        response = p.Packet(FEEDBACK | 0x80, 17, 19, self.request, 250, bytes([p.OK]) + raw)
        wire = response.encode()
        packets = self.decoder.feed(wire[:9]) + self.decoder.feed(wire[9:])
        sent = 1000 + tick / 1000
        sample = Feedback.read(packets[0], request, sent, sent + .002)
        if self.cache.push(sample) and kind in (2, 4):
            self.native.push(sample, (received + 6) & MASK)
        self.frames += 1

    @staticmethod
    def append(trace, pose):
        if not trace or any(abs(pose[key] - trace[-1][key]) > 1e-8 for key in ('x_m', 'y_m', 'yaw_rad')):
            trace.append({key: pose[key] for key in ('x_m', 'y_m', 'yaw_rad')})
            if len(trace) > 300:
                trace[:] = trace[::2]

    def advance(self, report=True):
        self.elapsed_ms += 10
        tick = self.elapsed_ms
        now = (self.base_ms + tick + 6) & MASK
        while self.plant.ms < tick + 6:
            due = [item for item in self.pending if item[0] <= self.plant.ms]
            self.pending = [item for item in self.pending if item[0] > self.plant.ms]
            for _, targets, action, token in due:
                if isinstance(targets, dict):
                    self.plant.move(targets['micro_rev'], targets['micro_rpm'])
                else:
                    self.plant.speed(targets)
                if token and self.ack:
                    if self.tracker.sent(action, token, (self.base_ms + self.plant.ms) & MASK):
                        self.acknowledged.add((action, token))
            self.plant.integrate(self.disturbance)
        # 当前控制在t+6ms，观测取独立车体历史t，完整保留传输延期。
        _, rpm, turns, yaw, wz = self.plant.sample(tick)
        raw_yaw = ((round(yaw / self.calibration.yaw_scale_rad) + 32768) % 65536) - 32768
        raw_gyro = round(wz / GYRO_SCALE)
        self.deliver(4, 0, (0, 0, raw_yaw), tick, 5, 7)
        self.deliver(5, 0, (0, 0, raw_gyro), tick, 5, 7)
        index = (tick // 10) % 4
        address = self.calibration.address[index]
        sign = self.calibration.sign[index]
        count = round(turns[index] * self.calibration.counts_per_rev[index] * sign)
        self.deliver(2, address, (count, 0, 0), tick, 3, 3)
        self.deliver(1, address, (round(rpm[index] * sign), 0, 0), tick, 3, 3)
        self.native.step(now)
        self.pose = self.native.snapshot(now)
        self.observation = self.tracker.observation(self.cache, now, GYRO_SCALE)
        if self.auto_start and not self.started and tick >= 120:
            result = self.tracker.start(self.goal, self.observation, now)
            if result != 1:
                raise RuntimeError(f'模拟目标未被接受：{result}')
            self.started = True
        if self.started:
            self.tracker.step(self.observation, now)
        output = self.tracker.output()
        self.peak_rpm = max(self.peak_rpm, *map(abs, output['rpm']))
        if not self.transitions or self.transitions[-1]['state'] != STATES[output['state']]:
            self.transitions.append(dict(at_ms=tick, state=STATES[output['state']]))
        if output['reference']['done'] and self.reference_end_ms is None:
            self.reference_end_ms = tick
        if self.manual_rpm is not None:
            self.pending.clear()
            rate = self.config.slew_rpm_s * .01
            self.plant.speed([old + min(rate, max(-rate, new - old))
                              for old, new in zip(self.plant.targets, self.manual_rpm)])
        elif output['state'] in (5, 6):
            self.pending.clear()
            self.plant.speed([0] * 4)
        elif output['micro_hold']:
            # 同一位置票据只下发一次；执行期间不发速度命令。
            token = self.tracker.token()
            if token != self.position_token:
                self.position_token = token
                self.pending.append((tick + 6 + self.send_delay_ms, dict(micro_rev=output['micro_rev'],
                                     micro_rpm=output['micro_rpm']), output['id'], token))
        elif output['output_id'] != self.last_output:
            self.last_output = output['output_id']
            self.pending.append((tick + 6 + self.send_delay_ms, list(output['rpm']), output['id'], self.tracker.token()))
        if self.started and self.pose['valid']:
            self.append(self.trace, self.pose)
            self.append(self.reference_trace, output['reference'])
        return self.snapshot() if report else None

    def begin(self, target):
        """多段模拟共用同一个里程计/车体；完成前不能接受下一动作。"""
        if self.observation is None:
            raise ValueError('尚无模拟反馈')
        result = self.tracker.start(target, self.observation, (self.base_ms + self.elapsed_ms + 6) & MASK)
        if result != 1:
            raise ValueError(f'下一段未被接受：{result}')
        self.goal, self.started = target, True
        self.reference_end_ms = None

    def snapshot(self):
        output = self.tracker.output()
        return dict(domain='simulation', kind=self.kind, state=STATES[output['state']],
                    output=output, pose=dict(self.pose), elapsed_ms=self.elapsed_ms,
                    reference_end_ms=self.reference_end_ms, frames=self.frames,
                    peak_rpm=self.peak_rpm, trace=list(self.trace), reference_trace=list(self.reference_trace),
                    transitions=list(self.transitions), physical_pose_valid=False,
                    plant=dict(x_m=self.plant.x, y_m=self.plant.y, yaw_rad=self.plant.yaw,
                               rpm=list(self.plant.rpm)),
                    source='合成车体含电机滞后/载荷差异，非本车参数')

    def run(self, limit_ms=22000):
        while self.elapsed_ms < limit_ms:
            result = self.advance()
            if result['state'] in ('COMPLETE', 'FAILED', 'CANCELLED'):
                return result
        return self.snapshot()


class TrackDemo:
    def __init__(self, library):
        self.model = ClosedLoop(library)
        self.started = time.monotonic()

    def step(self, now):
        target = int((now - self.started) * 1000 / 10) * 10
        for _ in range(16):
            if self.model.elapsed_ms >= target or self.model.snapshot()['state'] in ('COMPLETE', 'FAILED', 'CANCELLED'):
                break
            self.model.advance()

    def snapshot(self):
        return self.model.snapshot()


def main():
    parser = argparse.ArgumentParser(description='仅离线闭环，禁止连接设备')
    parser.add_argument('--library', type=Path, required=True)
    parser.add_argument('--kind', default='diagonal')
    parser.add_argument('--output', type=Path, default=Path('output/tracking/report.json'))
    args = parser.parse_args()
    model = ClosedLoop(args.library, args.kind)
    report = model.run()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({key: value for key, value in report.items() if key not in ('trace', 'reference_trace')}, ensure_ascii=False))


if __name__ == '__main__':
    main()
