"""工作台离线执行适配器：共用正式C跟踪器、里程计及独立车体模型。"""

import math
import uuid

from ..device.tracking import Goal, Spec, Limit, Tracker
from ..nav.frames import Pose, compose, relative, angle
from ..nav.field import Field
from ..nav.reference import NativeReference
from ..track_sim import ClosedLoop, settings
from ..odom_sim import fixture
from .scene import start_zones


class Simulation:
    def __init__(self, library):
        self.library = library
        self.reset('start_1')

    def reset(self, start, max_rpm=100., wheel_slew_rpm_s=180., lag_ff_s=0.):
        calibration = fixture()
        calibration.max_rpm = max(100., max_rpm * 1.2)
        config = settings(calibration)
        config.max_rpm = max_rpm
        config.slew_rpm_s = wheel_slew_rpm_s
        config.lag_ff_s = lag_ff_s
        config.position_tol_m, config.yaw_tol_rad = .003, .012
        self.model = ClosedLoop(self.library, config=config, auto_start=False, disturbance=False,
                                calibration=calibration)
        field = Field().data
        self.origin = Pose(*field['nodes'][start], field['station_heading_quarters'][start]*math.pi/2)
        self.session = uuid.uuid4().hex
        self.mode, self.reason = 'IDLE', ''
        self.failure_reason = ''
        self.manual = [0.] * 4
        self.plan, self.index, self.next_id = None, 0, 1
        self.results, self.trace = [], []
        self.run_ms = 0
        self.end_ms = None
        self.move_deadline = 0
        self.field = None
        self.truth_ms = -1
        for _ in range(16):
            self.model.advance(report=False)
        self.run_ms = self.end_ms = self.model.elapsed_ms

    def stopped(self):
        return max(map(abs, self.model.plant.rpm)) < .5 and self.mode not in ('RUNNING', 'MANUAL')

    def halt(self, reason):
        if self.mode in ('RUNNING', 'MANUAL') or self.end_ms is None:
            self.end_ms = self.model.elapsed_ms
        self.model.tracker.cancel()
        self.model.pending.clear()
        self.model.manual_rpm = None
        self.model.plant.speed([0.] * 4)
        self.manual[:] = [0.] * 4
        self.mode = 'FAILED' if self.failure_reason else 'STOPPING'
        self.reason = self.failure_reason or reason

    def fail(self, reason):
        self.failure_reason = reason
        self.halt(reason)

    def jog(self, velocity):
        if self.mode != 'MANUAL':
            self.run_ms = self.model.elapsed_ms
        self.manual = list(self.model.native.inverse(*velocity))
        scale = min(1, 40 / max(1, *map(abs, self.manual)))
        self.manual = [v * scale for v in self.manual]
        self.mode = 'MANUAL'
        self.end_ms = None

    def _begin(self, spec, seconds):
        # 取消后的新动作重新初始化跟踪器，不重建里程计和车体。
        if self.model.tracker.output()['state'] == 6:
            self.model.tracker = Tracker(self.model.native, self.model.config)
            self.model.last_output = 0
        self.model.begin(Goal(self.next_id, max(20000, math.ceil((seconds + 15) * 1000)), spec))
        self.next_id += 1
        self.move_deadline = self.model.elapsed_ms + math.ceil((seconds + 15) * 1000)
        self.mode, self.reason = 'RUNNING', ''
        self.end_ms = None

    def move(self, command):
        spec = Spec()
        spec.kind = 5
        spec.x_m, spec.y_m, spec.yaw_rad = command.x_m, command.y_m, command.yaw_rad
        spec.linear, spec.angular = Limit(command.speed_m_s, .15, .6), Limit(.3, .8, 3)
        current = Pose(self.model.pose['x_m'], self.model.pose['y_m'], self.model.pose['yaw_rad'])
        ref = NativeReference(self.model.native.lib, current, spec)
        self.plan, self.results, self.index = None, [], 0
        self.field = None  # 定量校准不沿用上次路线的场景/计划指针。
        self.run_ms = self.model.elapsed_ms
        self._begin(spec, ref.duration_s)

    def run(self, plan):
        # 开始模拟时建立独立运行原点，与地图配准无关。
        self.reset(plan.view['parameters']['start'], plan.view['parameters']['max_rpm'],
                   plan.view['parameters'].get('dynamics', {}).get('wheel_slew_rpm_s', 180.),
                   plan.view['parameters'].get('dynamics', {}).get('lag_ff_s', 0.))
        self.field = Field(data=plan.view['field'])
        self.origin = Pose(*plan.view['origin'])
        self.plan = plan
        self.run_ms = self.model.elapsed_ms
        self._leg()

    def _leg(self):
        spec = Spec.from_buffer_copy(self.plan.specs[self.index])
        planned = self.origin
        for raw in self.plan.specs[:self.index]:
            planned = NativeReference(self.model.native.lib, planned, Spec.from_buffer_copy(raw)).end
        desired = NativeReference(self.model.native.lib, planned, spec).end
        current = compose(self.origin, Pose(self.model.pose['x_m'], self.model.pose['y_m'],
                                           self.model.pose['yaw_rad']))
        if spec.kind in (1, 5, 6):
            local = relative(current, desired)
            spec.x_m, spec.y_m = local.x_m, local.y_m
        if spec.kind in (2, 4, 5, 6):
            spec.yaw_rad += angle(desired.yaw_rad - current.yaw_rad - spec.yaw_rad)
        if spec.kind == 4:
            spec.path_heading_rad = angle(planned.yaw_rad + spec.path_heading_rad - current.yaw_rad)
        self._begin(spec, self.plan.view['legs'][self.index]['seconds'])

    def step(self):
        self.model.manual_rpm = self.manual if self.mode == 'MANUAL' else None
        self.model.advance(report=False)
        if self.mode == 'MANUAL':
            if not self.model.pose['valid']:
                self.fail('点动模型反馈失效')
        elif self.mode in ('STOPPING', 'FAILED'):
            self.model.pending.clear()
            self.model.plant.speed([0.] * 4)
            if self.mode == 'STOPPING' and max(map(abs, self.model.plant.rpm)) < .5:
                self.mode = 'STOPPED'
        elif self.mode == 'RUNNING':
            if not self._check_truth():
                return
            out = self.model.tracker.output()
            if out['state'] in (5, 6) or self.model.elapsed_ms >= self.move_deadline:
                self.fail(f'模型故障或超时：{out["fault"]}')
            elif out['state'] == 4:
                self.results.append(dict(action_id=out['id'], position_error_m=out['position_error_m'],
                                         yaw_error_rad=out['yaw_error_rad'],
                                         elapsed_ms=self.model.elapsed_ms - self.run_ms))
                self.index += 1
                if self.plan and self.index < len(self.plan.specs):
                    self._leg()
                else:
                    self.mode = 'COMPLETE'
                    self.end_ms = self.model.elapsed_ms
        if self.model.elapsed_ms % 100 == 0:
            point = self.pose()
            if not self.trace or math.dist(point[:2], self.trace[-1][:2]) > .0005 or abs(point[2] - self.trace[-1][2]) > .002:
                self.trace.append(point)
                if len(self.trace) > 2000:
                    self.trace[:] = self.trace[::2]

    def _check_truth(self):
        """检查独立Plant的2ms扫掠，不以规划参考代替模拟实际位置。"""
        if self.field is None:
            return True
        speed = max(abs(rpm) for sample in self.model.plant.history for rpm in sample[1]) * 2 * math.pi * .05 / 60
        movement = (math.sqrt(2) * speed + (self.field.radius + .04) * speed / .22) * .002
        for tick, x, y, yaw in self.model.plant.positions:
            if tick <= self.truth_ms:
                continue
            pose = compose(self.origin, Pose(x, y, yaw))
            errors = self.field.errors(pose, start_zones(self.plan.view['parameters']), movement, tracking=False)
            self.truth_ms = tick
            if errors:
                self.fail('模型扫掠碰撞：' + ','.join(errors))
                return False
        return True

    def pose(self):
        return compose(self.origin, Pose(self.model.pose['x_m'], self.model.pose['y_m'],
                                         self.model.pose['yaw_rad'])).values()

    def view(self):
        out = self.model.tracker.output()
        return dict(domain='simulation', physical_pose_valid=False, session=self.session,
                    mode=self.mode, reason=self.reason, pose=self.pose(), trace=list(self.trace),
                    failure_reason=self.failure_reason,
                    rpm=list(self.model.plant.rpm), target_rpm=list(self.model.plant.targets),
                    stopped=self.stopped(), completed=len(self.results),
                    elapsed_s=max(0, ((self.end_ms if self.end_ms is not None else self.model.elapsed_ms) - self.run_ms) / 1000),
                    action_id=out['id'], fault=out['fault'],
                    active_plan_id=self.plan.view['id'] if self.plan else None,
                    total=len(self.plan.specs) if self.plan else (1 if self.next_id > 1 else 0),
                    position_error_m=out['position_error_m'], yaw_error_rad=out['yaw_error_rad'],
                    odom_origin_ms=self.model.pose['origin_ms'], results=list(self.results))
