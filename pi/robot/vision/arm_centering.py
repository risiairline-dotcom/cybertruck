"""底盘停车后用伸缩+PE11转臂把2号圆环移到画面中心；相机随臂运动，执行复用联调组。

局部模型 e_px ≈ J·[Δ伸缩mm, Δ转臂deg]，J 由同一观察姿态的正反小幅探测拟合，
只在用户确认无碰撞的窗口内使用，不作为通用逆运动学。
"""
import copy
import math

from ..device.mech_sequence import Sequence, digest, groups, record
from .centering import FrameNotFresh, RingWatch, camera_signature
from .legacy.alignment_vision.core import fingerprint

# 代码硬上限；标定文件只能在此范围内收窄。
MAX_STEP = dict(reach_mm=3., arm_deg=1.5)
MAX_WINDOW = dict(reach_mm=10., arm_deg=5.)
MAX_STEPS = 6
AUTO_BUDGET_S = 10.  # 比赛停止运行15秒即结束，留出抓放时间。
PROBE_BUDGET_S = 40.
SETTLE_LIMIT_S = 3.
POSE_TOL_UM = 500


def _offsets_valid(value, limits, positive=False):
    return (isinstance(value, dict) and set(value) == set(limits) and
            all(type(value[k]) in (int, float) and math.isfinite(value[k]) and
                (0 < value[k] if positive else 0 <= value[k]) and value[k] <= limits[k] for k in limits))


def solve(jacobian, error_px):
    """返回使 J·Δu = -e 的 Δu=[mm, deg]；矩阵奇异时返回 None。"""
    (a, b), (c, d) = jacobian
    determinant = a * d - b * c
    if not math.isfinite(determinant) or abs(determinant) < 1e-9:
        return None
    ex, ey = error_px
    return [(-d * ex + b * ey) / determinant, (c * ex - a * ey) / determinant]


def condition(jacobian):
    """2×2 矩阵二范数条件数。"""
    (a, b), (c, d) = jacobian
    s = a * a + b * b + c * c + d * d
    det = abs(a * d - b * c)
    root = math.sqrt(max(0., s * s / 4 - det * det))
    small = math.sqrt(max(0., s / 2 - root))
    return math.inf if small < 1e-12 else math.sqrt(s / 2 + root) / small


def fit(samples):
    """最小二乘拟合 Δpx = J·Δu；samples 为 [(Δu, Δpx)]，返回 (J, 残差RMS像素)。"""
    suu = [[0., 0.], [0., 0.]]
    spu = [[0., 0.], [0., 0.]]
    for du, dp in samples:
        for i in (0, 1):
            for j in (0, 1):
                suu[i][j] += du[i] * du[j]
                spu[i][j] += dp[i] * du[j]
    (a, b), (c, d) = suu
    det = a * d - b * c
    if abs(det) < 1e-9:
        raise ValueError('探测动作不足以区分伸缩和转臂')
    inverse = [[d / det, -b / det], [-c / det, a / det]]
    jacobian = [[sum(spu[i][k] * inverse[k][j] for k in (0, 1)) for j in (0, 1)] for i in (0, 1)]
    squares = [sum((dp[i] - sum(jacobian[i][j] * du[j] for j in (0, 1))) ** 2 for i in (0, 1))
               for du, dp in samples]
    return jacobian, math.sqrt(sum(squares) / len(squares))


def load_calibration(value):
    """校验已人工确认的局部标定；未确认窗口无碰撞时拒绝。"""
    if (not isinstance(value, dict) or value.get('goal') != 'camera_image_center' or
            value.get('kind') != 'arm_reach_local' or value.get('schema') != 1 or
            value.get('sha256') != fingerprint({k: v for k, v in value.items() if k != 'sha256'})):
        raise ValueError('转臂对中标定缺少校验或类型不符')
    camera_signature(value.get('camera_signature') or {})
    binding = value.get('observation_binding_sha256')
    if not isinstance(binding, str) or len(binding) != 64:
        raise ValueError('转臂对中标定缺少观察姿态摘要')
    jacobian = value.get('jacobian_px')
    if (not isinstance(jacobian, list) or len(jacobian) != 2 or
            any(not isinstance(row, list) or len(row) != 2 or
                any(type(v) not in (int, float) or not math.isfinite(v) for v in row) for row in jacobian)):
        raise ValueError('转臂对中响应矩阵无效')
    if condition(jacobian) > 20:
        raise ValueError('伸缩与转臂在画面中的响应接近共线，不能稳定求解')
    if not _offsets_valid(value.get('window'), MAX_WINDOW, True) or not _offsets_valid(value.get('max_step'), MAX_STEP, True):
        raise ValueError('转臂对中窗口或单步上限超出代码硬上限')
    deadband = value.get('deadband_px')
    if type(deadband) not in (int, float) or not 1 <= deadband <= 30:
        raise ValueError('画面中心容差需为1～30像素')
    if value.get('physical_window_checked') is not True:
        raise ValueError('尚未实物确认该窗口内伸缩和转臂无碰撞')
    return copy.deepcopy(value)


def observation_pose(value):
    """观察联调表执行完毕后的各执行器目标与速度；必须包含伸缩和转臂。"""
    if digest(value) != value.get('sha256'):
        raise ValueError('观察姿态版本SHA不匹配')
    final = {}
    for stage in groups(value['rows']):
        for member in stage['members']:
            final[member['axis']] = member
    if not {'lift', 'reach', 'arm'} <= set(final):
        raise ValueError('观察姿态需明确包含升降、伸缩和转臂目标')
    return final


class ArmCentering:
    def __init__(self):
        self.status = 'IDLE'
        self.mode = 'auto'
        self.reason = ''
        self.calibration = None
        self.calibration_error = '尚未导入已确认的转臂对中标定'
        self.watch = RingWatch()
        self.runner = Sequence()
        self.pose = None
        self.binding = None
        self.offset = [0., 0.]
        self.plan = []
        self.samples = []
        self.steps = 0
        self.serial = 0
        self.started = self.ready_at = 0.
        self.previous_radius = None
        self.not_improving = 0
        self.last_observation = None
        self.probe = None
        self.completed = None

    @property
    def active(self):
        return self.status in ('MOVING', 'OBSERVING')

    def set_calibration(self, value):
        self.calibration = load_calibration(value)
        self.calibration_error = ''

    def view(self):
        return dict(status=self.status, mode=self.mode, reason=self.reason,
                    offset=dict(reach_mm=round(self.offset[0], 3), arm_deg=round(self.offset[1], 3)),
                    steps=self.steps, observation=self.last_observation, probe=self.probe,
                    completed=self.completed, calibration_available=self.calibration is not None,
                    calibration_error=self.calibration_error, sequence=self.runner.view())

    def start(self, link, observed, mode, args, now):
        """observed 为刚执行完毕的观察联调表；底盘保持停车，不交接底盘执行权。"""
        if self.active:
            raise ValueError('转臂对中正在执行')
        if mode not in ('auto', 'probe'):
            raise ValueError('转臂对中模式无效')
        pose = observation_pose(observed)
        for axis, field in (('lift', 'lift_um'), ('reach', 'reach_um')):
            if abs(link.actual.get(field, -10**9) - round(pose[axis]['target_m'] * 1e6)) > POSE_TOL_UM:
                raise ValueError('升降/伸缩实际位置与观察姿态不符，请先执行观察联调表')
        if mode == 'auto':
            if self.calibration is None:
                raise ValueError(self.calibration_error)
            if self.calibration['observation_binding_sha256'] != observed['sha256']:
                raise ValueError('当前观察姿态与转臂对中标定不一致')
            plan = []
        else:
            reach, arm = args.get('probe_reach_mm', 2.), args.get('probe_arm_deg', 1.)
            if not _offsets_valid(dict(reach_mm=reach, arm_deg=arm), MAX_STEP, True):
                raise ValueError('探测幅度超出单步上限')
            plan = [[reach, 0.], [0., 0.], [-reach, 0.], [0., 0.], [0., arm], [0., 0.], [0., -arm], [0., 0.]]
        self.mode = mode
        self.pose = pose
        self.binding = observed['sha256']
        self.watch.reset(self.binding)
        self.watch.fence(now)
        self.offset = [0., 0.]
        self.plan = plan
        self.samples = []
        self.steps = 0
        self.started = self.ready_at = now
        self.previous_radius = None
        self.not_improving = 0
        self.last_observation = self.probe = self.completed = None
        self.status = 'OBSERVING'
        self.reason = ''

    def cancel(self, reason):
        if self.active:
            self.status = 'CANCELLED'
            self.reason = reason
        self.runner.cancel(reason)

    def _fail(self, reason):
        self.status = 'FAILED'
        self.reason = reason
        return reason

    def _move(self, link, offset, now):
        window = self.calibration['window'] if self.mode == 'auto' else MAX_WINDOW
        if abs(offset[0]) > window['reach_mm'] + 1e-9 or abs(offset[1]) > window['arm_deg'] + 1e-9:
            return self._fail('修正超出已确认窗口，保持当前姿态')
        reach, arm = self.pose['reach'], self.pose['arm']
        rows = [dict(order=1, axis='reach', target_m=reach['target_m'] + offset[0] / 1000, rpm=reach.get('rpm', 5)),
                dict(order=1, axis='arm', target_rad=arm['target_rad'] + math.radians(offset[1]),
                     speed_rad_s=arm.get('speed_rad_s', math.pi / 4))]
        self.serial += 1
        try:
            self.runner.start(record('arm_centering', rows, self.serial), 'together', link, now)
        except ValueError as exc:
            return self._fail(str(exc))
        self.offset = list(offset)
        self.status = 'MOVING'
        return ''

    def _probe_step(self, link, o, now):
        index = len(self.samples)
        self.samples.append(dict(offset=list(self.offset), center_px=list(o['center_px'])))
        if index < len(self.plan):
            return self._move(link, self.plan[index], now)
        pairs = [([b['offset'][i] - a['offset'][i] for i in (0, 1)],
                  [b['center_px'][i] - a['center_px'][i] for i in (0, 1)])
                 for a, b in zip(self.samples, self.samples[1:])]
        homes = [s['center_px'] for s in self.samples if s['offset'] == [0., 0.]]
        hysteresis = max(math.dist(homes[0], h) for h in homes)
        try:
            jacobian, residual = fit(pairs)
        except ValueError as exc:
            return self._fail(str(exc))
        draft = dict(goal='camera_image_center', kind='arm_reach_local', schema=1,
                     camera_signature=copy.deepcopy(self.watch.camera), observation_binding_sha256=self.binding,
                     jacobian_px=[[round(v, 6) for v in row] for row in jacobian],
                     residual_px=round(residual, 3), hysteresis_px=round(hysteresis, 3),
                     condition=round(condition(jacobian), 3), samples=copy.deepcopy(self.samples),
                     boot_id=str(link.boot), zero_version=link.actual.get('zero_version'),
                     physical_window_checked=False)
        self.probe = draft
        self.status = 'DONE'
        self.reason = '探测完成；核对残差、回差和窗口无碰撞后再确认导入'
        return ''

    def _auto_step(self, link, o, now):
        data = self.calibration
        error = [o['center_px'][i] - o['image_center_px'][i] for i in (0, 1)]
        radius = math.hypot(*error)
        if radius <= data['deadband_px'] and o['window_max_error_px'] <= data['deadband_px']:
            self.status = 'CENTERED'
            self.completed = dict(offset=dict(reach_mm=self.offset[0], arm_deg=self.offset[1]),
                                  observation=copy.deepcopy(o), binding_sha256=self.binding,
                                  boot_id=str(link.boot), zero_version=link.actual.get('zero_version'),
                                  physical_validated=False)
            self.reason = '2号圆环已进入画面中心容差'
            return ''
        if radius <= data['deadband_px']:
            return ''  # 整个稳定窗口都在容差内才算完成，不挑最后一帧。
        if self.previous_radius is not None:
            if radius > self.previous_radius * 1.6 + 2:
                return self._fail('误差增大，检查响应矩阵方向；未继续追赶')
            self.not_improving = self.not_improving + 1 if radius >= self.previous_radius - .25 else 0
            if self.not_improving >= 2:
                return self._fail('连续两步未收敛，保持当前姿态')
        if self.steps >= MAX_STEPS:
            return self._fail(f'达到{MAX_STEPS}步上限仍未进入容差')
        delta = solve(data['jacobian_px'], error)
        if delta is None:
            return self._fail('响应矩阵奇异，不能求解修正')
        limits = data['max_step']
        scale = min(1., limits['reach_mm'] / max(abs(delta[0]), 1e-12), limits['arm_deg'] / max(abs(delta[1]), 1e-12))
        self.previous_radius = radius
        self.steps += 1
        return self._move(link, [self.offset[i] + delta[i] * scale for i in (0, 1)], now)

    def advance(self, link, snapshot, now):
        """由唯一设备核心周期调用；返回非空错误时调用方统一STOP。"""
        if not self.active:
            return ''
        budget = AUTO_BUDGET_S if self.mode == 'auto' else PROBE_BUDGET_S
        if now - self.started > budget:
            return self._fail(f'转臂对中超过{budget:g}秒预算，保持当前姿态')
        if self.status == 'MOVING':
            error = self.runner.advance(link, now)
            if error:
                return self._fail(error)
            if self.runner.status != 'DONE':
                return ''
            self.watch.fence(now)
            self.ready_at = now
            self.status = 'OBSERVING'
            return ''
        if not link.base.get('stopped') or link.actual.get('fault') or link.base.get('fault'):
            return self._fail('底盘未停车或出现故障，转臂对中停止')
        if now - self.ready_at > SETTLE_LIMIT_S:
            return self._fail('动作结束后3秒内没有稳定的2号圆环新帧')
        if not snapshot:
            return ''
        value, received = snapshot
        expected = self.calibration['camera_signature'] if self.mode == 'auto' else None
        try:
            o = self.watch.observe(value, received, now, expected)
        except FrameNotFresh:
            return ''
        except (ValueError, TypeError, KeyError) as exc:
            return self._fail(str(exc))
        if o is None:
            return ''
        self.last_observation = o
        if not o['stable']:
            return ''
        return self._probe_step(link, o, now) if self.mode == 'probe' else self._auto_step(link, o, now)
