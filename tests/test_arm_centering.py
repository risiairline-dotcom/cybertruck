"""转臂+伸缩对中：合成相机与联调组回执，不访问实际相机或电机。"""
import copy
import math
import struct
import tempfile
import unittest

from test_execution import ROOT  # noqa: F401  设置导入路径
import test_physical_mechanism as fixture
from test_centering import CAMERA, frame
from robot.device.mech_sequence import record
from robot.vision import arm_centering as ac
from robot.vision.legacy.alignment_vision.core import fingerprint

TRUE_J = [[6., -1.5], [1., 9.]]  # 每mm伸缩、每度转臂的像素响应
BASE = dict(lift=.03, reach=.04, arm=math.pi)


def observed():
    rows = [dict(order=1, axis='lift', target_m=BASE['lift'], rpm=10),
            dict(order=2, axis='reach', target_m=BASE['reach'], rpm=10),
            dict(order=3, axis='arm', target_rad=BASE['arm'], speed_rad_s=math.pi / 4)]
    return record('rough2_observe', rows, 1)


def calibration(binding, **change):
    value = dict(goal='camera_image_center', kind='arm_reach_local', schema=1, camera_signature=CAMERA,
                 observation_binding_sha256=binding, jacobian_px=TRUE_J, residual_px=0, hysteresis_px=0,
                 condition=1, samples=[], window=dict(reach_mm=8, arm_deg=4), max_step=dict(reach_mm=3, arm_deg=1.5),
                 deadband_px=3, physical_window_checked=True)
    value.update(change)
    value['sha256'] = fingerprint({k: v for k, v in value.items() if k != 'sha256'})
    return value


class Plant:
    """真实圆环偏差 error0；画面中心误差 = error0 + J·当前偏移。"""
    def __init__(self, error0):
        self.link = fixture.PhysicalMechanismTests().ready()
        self.link.firmware = 'GC-BASE-0.3.51'
        self.link.commissioning = self.link.manual_debug = True
        self.link.actual.update(zero_valid=1, attached=1, config_mask=3, base_stopped=1, stable_samples=5,
                                lift_um=round(BASE['lift'] * 1e6), reach_um=round(BASE['reach'] * 1e6))
        self.link.group = dict(id=0, state=0)
        self.error0 = error0
        self.offset = [0., 0.]
        self.now = 0.
        self.sequence = 0
        self.groups = []

    def error(self):
        return [self.error0[i] + sum(TRUE_J[i][j] * self.offset[j] for j in (0, 1)) for i in (0, 1)]

    def step(self, controller):
        self.now += .05
        link = self.link
        link.actual_at = link.base_at = link.group_at = self.now
        if link.commands:
            kind, data, request = link.commands.popleft()
            assert kind == 63
            group_id, _, mask = struct.unpack_from('<III', data, 8)
            targets = struct.unpack_from('<5i', data, 20)
            self.groups.append((mask, targets))
            self.offset = [(targets[1] - BASE['reach'] * 1e6) / 1000, (targets[3] - 180000) / 1000]
            link.actual['reach_um'] = targets[1]
            link.results.appendleft(dict(request_id=request, result=0, status='ACCEPTED'))
            link.group = dict(id=group_id, state=2)
        self.sequence += 1
        if self.sequence % 3:
            return controller.advance(link, None, self.now)  # 相机约6.7帧/秒
        return controller.advance(link, (frame(self.now, self.sequence, self.error()), self.now), self.now)

    def run(self, controller, limit=2000):
        for _ in range(limit):
            message = self.step(controller)
            if message or not controller.active:
                return message
        raise AssertionError('controller did not finish')


class MathTests(unittest.TestCase):
    def test_fit_recovers_response_and_solve_cancels_error(self):
        moves = [[2, 0], [-2, 0], [-2, 0], [2, 0], [0, 1], [0, -1], [0, -1], [0, 1]]
        samples = [(m, [sum(TRUE_J[i][j] * m[j] for j in (0, 1)) for i in (0, 1)]) for m in moves]
        jacobian, residual = ac.fit(samples)
        for i in (0, 1):
            for j in (0, 1):
                self.assertAlmostEqual(jacobian[i][j], TRUE_J[i][j])
        self.assertAlmostEqual(residual, 0)
        delta = ac.solve(TRUE_J, [12, -7])
        for i in (0, 1):
            self.assertAlmostEqual(12 * (i == 0) - 7 * (i == 1) + sum(TRUE_J[i][j] * delta[j] for j in (0, 1)), 0)
        self.assertIsNone(ac.solve([[1, 2], [2, 4]], [1, 1]))
        with self.assertRaises(ValueError):
            ac.fit([([1, 0], [1, 1]), ([2, 0], [2, 2])])

    def test_calibration_requires_confirmed_bounded_window(self):
        binding = observed()['sha256']
        ac.load_calibration(calibration(binding))
        for change in (dict(physical_window_checked=False), dict(window=dict(reach_mm=11, arm_deg=4)),
                       dict(max_step=dict(reach_mm=3, arm_deg=2)), dict(jacobian_px=[[1, 2], [2, 4.01]]),
                       dict(deadband_px=0)):
            with self.subTest(change=change), self.assertRaises(ValueError):
                ac.load_calibration(calibration(binding, **change))
        tampered = calibration(binding)
        tampered['deadband_px'] = 20
        with self.assertRaises(ValueError):
            ac.load_calibration(tampered)


class ControllerTests(unittest.TestCase):
    def start(self, mode='auto', error0=(20, -15), **args):
        plant = Plant(list(error0))
        controller = ac.ArmCentering()
        controller.set_calibration(calibration(observed()['sha256']))
        controller.start(plant.link, observed(), mode, args, plant.now)
        return plant, controller

    def test_auto_converges_inside_window_and_budget(self):
        plant, controller = self.start()
        self.assertEqual(plant.run(controller), '')
        self.assertEqual(controller.status, 'CENTERED', controller.reason)
        self.assertLessEqual(math.hypot(*plant.error()), 3)
        self.assertLessEqual(controller.steps, ac.MAX_STEPS)
        self.assertLess(plant.now, ac.AUTO_BUDGET_S)
        self.assertTrue(all(mask == 0b01010 for mask, _ in plant.groups))  # 只动伸缩和转臂，升降不参与水平修正
        offset = controller.completed['offset']
        self.assertAlmostEqual(offset['reach_mm'], plant.offset[0], places=2)  # 下发量化到1μm
    def test_wrong_direction_fails_without_chasing(self):
        plant, controller = self.start()
        controller.calibration['jacobian_px'] = [[-6., 1.5], [-1., -9.]]
        message = plant.run(controller)
        self.assertIn('误差增大', message)
        self.assertEqual(controller.status, 'FAILED')
        self.assertLessEqual(len(plant.groups), 2)

    def test_error_beyond_window_stops_before_moving_out(self):
        plant, controller = self.start(error0=(200, 0))
        message = plant.run(controller)
        self.assertIn('窗口', message)
        self.assertTrue(all(abs(t[1] - BASE['reach'] * 1e6) <= 8000 for _, t in plant.groups))

    def test_probe_fits_draft_but_never_marks_window_checked(self):
        plant, controller = self.start('probe', probe_reach_mm=2, probe_arm_deg=1)
        self.assertEqual(plant.run(controller), '')
        self.assertEqual(controller.status, 'DONE')
        draft = controller.probe
        self.assertFalse(draft['physical_window_checked'])
        self.assertEqual(len(plant.groups), 8)
        for i in (0, 1):
            for j in (0, 1):
                self.assertAlmostEqual(draft['jacobian_px'][i][j], TRUE_J[i][j], places=2)
        self.assertLess(draft['hysteresis_px'], 1e-6)

    def test_pose_mismatch_or_missing_calibration_refuses_start(self):
        plant = Plant([0, 0])
        controller = ac.ArmCentering()
        with self.assertRaises(ValueError):
            controller.start(plant.link, observed(), 'auto', {}, 0)
        controller.set_calibration(calibration(observed()['sha256']))
        plant.link.actual['reach_um'] += 600
        with self.assertRaises(ValueError):
            controller.start(plant.link, observed(), 'auto', {}, 0)
        self.assertFalse(plant.link.commands)

    def test_stale_feedback_during_move_fails_through_sequence(self):
        plant, controller = self.start()
        for _ in range(40):
            plant.step(controller)
            if controller.status == 'MOVING':
                break
        self.assertEqual(controller.status, 'MOVING')
        plant.link.commands.clear()
        message = controller.advance(plant.link, None, plant.now + .3)
        self.assertIn('过期', message)
        self.assertEqual(controller.status, 'FAILED')


class CoreTests(unittest.TestCase):
    def test_accept_writes_file_before_loading_and_stop_cancels(self):
        from robot.mech_core import DeviceCore
        with tempfile.TemporaryDirectory() as folder:
            plant, controller = ControllerTests.start(ControllerTests(), 'probe')
            plant.run(controller)
            core = DeviceCore(clock=lambda: plant.now, runtime=folder, site_ready=True)
            core.arm_centering = controller
            args = dict(confirmed=True, physical_window_checked=True, window=dict(reach_mm=8, arm_deg=4),
                        max_step=dict(reach_mm=3, arm_deg=1.5), deadband_px=3)
            with self.assertRaises(ValueError):
                core.accept_arm_calibration(dict(args, physical_window_checked=False))
            core.accept_arm_calibration(args)
            self.assertTrue(core.arm_calibration_path.exists())
            reloaded = DeviceCore(runtime=folder)
            self.assertIsNotNone(reloaded.arm_centering.calibration)
            controller.status = 'MOVING'
            core.stop_control('test', 'test', '')
            self.assertEqual(controller.status, 'CANCELLED')


if __name__ == '__main__':
    unittest.main()
