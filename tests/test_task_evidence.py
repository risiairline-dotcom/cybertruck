"""旧视觉观测到任务候选的离线适配；不打开相机或设备。"""

import copy
import json
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'pi'))

from robot.task_check import TaskError
from robot.vision.task_evidence import CandidateGate
from robot.vision.observe import Detector
from robot.vision.legacy.ring_vision.core import RingObserver
from robot.vision.legacy.qr_camera.task_codec import parse_task


def context(view):
    return dict(camera_id='fixture-camera', camera_config_sha256='a' * 64, image_size=[1280, 720],
                pose_id='fixture:' + view, pose_revision=1, pose_sha256='b' * 64,
                target_plane='fixture:' + view, view=view)

CODE = '136+321+631+312'


class CandidateTests(unittest.TestCase):
    def ring_frame(self):
        profile = json.loads((ROOT / 'config/vision/ring_profile.json').read_text(encoding='utf-8'))
        observer = RingObserver(profile)
        task = parse_task(CODE)
        # 来自合成候选几何，而非现场识别；真实RingObserver负责5帧稳定判断。
        candidate = dict(position_id=3, center_px=[600, 350], digit_similarity=.95,
                         digit_margin=.3, frame_clipped=False, roi_clipped=False)
        observer.process([candidate], 0, 1., 1., task, 'rough', 1, 1, [1280, 720])
        observer.begin(1.)
        for sequence in range(1, 6):
            now = 1 + sequence * .13
            observation = observer.process([candidate], sequence, now, now, task, 'rough', 1, 1, [1280, 720])
        self.assertTrue(observation['ring_target_ready'])
        return dict(domain='offline_fixture', context=context('rough:3'), observation=observation), now

    def gate(self, mode='ring_number', view='rough:3', color=None):
        return CandidateGate(context(view), CODE, 1, 1, mode, color=color, ring=3)

    def test_actual_ring_observer_adapts_stable_candidate_without_success(self):
        packet, now = self.ring_frame()
        result = self.gate().observe(packet, now)
        self.assertTrue(result['candidate_ready'])
        self.assertEqual(result['material_event'], 'UNKNOWN')
        self.assertFalse(result['confirmation_allowed'])

    def test_wrong_ring_context_task_color_and_old_frame_block_candidates(self):
        packet, now = self.ring_frame()
        variants = [('target', dict(position_id=2)), ('task_raw', '245+123+542+321'),
                    ('station', 'storage'), ('order', 2), ('batch', True),
                    ('captured_monotonic', now - .3), ('reasons', ['TARGET_UNSTABLE']),
                    ('ring_target_ready', False)]
        for key, value in variants:
            invalid = copy.deepcopy(packet)
            invalid['observation'][key] = value
            with self.subTest(key=key):
                self.assertFalse(self.gate().observe(invalid, now)['candidate_ready'])
        invalid = copy.deepcopy(packet)
        invalid['context']['pose_revision'] = True
        self.assertFalse(self.gate().observe(invalid, now)['candidate_ready'])

    def test_duplicate_frames_and_future_or_invalid_time_never_candidate(self):
        packet, now = self.ring_frame()
        gate = self.gate()
        self.assertTrue(gate.observe(packet, now)['candidate_ready'])
        self.assertFalse(gate.observe(packet, now)['candidate_ready'])
        for stamp in (float('nan'), float('inf'), True, now + 1):
            invalid = copy.deepcopy(packet)
            invalid['observation']['captured_monotonic'] = stamp
            self.assertFalse(self.gate().observe(invalid, now)['candidate_ready'])

    def test_blank_detector_output_is_not_empty_slot_confirmation(self):
        raw = Detector().process(np.full((720, 1280, 3), 220, np.uint8), 'material')
        self.assertEqual(raw['items'], [])
        packet = dict(domain='offline_fixture', context=context('raw'), observation=raw)
        result = CandidateGate(context('raw'), CODE, 1, 1, 'material_local', color=1).observe(packet, 1.)
        self.assertFalse(result['candidate_ready'])
        self.assertEqual(result['material_event'], 'UNKNOWN')

    def test_material_target_color_checked_independently_of_ready(self):
        packet = dict(domain='offline_fixture', context=context('raw'), observation=dict(
            task_raw=CODE, batch=1, order=1, frame_sequence=1, captured_monotonic=1.,
            local_target_ready=True, target=dict(color_id=1), reasons=[]))
        result = CandidateGate(context('raw'), CODE, 1, 1, 'material_local', color=1).observe(packet, 1.)
        self.assertTrue(result['candidate_ready'])
        for color in (5, True, None):
            packet['observation']['target']['color_id'] = color
            gate = CandidateGate(context('raw'), CODE, 1, 1, 'material_local', color=1)
            self.assertFalse(gate.observe(packet, 1.)['candidate_ready'])

    def test_occupancy_requires_number_color_current_scene_and_still_not_release(self):
        packet, now = self.ring_frame()
        packet['observation'].update(occupancy_observation_ready=True, reference_id='fixture-current-scene',
                                     target=dict(position_id=3, occupancy='OCCUPIED', observed_color_id=1, reasons=[]))
        result = self.gate('occupancy', color=1).observe(packet, now)
        self.assertTrue(result['candidate_ready'])
        self.assertFalse(result['confirmation_allowed'])
        for key, value in (('occupancy', 'UNKNOWN'), ('observed_color_id', 2), ('position_id', 1)):
            invalid = copy.deepcopy(packet)
            invalid['observation']['target'][key] = value
            self.assertFalse(self.gate('occupancy', color=1).observe(invalid, now)['candidate_ready'])
        packet['observation']['reference_id'] = None
        self.assertFalse(self.gate('occupancy', color=1).observe(packet, now)['candidate_ready'])

    def test_inconsistent_task_target_rejected_at_construction(self):
        with self.assertRaises(TaskError):
            CandidateGate(context('rough:2'), CODE, 1, 1, 'ring_number', ring=2)
        with self.assertRaises(TaskError):
            CandidateGate(context('raw'), CODE, 1, 1, 'material_local', color=2)


if __name__ == '__main__':
    unittest.main()
