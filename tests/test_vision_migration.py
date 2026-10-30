"""迁入观测、旧帧拒绝及Web隔离；不打开实物设备。"""

import json
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pi'))

import numpy as np
from fastapi import FastAPI
from fastapi.testclient import TestClient
from robot.vision.observe import Detector
from robot.vision.camera import Capture
from robot.vision.camera import create_app as camera_app
from robot.vision.legacy.vision_runtime.engine import FrameEngine
from robot.vision.legacy.qr_camera.task_codec import parse_task
from robot.web.vision import router


class VisionTests(unittest.TestCase):
    def test_planar_color_distance_matches_legacy_norm(self):
        from robot.vision.legacy.material_vision.core import color_distance
        lab = np.random.default_rng(42).uniform(0, 255, (270, 480, 3)).astype(np.float32)
        profiles = Detector().profiles['material_profile']['colors']
        for model in profiles.values():
            for sample in model.get('examples', [model]):
                expected = np.linalg.norm((lab-np.float32(sample['lab']))/np.float32(sample['scale']), axis=2)
                actual = color_distance(np.ascontiguousarray(lab.transpose(2, 0, 1)), sample)
                np.testing.assert_array_equal(actual, expected)

    def test_optimized_detector_preserves_replay_results(self):
        import cv2
        from robot.vision.legacy.material_vision import core
        def legacy(channels, sample):
            lab = channels.transpose(1, 2, 0)
            return np.linalg.norm((lab-np.float32(sample['lab']))/np.float32(sample['scale']), axis=2)
        frames = [np.random.default_rng(4).integers(0, 256, (720,1280,3), dtype=np.uint8)]
        # 本机有历史样本时一并对照；样本不入Git。
        index = ROOT / 'output/vision/samples.json'
        for item in json.loads(index.read_text(encoding='utf-8')) if index.is_file() else []:
            path = ROOT / 'output/vision/samples' / item['file']
            frames.append(cv2.imdecode(np.frombuffer(path.read_bytes(), dtype=np.uint8), cv2.IMREAD_COLOR))
        for frame in frames:
            actual = Detector().process(frame, 'material')
            with patch.object(core, 'color_distance', side_effect=legacy):
                expected = Detector().process(frame, 'material')
            self.assertEqual(actual, expected)

    def test_blank_is_candidate_empty_not_success(self):
        result = Detector().process(np.full((720, 1280, 3), 220, np.uint8), 'material')
        self.assertEqual(result['items'], [])
        self.assertFalse(result['metric_valid'])
        self.assertFalse(result['motion_authority'])

    def test_expired_capture_never_returns_image(self):
        capture = Capture('/not-opened', [1280, 720])
        capture.latest = (1, time.monotonic() - 1, np.zeros((1, 1, 3), np.uint8))
        with self.assertRaises(ValueError):
            capture.read()

    def test_historical_reference_cannot_implicitly_approve(self):
        detector = Detector()
        engine = FrameEngine(*detector.profiles.values(), 'actual_mount_20261020')
        reference = json.loads((ROOT / 'config/vision/historical_reference.json').read_text(encoding='utf-8'))
        with self.assertRaisesRegex(ValueError, 'UNAPPROVED'):
            engine.prepare('alignment_pixel', parse_task('136+321+631+312'), 'rough', 1, 2, 100,
                           reference=reference)

    def test_occupancy_requires_current_scene(self):
        detector = Detector()
        engine = FrameEngine(*detector.profiles.values(), 'actual_mount_20261020')
        with self.assertRaisesRegex(ValueError, 'FRESH_SESSION'):
            engine.prepare('occupancy', parse_task('136+321+631+312'), 'rough', 1, 1, 100,
                           empty_reference={})

    def test_web_replay_and_fixed_proxy(self):
        app = FastAPI()
        app.include_router(router())
        client = TestClient(app)
        self.assertEqual(client.get('/api/vision/replay/not-a-file').status_code, 404)
        self.assertEqual(client.get('/api/vision/live/stop').status_code, 400)
        self.assertEqual(client.post('/api/vision/live/observation').status_code, 405)
        samples = ROOT / 'output/vision/samples.json'
        if samples.exists():
            sample = json.loads(samples.read_text(encoding='utf-8'))[0]
            result = client.get('/api/vision/replay/' + sample['id']).json()
            self.assertEqual(result['domain'], 'historical_replay')
            self.assertFalse(result['motion_authority'])
            self.assertEqual(client.get(result['image']).headers['content-type'], 'image/jpeg')

    def test_camera_response_binds_image_and_frame_without_opening_device(self):
        client = TestClient(camera_app())
        image = np.full((720, 1280, 3), 220, np.uint8)
        with patch.object(Capture, 'read', return_value=(17, time.monotonic(), image)):
            result = client.get('/observation?mode=material').json()
        self.assertEqual(result['frame_id'], 17)
        self.assertEqual(result['domain'], 'camera_live')
        self.assertTrue(result['image'].startswith('data:image/jpeg;base64,'))
        self.assertFalse(result['motion_authority'])
        with patch.object(Capture, 'read', side_effect=ValueError('旧帧')):
            self.assertEqual(client.get('/observation').status_code, 503)

    def test_four_group_qr_image_uses_old_parser(self):
        import cv2
        encoder = cv2.QRCodeEncoder_create()
        code = encoder.encode('136+321+631+312')
        code = cv2.copyMakeBorder(code, 8, 8, 8, 8, cv2.BORDER_CONSTANT, value=255)
        frame = cv2.cvtColor(cv2.resize(code, None, fx=10, fy=10, interpolation=cv2.INTER_NEAREST), cv2.COLOR_GRAY2BGR)
        result = Detector().process(frame, 'qr')
        self.assertTrue(result['items'][0]['valid'])
        self.assertEqual(result['items'][0]['task']['raw'], '136+321+631+312')


if __name__ == '__main__':
    unittest.main()
