"""原六色得分公式的内存优化必须逐元素、逐候选等价，含拒识与歧义。"""

import copy
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'pi'))
from robot.vision.legacy.material_vision import core
from robot.vision.legacy.placement_vision.core import cleaned


def original_scores(channels,hsv,colors):
    scores=[]
    for color in range(1,7):
        model=colors[str(color)]
        distances=[]
        for sample in model.get('examples',[model]):
            distance=core.color_distance(channels,sample)
            gate=hsv[:,:,2]<=min(145,max(70,sample['v95']+20)) if color==5 else hsv[:,:,1]>=25
            distances.append(np.where(gate,distance,1e4))
        scores.append(np.minimum.reduce(distances))
    return np.stack(scores)


class ScoreEquivalenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.profile=json.loads((ROOT/'config/vision/material_profile.json').read_text(encoding='utf-8'))

    def test_real_profile_all_example_scores_match_bit_for_bit(self):
        image=np.random.default_rng(44).integers(0,256,(270,480,3),dtype=np.uint8)
        channels=np.ascontiguousarray(cv2.cvtColor(image,cv2.COLOR_BGR2LAB).astype(np.float32).transpose(2,0,1))
        hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        before=channels.copy()
        actual=core._color_scores(channels,hsv,self.profile['colors'])
        expected=original_scores(channels,hsv,self.profile['colors'])
        np.testing.assert_array_equal(actual,expected)
        np.testing.assert_array_equal(channels,before)
        self.assertEqual(actual.dtype,np.float32)

    def test_gate_boundaries_single_and_eight_example_models(self):
        for count in (1,8):
            colors=copy.deepcopy(self.profile['colors'])
            for model in colors.values():
                sample=model.get('examples',[model])[0]
                model['examples']=[copy.deepcopy(sample) for _ in range(count)]
            hsv=np.zeros((7,8,3),np.uint8)
            hsv[:,:,1]=np.array([0,24,25,26,44,45,254,255])
            hsv[:,:,2]=np.array([0,69,70,71,129,145,255])[:,None]
            channels=np.random.default_rng(6).uniform(0,255,(3,7,8)).astype(np.float32)
            np.testing.assert_array_equal(core._color_scores(channels,hsv,colors),original_scores(channels,hsv,colors))

    def test_full_detector_candidates_and_rejections_are_identical(self):
        rng=np.random.default_rng(11)
        frames=[np.zeros((720,1280,3),np.uint8),np.full((720,1280,3),220,np.uint8),
                rng.integers(0,256,(720,1280,3),dtype=np.uint8)]
        for color in range(1,7):
            frame=np.full((720,1280,3),220,np.uint8)
            cv2.ellipse(frame,(640,410),(60,50),35,0,360,core.PALETTE[color],-1)
            frames.append(frame)
        for index,frame in enumerate(frames):
            detector=core.MaterialDetector(self.profile)
            actual=cleaned(detector.detect(frame))
            rejected,uncertain=copy.deepcopy(detector.rejected_regions),copy.deepcopy(detector.uncertain_regions)
            with patch.object(core,'_color_scores',side_effect=original_scores):
                expected=cleaned(detector.detect(frame))
            with self.subTest(frame=index):
                self.assertEqual(actual,expected)
                self.assertEqual(rejected,detector.rejected_regions)
                self.assertEqual(uncertain,detector.uncertain_regions)


if __name__=='__main__':
    unittest.main()
