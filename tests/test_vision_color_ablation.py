"""开发消融的边界；不把拟合图回读包装成独立验证，不应用配置。"""

import copy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(ROOT/'pi'))
from tools.material_color_ablation import fit_development,run
from tools.replay_material_dataset import fixture_profile


class AblationTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.root=Path(self.temp.name)
        (self.root/'data').mkdir()
        frame=np.full((720,1280,3),(28,28,150),np.uint8)
        raw=cv2.imencode('.png',frame)[1].tobytes()
        self.row=dict(file='data/frame.png',sha256=hashlib.sha256(raw).hexdigest(),color_id=1,sample_roi=[100,100,140,140])
        (self.root/self.row['file']).write_bytes(raw)
        self.profile=fixture_profile()

    def tearDown(self):
        self.temp.cleanup()

    def test_fit_changes_only_color_samples_without_mutating_original(self):
        before=copy.deepcopy(self.profile)
        draft,used=fit_development(self.profile,[self.row],self.root)
        self.assertEqual(before,self.profile)
        self.assertEqual({k:v for k,v in before.items() if k!='colors'},
                         {k:v for k,v in draft.items() if k!='colors'})
        self.assertEqual(used[0]['model']['source_sha256'],self.row['sha256'])
        self.assertLessEqual(len(draft['colors']['1']['examples']),8)

    def test_profile_is_reproducible_without_runtime_timestamp(self):
        first=fit_development(self.profile,[self.row],self.root)
        second=fit_development(self.profile,[self.row],self.root)
        self.assertEqual(first,second)
        self.assertNotIn('sampled_at_unix',first[1][0]['model'])

    def test_changed_image_is_rejected(self):
        (self.root/self.row['file']).write_bytes(b'changed')
        with self.assertRaises(ValueError):
            fit_development(self.profile,[self.row],self.root)

    def test_invalid_label_or_unreviewed_roi_types_are_rejected(self):
        for changes in (dict(color_id=True),dict(color_id=7),dict(sample_roi=[1.,1,20,20])):
            with self.subTest(changes=changes),self.assertRaises(ValueError):
                fit_development(self.profile,[dict(self.row,**changes)],self.root)

    def prepare_package(self,independent=False):
        metadata=dict(name='红色_正常光_fixture',color_id=1)
        meta=self.root/'data/frame.json'
        meta.write_text(json.dumps(metadata),encoding='utf-8')
        entries=[dict(path=self.row['file'],sha256=self.row['sha256']),
                 dict(path='data/frame.json',sha256=hashlib.sha256(meta.read_bytes()).hexdigest())]
        for name in ('data_manifest.json','package_manifest.json'):
            (self.root/name).write_text(json.dumps(dict(files=entries)),encoding='utf-8')
        profile=self.root/'profile.json'
        profile.write_text(json.dumps(self.profile),encoding='utf-8')
        plan=self.root/'plan.json'
        plan.write_text(json.dumps(dict(purpose='development_ablation_only',independent_validation=independent,
                                        a_confirmation_pending=True,samples=[self.row])),encoding='utf-8')
        return plan,profile

    def test_run_has_no_device_calls_no_apply_and_no_independent_claim(self):
        plan,profile=self.prepare_package()
        profile_before=profile.read_bytes()
        with tempfile.TemporaryDirectory() as out:
            target=Path(out)/'new-output'
            with patch.object(cv2,'VideoCapture',side_effect=AssertionError('不能打开相机')):
                report=run(self.root,plan,profile,target)
            self.assertFalse(report['applied'])
            self.assertFalse(report['independent_validation'])
            self.assertFalse(report['physical_acceptance'])
            self.assertEqual(profile.read_bytes(),profile_before)
            with self.assertRaises(ValueError):
                run(self.root,plan,profile,target)

    def test_development_data_cannot_be_relabelled_as_independent_validation(self):
        plan,profile=self.prepare_package(independent=True)
        with tempfile.TemporaryDirectory() as out,self.assertRaises(ValueError):
            run(self.root,plan,profile,Path(out)/'new-output')


if __name__=='__main__':
    unittest.main()
