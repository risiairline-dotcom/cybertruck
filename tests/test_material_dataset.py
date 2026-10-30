"""既有算法的样本审计、会话隔离、评分及只读打包；测试图均为合成。"""

import copy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
import zipfile

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(ROOT/'pi'))
from robot.vision.material_dataset import audit, compare, confusion, metrics, plan, expanded_profile
from tools.export_material_samples import export, canonical
from tools.replay_material_dataset import create_fixture
from tools.replay_material_handoff import verify_package


class DatasetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        cls.manifest = create_fixture(Path(cls.directory.name)/'fixture')
        cls.original = json.loads(cls.manifest.read_text(encoding='utf-8'))
        cls.profile_path = cls.manifest.parent/'baseline_fixture_profile.json'

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def write_manifest(self, data):
        self.manifest.write_text(json.dumps(data,ensure_ascii=False),encoding='utf-8')

    def tearDown(self):
        self.write_manifest(self.original)

    def test_plan_has_six_colors_all_single_factors_without_claiming_images(self):
        self.assertEqual(len(plan()['positive_cases']),66)
        self.assertEqual(len(plan(True)['positive_cases']),378)
        self.assertFalse(plan()['images_present'])

    def test_synthetic_data_never_fills_real_coverage(self):
        _, report = audit(self.manifest)
        self.assertEqual(report['valid'],219)
        self.assertEqual(report['sources']['synthetic_fixture'],219)
        self.assertEqual(report['coverage']['complete_cells'],0)
        self.assertEqual(len(report['coverage']['missing']),66)

    def test_missing_metadata_is_invalid_and_not_inferred_from_filename(self):
        data = copy.deepcopy(self.original)
        data['samples'] = data['samples'][:1]
        for key in ('capture_session','annotations','conditions','context'):
            invalid = copy.deepcopy(data)
            invalid['samples'][0][key] = None
            self.write_manifest(invalid)
            self.assertEqual(audit(self.manifest)[1]['status'],'INVALID')

    def test_duplicate_image_and_cross_session_split_are_rejected(self):
        data = copy.deepcopy(self.original)
        data['samples'] = [data['samples'][0],copy.deepcopy(data['samples'][0])]
        data['samples'][1].update(id='other',split='test')
        self.write_manifest(data)
        self.assertTrue(audit(self.manifest)[1]['errors'])
        data = copy.deepcopy(self.original)
        data['samples'][73]['capture_session'] = data['samples'][0]['capture_session']
        self.write_manifest(data)
        self.assertTrue(any('会话' in e['reason'] for e in audit(self.manifest)[1]['errors']))

    def test_augmented_parent_cannot_leak_in_either_input_order(self):
        for reverse in (False,True):
            data = copy.deepcopy(self.original)
            rows = [data['samples'][0],data['samples'][73]]
            rows[1]['parent_sha256'] = rows[0]['sha256']
            data['samples'] = list(reversed(rows)) if reverse else rows
            self.write_manifest(data)
            self.assertTrue(audit(self.manifest)[1]['errors'])

    def test_bad_sha_escape_and_invalid_numeric_labels_rejected(self):
        for changes in (dict(sha256='0'*64),dict(file='../outside.png'),dict(file=str(self.manifest.resolve())),
                        dict(annotations=[dict(color_id=True,bbox_px=[1,1,20,20],sample_roi=[2,2,12,12])])):
            data = copy.deepcopy(self.original)
            data['samples'] = [dict(data['samples'][0],**changes)]
            self.write_manifest(data)
            self.assertEqual(audit(self.manifest)[1]['status'],'INVALID')

    def test_only_calibration_frames_enter_parameter_expansion(self):
        samples,_ = audit(self.manifest)
        baseline = json.loads(self.profile_path.read_text(encoding='utf-8'))
        before = copy.deepcopy(baseline)
        expanded,used = expanded_profile(baseline,samples,self.manifest.parent)
        self.assertEqual(baseline,before)
        allowed = {s['id'] for s in samples if s['split']=='calibration'}
        self.assertTrue(all(u['id'] in allowed for u in used))
        self.assertEqual({u['color_id'] for u in used},set(range(1,7)))
        changed = copy.deepcopy(samples)
        for sample in changed:
            if sample['split'] != 'calibration':
                sample['file'] = 'not-read-by-calibration.png'
        other,_ = expanded_profile(baseline,changed,self.manifest.parent)
        self.assertEqual(expanded,other)
        for key in baseline:
            if key != 'colors':
                self.assertEqual(baseline[key],expanded[key])

    def test_full_comparison_uses_same_algorithm_and_does_not_apply(self):
        _,report = compare(self.manifest,self.profile_path)
        self.assertEqual(report['algorithm'],'unchanged MaterialDetector')
        self.assertEqual(len(report['baseline']['records']),146)
        self.assertEqual(len(report['expanded']['records']),146)
        self.assertFalse(report['applied'])
        self.assertFalse(report['physical_acceptance'])
        self.assertTrue(all(k.startswith('synthetic_fixture/') for k in report['expanded']['groups']))

    def test_wrong_color_duplicate_prediction_and_negative_counted(self):
        label = dict(color_id=3,bbox_px=[10,10,40,40])
        wrong = dict(color_id=6,bbox_px=[10,10,40,40])
        result = metrics(confusion([label],[wrong,wrong]))
        self.assertEqual((result['tp'],result['fp'],result['fn']),(0,2,1))
        self.assertEqual(result['confusion_matrix'][3][6],1)
        self.assertEqual(result['confusion_matrix'][0][6],1)
        self.assertIsNone(metrics(confusion([],[]))['per_color']['1']['recall'])

    def test_empty_dataset_has_missing_cells_not_success_metrics(self):
        self.write_manifest(dict(schema_version=1,samples=[]))
        self.assertEqual(audit(self.manifest)[1]['status'],'EMPTY')
        with self.assertRaises(ValueError):
            compare(self.manifest,self.profile_path)

    def test_camera_configuration_and_source_kinds_are_not_mixed_for_calibration(self):
        for field,value in (('source_kind','historical_capture'),('camera_config_sha256','d'*64)):
            data=copy.deepcopy(self.original)
            if field=='source_kind':
                data['samples'][73][field]=value
            else:
                data['samples'][73]['context'][field]=value
            self.write_manifest(data)
            with self.assertRaises(ValueError):
                compare(self.manifest,self.profile_path)


class ExportTests(unittest.TestCase):
    def test_export_is_read_only_hash_checked_and_credentials_excluded(self):
        with tempfile.TemporaryDirectory() as directory:
            project = Path(directory)/'project'
            samples = project/'output/vision/workbench/samples'
            samples.mkdir(parents=True)
            raw = cv2.imencode('.jpg',np.full((32,32,3),128,np.uint8))[1].tobytes()
            record = dict(id='a'*32,source_domain='camera_live',label='present',color_id=1,stage='raw',
                          image_sha256=hashlib.sha256(raw).hexdigest(),note='PRIVATE_NOTE',
                          observation=dict(camera='material',image_size=[32,32],token='SECRET_TOKEN'),
                          device=dict(password='SECRET_PASSWORD',owner='SECRET_OWNER'))
            record['sha256'] = hashlib.sha256(canonical(record).encode()).hexdigest()
            path = samples/('a'*32+'.json')
            path.write_text(canonical(record),encoding='utf-8')
            path.with_suffix('.jpg').write_bytes(raw)
            before = {p.name:p.read_bytes() for p in samples.iterdir()}
            output = Path(directory)/'exported'
            report = export(project,output)
            self.assertEqual(report['exported'],1)
            self.assertEqual(before,{p.name:p.read_bytes() for p in samples.iterdir()})
            manifest = (output/'manifest.json').read_text(encoding='utf-8')
            for forbidden in ('SECRET_TOKEN','SECRET_PASSWORD','SECRET_OWNER','PRIVATE_NOTE'):
                self.assertNotIn(forbidden,manifest)
            entry = json.loads(manifest)['samples'][0]
            self.assertIsNone(entry['annotations'])
            self.assertIsNone(entry['conditions']['camera_height_mm'])
            with zipfile.ZipFile(output.with_suffix('.zip')) as archive:
                self.assertIn('manifest.json',archive.namelist())
            with self.assertRaises(ValueError):
                export(project,output)

    def test_empty_and_corrupt_exports_do_not_report_complete(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            self.assertEqual(export(root/'missing',root/'empty')['status'],'EMPTY')
            source=root/'project/output/vision/workbench/samples'
            source.mkdir(parents=True)
            (source/'bad.json').write_text('{"sha256":"wrong"}',encoding='utf-8')
            result=export(root/'project',root/'corrupt')
            self.assertEqual(result['status'],'EMPTY')
            self.assertTrue(result['errors'])


class HandoffTests(unittest.TestCase):
    def test_manifest_tampering_is_rejected_before_replay(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            (root/'data').mkdir()
            (root/'data/frame.jpg').write_bytes(b'original')
            row=dict(path='data/frame.jpg',sha256=hashlib.sha256(b'original').hexdigest())
            (root/'data_manifest.json').write_text(json.dumps(dict(files=[row])),encoding='utf-8')
            data_hash=hashlib.sha256((root/'data_manifest.json').read_bytes()).hexdigest()
            package=dict(files=[row,dict(path='data_manifest.json',sha256=data_hash)])
            (root/'package_manifest.json').write_text(json.dumps(package),encoding='utf-8')
            self.assertEqual(len(verify_package(root)[1]['files']),1)
            (root/'data/frame.jpg').write_bytes(b'changed')
            with self.assertRaises(ValueError):
                verify_package(root)

    def test_handoff_paths_cannot_escape_package(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)/'package'
            root.mkdir()
            (root.parent/'outside').write_bytes(b'secret')
            row=dict(path='../outside',sha256=hashlib.sha256(b'secret').hexdigest())
            for name in ('data_manifest.json','package_manifest.json'):
                (root/name).write_text(json.dumps(dict(files=[row])),encoding='utf-8')
            with self.assertRaises(ValueError):
                verify_package(root)


if __name__ == '__main__':
    unittest.main()
