"""静止实测点配准的几何与证据约束；没有设备访问。"""
import copy
import math
import unittest
from test_lidar_health import ROOT
from robot.lidar.calibration import center,sample,fit,validate
from robot.lidar.replay import map_point


def report(identity='a'):
    frames=[dict(scan_sequence=i,frame_count=14,objects=[dict(x_mm=400,y_mm=1900,
        frame_hits=14,confidence_percent=100,point_count=80,grid_id=3)]) for i in range(8)]
    return dict(id=identity,domain='physical_lidar_scan',live=False,status='COMPLETE',stop_confirmed=True,
        errors=[],decode_bad=0,decode_unknown=0,zone=1,board_identity=['boot','session'],frames=frames,
        information=dict(zone=1,running=0,x_mm=156,y_mm=2254,heading_deg=198.4,firmware='1.37'))


def samples():
    values=[];transform=[.04,.03,.05]
    for i,p in enumerate(((.4,1.9),(.7,1.9),(.4,1.6))):
        target=map_point(p[0]*1000,p[1]*1000,transform)
        value=sample(report(str(i)),target[0]*1000,2400-target[1]*1000,5)
        value['source_m']=list(p);values.append(value)
    return values,transform


class CalibrationTests(unittest.TestCase):
    def test_center_requires_real_complete_stop_and_eight_distinct_results(self):
        self.assertEqual([round(v,9) for v in center(report())['source_m']],[.4,1.9])
        for key,value in [('domain','simulation'),('stop_confirmed',False),('live',True),('decode_bad',1)]:
            r=report();r[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):center(r)
        r=report();r['frames'][0]['scan_sequence']=1
        with self.assertRaises(ValueError):center(r)

    def test_unknown_empty_ambiguous_low_quality_and_unstable_targets_rejected(self):
        for objects in ([],report()['frames'][0]['objects']*2):
            r=report();r['frames'][0]['objects']=objects
            with self.assertRaises(ValueError):center(r)
        r=report();r['frames'][0]['objects'][0]['confidence_percent']=40
        with self.assertRaises(ValueError):center(r)
        r=report();r['frames'][0]['objects'][0]['x_mm']=550
        with self.assertRaises(ValueError):center(r)

    def test_rigid_transform_and_independent_check(self):
        values,transform=samples();candidate=fit(values)
        for actual,expected in zip(candidate['map_transform'],transform):self.assertAlmostEqual(actual,expected)
        self.assertFalse(candidate['real_map_registered'])
        independent=copy.deepcopy(values[0]);independent['scan_id']='independent';independent['source_m']=[.55,1.9]
        independent['map_m']=map_point(550,1900,transform)
        good=validate(candidate,independent)
        self.assertTrue(good['real_map_registered']);self.assertFalse(good['authorizes_motion'])
        self.assertEqual(good['coverage'],'unknown');self.assertLess(good['independent_check']['error_m'],1e-8)

    def test_duplicates_profile_changes_collinear_mirror_and_bad_scale_rejected(self):
        values,_=samples()
        variants=[]
        v=copy.deepcopy(values);v[1]['scan_id']=v[0]['scan_id'];variants.append(v)
        v=copy.deepcopy(values);v[1]['board_identity']=['other','session'];variants.append(v)
        v=copy.deepcopy(values);v[2]['map_m']=v[1]['map_m'];variants.append(v)
        v=copy.deepcopy(values);v[2]['source_m'][1]=2.2;variants.append(v)
        v=copy.deepcopy(values);v[1]['source_m'][0]=.9;variants.append(v)
        for v in variants:
            with self.subTest(v=v),self.assertRaises(ValueError):fit(v)

    def test_holdout_cannot_reuse_fit_or_accept_large_error_or_modified_version(self):
        values,_=samples();candidate=fit(values)
        with self.assertRaises(ValueError):validate(candidate,values[0])
        independent=copy.deepcopy(values[0]);independent['scan_id']='new';independent['map_m']=[1,1]
        with self.assertRaises(ValueError):validate(candidate,independent)
        candidate['map_transform'][0]+=.1
        with self.assertRaises(ValueError):validate(candidate,independent)

    def test_measurements_are_not_assumed_from_nominal_or_nonfinite_values(self):
        for values in ((450,450,0),(450,450,50),(math.nan,450,5),(3000,450,5)):
            with self.assertRaises(ValueError):sample(report(),*values)


if __name__=='__main__':unittest.main()
