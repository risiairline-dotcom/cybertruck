"""生成明确标注的合成流程夹具，调用不变的六色算法；不是现场样本或真实高度测试。"""

import argparse
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'pi'))
from robot.vision.legacy.material_vision.core import PALETTE, default_profile, sample_color, append_shape_reference
from robot.vision.material_dataset import compare, plan


def fixture_profile():
    profile = default_profile()
    profile.update(plate_roi=[180,140,1120,660], plate_roi_shape='rectangle', grasp_roi=[180,140,1120,660])
    for color in range(1,7):
        profile['colors'][str(color)] = sample_color(np.full((40,40,3), PALETTE[color], np.uint8), [0,0,40,40], color)
    append_shape_reference(profile, [[605,460],[695,460],[675,360],[625,360]])
    return profile


def create_fixture(output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    (output/'images').mkdir()
    samples = []
    serial = 0
    spec = plan()
    cases = spec['positive_cases'] + [dict(color_id=0, lighting=light, angle_bin='front', height_bin='nominal')
                                      for light in spec['negative_lights']]
    for split_index, split in enumerate(spec['splits']):
        for case in cases:
            serial += 1
            color, light, angle, height = (case[k] for k in ('color_id','lighting','angle_bin','height_bin'))
            frame = np.full((720,1280,3), 230, np.uint8)
            # 每图显式夹具编号写于检测ROI外；不把相似合成图当作独立现场样本。
            frame[0,0] = [serial%256, serial//256, split_index]
            annotations = []
            if color:
                bgr = np.array(PALETTE[color], dtype=float)
                if light in ('dim','shadow'):
                    bgr *= (.70 if light == 'dim' else .52) + split_index*.015
                elif light == 'bright':
                    bgr *= 1.16 + split_index*.01
                elif light == 'warm':
                    bgr *= [.85,1.,1.1]
                elif light == 'cool':
                    bgr *= [1.1,1.,.85]
                bgr = np.clip(np.rint(bgr),0,255).astype(np.uint8)
                points = np.float32([[-45,50],[45,50],[25,-50],[-25,-50]])
                theta = np.deg2rad(dict(front=0,left=-30,right=30)[angle])
                rotation = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
                scale = dict(low=1.35,nominal=1.,high=.65)[height]
                points = np.rint(points @ rotation.T*scale+[650,410]).astype(np.int32)
                cv2.fillPoly(frame,[points],tuple(int(v) for v in bgr))
                if light == 'glare':
                    cv2.circle(frame,(650,380),7,(245,245,245),-1)
                label = dict(color_id=color,bbox_px=list(cv2.boundingRect(points)))
                if split == 'calibration':
                    label['sample_roi'] = [644,404,656,416]
                annotations.append(label)
            else:
                cv2.line(frame,(500,400),(800,400),(35,35,35),2)
            ok, encoded = cv2.imencode('.png',frame)
            if not ok:
                raise RuntimeError('合成PNG编码失败')
            raw = encoded.tobytes()
            filename = f'images/fixture_{serial:04d}.png'
            (output/filename).write_bytes(raw)
            samples.append(dict(id=f'fixture-{serial}',file=filename,sha256=hashlib.sha256(raw).hexdigest(),
                                source_kind='synthetic_fixture',split=split,capture_session=f'fixture-{split}',
                                context=dict(camera_id='synthetic-camera',camera_config_sha256='a'*64,
                                             image_size=[1280,720],pose_id='fixture-'+angle+'-'+height,
                                             pose_revision=1,pose_sha256='b'*64,target_plane='fixture',view='raw'),
                                conditions=dict(lighting=light,angle_bin=angle,height_bin=height,
                                                view_angle_deg=dict(front=0,left=-30,right=30)[angle],
                                                camera_height_mm=dict(low=100,nominal=150,high=200)[height]),
                                annotations=annotations,measurement_kind='synthetic_parameters_not_measured'))
    (output/'manifest.json').write_text(json.dumps(dict(schema_version=1,samples=samples),ensure_ascii=False,indent=2),encoding='utf-8')
    (output/'baseline_fixture_profile.json').write_text(json.dumps(fixture_profile(),ensure_ascii=False,indent=2),encoding='utf-8')
    return output/'manifest.json'


def run(output):
    manifest = create_fixture(output)
    candidate, report = compare(manifest, manifest.parent/'baseline_fixture_profile.json')
    report['limitation'] = '全为合成流程夹具，角度/高度是渲染参数，不证明现场覆盖或准确率；算法完全相同，仅扩充样本参数'
    for name, value in (('comparison.json',report),('expanded_profile_draft.json',candidate)):
        (manifest.parent/name).write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    report = run(args.output)
    print(json.dumps(dict(valid=report['audit']['valid'],coverage=report['audit']['coverage']['complete_cells'],
                          groups={kind:{key:{m:value[m] for m in ('tp','fp','fn')} for key,value in report[kind]['groups'].items()
                                        if key.endswith('/all')} for kind in ('baseline','expanded')},physical_acceptance=False)))
