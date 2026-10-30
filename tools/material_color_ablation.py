"""原六色算法的开发集采样消融；拟合图也参与回读，结果不是独立验证或现场验收。"""

import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(ROOT/'pi'))

import cv2
import numpy as np
from robot.vision.legacy.material_vision.core import MaterialDetector, sample_color, merge_color_sample, validate_profile
from robot.vision.material_dataset import eligible
from tools.replay_material_handoff import verify_package, checked_path


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def review_candidates(items):
    """保留框/中心供A逐框复核；无真值时不生成中心误差或准确率。"""
    return [dict(color_id=int(item['color_id']), center_px=[float(v) for v in item['center_px']],
                 bbox_px=[float(v) for v in item['bbox_px']], quality_score=float(item['quality_score']),
                 shape_consistent=bool(item['shape_consistent'])) for item in items]


def load_frame(root,row):
    raw=checked_path(root,row['file']).read_bytes()
    if sha(raw)!=row['sha256']:
        raise ValueError('图像摘要不匹配')
    frame=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError('图像解码失败')
    return frame


def fit_development(profile,rows,root):
    """保留算法与阈值，仅用原采样/合并函数。调用者必须保留开发集来源标记。"""
    candidate=copy.deepcopy(validate_profile(copy.deepcopy(profile)))
    used=[]
    for row in rows:
        color=row.get('color_id')
        if type(color) is not int or color not in range(1,7):
            raise ValueError('无效颜色标签')
        frame=load_frame(root,row)
        if list(frame.shape[:2][::-1])!=profile['resolution']:
            raise ValueError('样本分辨率与基线不同')
        roi=row.get('sample_roi')
        if not isinstance(roi,list) or len(roi)!=4 or any(type(v) is not int for v in roi):
            raise ValueError('必须给出人工复核的整数ROI')
        model=sample_color(frame,roi,color)
        model.pop('sampled_at_unix',None)
        model.update(source_sha256=row['sha256'],annotation_origin='assistant_visual_review_pending_A')
        candidate['colors'][str(color)]=merge_color_sample(candidate['colors'][str(color)],model)
        used.append(dict(file=row['file'],sha256=row['sha256'],color_id=color,sample_roi=roi,model=model))
    # 不允许借离线草稿修改原有分辨率/ROI/形状/质量及稳定性门槛。
    if {k:v for k,v in profile.items() if k!='colors'}!={k:v for k,v in candidate.items() if k!='colors'}:
        raise ValueError('开发草稿改变了非颜色参数')
    return validate_profile(candidate),used


def run(package,plan_path,profile_path,output):
    package,output=Path(package).resolve(),Path(output).resolve()
    if output.exists() or output.is_relative_to(package):
        raise ValueError('输出需为原包外的新目录')
    _,manifest=verify_package(package)
    files={row['path']:row for row in manifest['files']}
    plan=json.loads(Path(plan_path).read_text(encoding='utf-8'))
    if (plan.get('purpose')!='development_ablation_only' or plan.get('independent_validation') is not False or
            plan.get('a_confirmation_pending') is not True):
        raise ValueError('本工具只能运行待复核的开发消融，不能标成独立验证')
    rows=plan['samples']
    if not isinstance(rows,list) or not rows or len(rows)>48:
        raise ValueError('开发采样清单无效')
    if len({row['sha256'] for row in rows})!=len(rows):
        raise ValueError('同一图像不能重复作为多份开发采样')
    for row in rows:
        if files.get(row['file'],{}).get('sha256')!=row['sha256']:
            raise ValueError('开发图不在已核验交接包内')
        meta_name=str(Path(row['file']).with_suffix('.json')).replace('\\','/')
        meta_raw=checked_path(package,meta_name).read_bytes()
        if sha(meta_raw)!=files[meta_name]['sha256']:
            raise ValueError('元数据摘要不匹配')
        metadata=json.loads(meta_raw.decode('utf-8'))
        if metadata.get('color_id')!=row['color_id']:
            raise ValueError('开发标签与原人工颜色提示冲突')
    raw_profile=Path(profile_path).read_bytes()
    profile=json.loads(raw_profile.decode('utf-8'))
    draft,used=fit_development(profile,rows,package)
    old,new=MaterialDetector(profile),MaterialDetector(draft)
    fitted={r['sha256'] for r in rows}
    records=[]
    for name,entry in files.items():
        if not name.lower().endswith(('.jpg','.png')):
            continue
        frame=load_frame(package,dict(file=name,sha256=entry['sha256']))
        before=eligible(old.detect(frame),profile)
        after=eligible(new.detect(frame),draft)
        before_ids=[c['color_id'] for c in before]
        after_ids=[c['color_id'] for c in after]
        record=dict(file=name,sha256=entry['sha256'],fit_image=entry['sha256'] in fitted,
                    before_ids=before_ids,after_ids=after_ids,candidate_count_changed=len(before)!=len(after),
                    before_candidates=review_candidates(before),after_candidates=review_candidates(after))
        meta_name=str(Path(name).with_suffix('.json')).replace('\\','/')
        if meta_name in files:
            meta_raw=checked_path(package,meta_name).read_bytes()
            if sha(meta_raw)!=files[meta_name]['sha256']:
                raise ValueError('回读元数据变化')
            metadata=json.loads(meta_raw.decode('utf-8'))
            hint=metadata.get('color_id')
            title=metadata.get('name','')
            if hint and any(token in title for token in ('正常光','稍转角','阴影','反光')):
                record.update(name=title,color_hint=hint,before_hint_present=hint in before_ids,after_hint_present=hint in after_ids)
        records.append(record)
    labelled=[r for r in records if 'color_hint' in r]
    same_capture_summary={}
    for group,predicate in (('fit_images_resubstitution',lambda r:r['fit_image']),
                            ('other_images_same_capture_session',lambda r:not r['fit_image'])):
        selected=[r for r in labelled if predicate(r)]
        same_capture_summary[group]=dict(images=len(selected),before=sum(r['before_hint_present'] for r in selected),
                                         after=sum(r['after_hint_present'] for r in selected))
    report=dict(domain='development_ablation',independent_validation=False,physical_acceptance=False,applied=False,
                algorithm='unchanged MaterialDetector/sample_color/merge_color_sample',
                algorithm_sha256=sha((ROOT/'pi/robot/vision/legacy/material_vision/core.py').read_bytes()),
                baseline_profile_sha256=sha(raw_profile),annotation_plan_sha256=sha(Path(plan_path).read_bytes()),
                opencv=cv2.__version__,numpy=np.__version__,thresholds_unchanged=True,
                images=len(records),used=used,same_capture_summary=same_capture_summary,records=records,
                caution='拟合图回读是重代入；其余图同一采集会话，不是独立测试。颜色提示出现不是准确率，新增框需人工复核。不得自动应用。')
    output.mkdir(parents=True)
    for name,value in (('profile_draft.json',draft),('report.json',report)):
        with (output/name).open('x',encoding='utf-8') as handle:
            json.dump(value,handle,ensure_ascii=False,indent=2,allow_nan=False)
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package',type=Path,required=True)
    parser.add_argument('--plan',type=Path,default=ROOT/'docs/vision_color_development_rois.json')
    parser.add_argument('--profile',type=Path,default=ROOT/'config/vision/material_profile.json')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    report=run(args.package,args.plan,args.profile,args.output)
    print(json.dumps(dict(images=report['images'],same_capture_summary=report['same_capture_summary'],
                          independent_validation=False,applied=False),ensure_ascii=False))
