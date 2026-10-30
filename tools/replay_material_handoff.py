"""只读核验原A交接包并回放现有识别器；不导入bundle，不恢复任何运行状态。"""

import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'pi'))


def checked_path(root, name):
    if not isinstance(name,str) or Path(name).is_absolute():
        raise ValueError('清单路径必须相对交接包')
    path=(root/name).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError('清单路径超出交接包')
    return path


def verify_package(root):
    root=Path(root).resolve()
    package=json.loads((root/'package_manifest.json').read_text(encoding='utf-8'))
    data=json.loads((root/'data_manifest.json').read_text(encoding='utf-8'))
    for manifest in (package,data):
        if not isinstance(manifest.get('files'),list):
            raise ValueError('交接包清单格式无效')
        seen=set()
        for row in manifest['files']:
            if row['path'] in seen:
                raise ValueError('清单存在重复路径')
            seen.add(row['path'])
            raw=checked_path(root,row['path']).read_bytes()
            if hashlib.sha256(raw).hexdigest()!=row['sha256']:
                raise ValueError('交接包SHA不匹配：'+row['path'])
    return package,data


def replay(root, output):
    root,output=Path(root).resolve(),Path(output).resolve()
    if output.exists() or output.is_relative_to(root):
        raise ValueError('输出需新文件，且不得写入交接包')
    package,data=verify_package(root)
    import cv2
    import numpy as np
    from robot.vision.observe import Detector
    from robot.vision.material_dataset import eligible
    detector=Detector()
    listed={row['path']:row for row in data['files']}
    records=[]
    for name,row in listed.items():
        if not name.lower().endswith(('.jpg','.png')):
            continue
        raw=checked_path(root,name).read_bytes()
        # 前面核验后再次核对本次用于推理的同一字节，避免校验后替换。
        if hashlib.sha256(raw).hexdigest()!=row['sha256']:
            raise ValueError('读取期间图像变化')
        frame=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError('图像解码失败：'+name)
        item=detector.process(frame,'material')
        metadata_name=str(Path(name).with_suffix('.json')).replace('\\','/')
        metadata={}
        if metadata_name in listed:
            content=checked_path(root,metadata_name).read_bytes()
            if hashlib.sha256(content).hexdigest()!=listed[metadata_name]['sha256']:
                raise ValueError('读取期间元数据变化')
            metadata=json.loads(content.decode('utf-8'))
        candidates=item['items']
        accepted=eligible(candidates,detector.profiles['material_profile'])
        hint=metadata.get('color_id')
        record=dict(file=name,sha256=row['sha256'],image_size=item['image_size'],items=candidates,
                    eligible_color_ids=[c['color_id'] for c in accepted],physical_acceptance=False)
        if metadata:
            title=metadata.get('name','')
            record.update(saved_name=title,operator_color_hint=hint,operator_state_label=metadata.get('label'),
                          source_domain=metadata.get('source_domain'),
                          selected_color_candidate_present=any(c['color_id']==hint for c in candidates) if hint else None,
                          selected_color_eligible_present=any(c['color_id']==hint for c in accepted) if hint else None,
                          saved_profile_matches=metadata.get('observation',{}).get('profile_sha256')==detector.version,
                          has_pose_binding=bool(metadata.get('binding')),
                          condition_name_hint=next((token for token in ('正常光','稍转角','阴影','反光') if token in title),None),
                          measured_angle_height_available=False,
                          metadata_sanitized=listed[metadata_name].get('sanitized',False))
        records.append(record)
    archive_path='data/output/vision/replay.json'
    archive=[]
    if archive_path in listed:
        raw=checked_path(root,archive_path).read_bytes()
        if hashlib.sha256(raw).hexdigest()!=listed[archive_path]['sha256']:
            raise ValueError('旧回放记录读取时变化')
        previous=json.loads(raw.decode('utf-8'))
        for entry in previous.get('records',[]):
            name='data/output/vision/samples/'+entry['sample']['file']
            if name not in listed or entry['sample']['sha256']!=listed[name]['sha256']:
                raise ValueError('历史回放引用的图像未通过包校验')
            raw=checked_path(root,name).read_bytes()
            if hashlib.sha256(raw).hexdigest()!=listed[name]['sha256']:
                raise ValueError('历史图像读取时变化')
            frame=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
            actual=detector.process(frame,entry['mode'])['items']
            exact=json.dumps(actual,sort_keys=True,allow_nan=False)==json.dumps(entry['items'],sort_keys=True,allow_nan=False)
            archive.append(dict(file=name,mode=entry['mode'],exact_items_match=exact))
    color_rows=[r for r in records if r.get('condition_name_hint') and r.get('operator_color_hint')]
    report=dict(domain='historical_handoff_replay',package_files_verified=len(package['files']),
                data_files_verified=len(data['files']),images=len(records),
                unique_images=len({r['sha256'] for r in records}),
                workbench_records=sum('saved_name' in r for r in records),
                named_color_records=len(color_rows),
                named_by_color={str(c):dict(records=sum(r['operator_color_hint']==c for r in color_rows),
                    candidate_present=sum(r['operator_color_hint']==c and r['selected_color_candidate_present'] for r in color_rows),
                    eligible_present=sum(r['operator_color_hint']==c and r['selected_color_eligible_present'] for r in color_rows))
                    for c in range(1,7)},
                full_pose_bindings=sum(r.get('has_pose_binding',False) for r in records),
                historical_profile_sha256=detector.version,opencv=cv2.__version__,numpy=np.__version__,
                prior_archive_comparison=archive,records=records,physical_acceptance=False,
                limitation='颜色选择提示的候选出现率不是准确率；没有完整框真值、独立会话拆分、实测角度高度，不能拟合后宣称现场提升。包内状态不是当前设备状态。')
    output.parent.mkdir(parents=True,exist_ok=True)
    with output.open('x',encoding='utf-8') as handle:
        json.dump(report,handle,ensure_ascii=False,indent=2,allow_nan=False)
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    report=replay(args.package,args.output)
    print(json.dumps({k:report[k] for k in ('package_files_verified','data_files_verified','images','workbench_records',
                                          'named_color_records','named_by_color','full_pose_bindings','physical_acceptance')},ensure_ascii=False))
