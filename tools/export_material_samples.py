"""A端只读导出已保存视觉资料；标准库即可，不连接Pi/相机/串口，不复制运行凭据。"""

import argparse
import hashlib
import json
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def canonical(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True, allow_nan=False)


def export(project, output):
    project, output = Path(project).resolve(), Path(output).resolve()
    if output.exists() or output.with_suffix('.zip').exists():
        raise ValueError('输出目录或ZIP已存在；请选新名称，原资料不会覆盖')
    sources = []
    errors = []
    workspace = project/'output/vision/workbench/samples'
    if workspace.is_dir():
        for path in sorted(workspace.glob('*.json')):
            try:
                if not path.resolve().is_relative_to(project):
                    raise ValueError('保存记录路径超出项目')
                record_raw = path.read_bytes()
                record = json.loads(record_raw.decode('utf-8'))
                unsigned = {k:v for k,v in record.items() if k != 'sha256'}
                if digest(canonical(unsigned).encode()) != record.get('sha256'):
                    raise ValueError('保存记录SHA不一致')
                sources.append((path.with_suffix('.jpg'), record.get('image_sha256'), dict(
                    source_id=record.get('id'), source_record_sha256=digest(record_raw),
                    source_domain=record.get('source_domain'), operator_label=record.get('label'),
                    color_id_hint=record.get('color_id'), stage=record.get('stage'),
                    observation={k:record.get('observation', {}).get(k) for k in
                                 ('camera', 'image_size', 'profile_sha256', 'frame_id', 'stream_id')})))
            except (OSError, ValueError, TypeError, AttributeError) as exc:
                errors.append(dict(file=path.name, reason=str(exc)))
    legacy = project/'output/vision/samples.json'
    if legacy.exists():
        try:
            if not legacy.resolve().is_relative_to(project):
                raise ValueError('旧清单路径超出项目')
            items = json.loads(legacy.read_text(encoding='utf-8'))
            if not isinstance(items, list):
                raise ValueError('旧样本清单不是列表')
            for item in items:
                file = item.get('file')
                if not isinstance(file, str) or Path(file).name != file:
                    errors.append(dict(file=str(file), reason='旧清单需包内文件名'))
                    continue
                sources.append((legacy.parent/'samples'/file, item.get('sha256'), dict(
                    source_id=item.get('id'), source_domain='historical_replay', observation={})))
        except (OSError, ValueError, TypeError, AttributeError) as exc:
            errors.append(dict(file='samples.json', reason=str(exc)))
    output.mkdir(parents=True)
    (output/'images').mkdir()
    samples, seen = [], set()
    duplicates = 0
    for image_path, expected_sha, metadata in sources:
        try:
            # 解析链接后的路径仍需位于项目内，防止误打包其它本机文件。
            if not image_path.resolve().is_relative_to(project):
                raise ValueError('图像路径超出项目')
            with image_path.open('rb') as handle:
                raw = handle.read(4*1024*1024+1)
            actual_sha = digest(raw)
            if not raw or len(raw) > 4*1024*1024 or actual_sha != expected_sha:
                raise ValueError('图像缺失/过大/与保存SHA不匹配')
            if actual_sha in seen:
                duplicates += 1
                continue
            seen.add(actual_sha)
            name = f'images/{actual_sha}.jpg'
            (output/name).write_bytes(raw)
            domain = metadata['source_domain']
            kind = ('physical_capture' if domain == 'camera_live' else 'historical_capture'
                    if domain == 'historical_replay' else 'synthetic_fixture' if domain == 'simulation' else None)
            obs = metadata['observation']
            samples.append(dict(id=actual_sha[:24], file=name, sha256=actual_sha, source_kind=kind,
                                capture_session=None, split=None, annotations=None,
                                context=dict(camera_id=obs.get('camera'), image_size=obs.get('image_size'),
                                             camera_config_sha256=None, pose_id=None, pose_revision=None,
                                             pose_sha256=None, target_plane=None, view=None),
                                conditions=dict(lighting=None, angle_bin=None, height_bin=None,
                                                view_angle_deg=None, camera_height_mm=None),
                                exported_metadata=metadata))
        except (OSError, ValueError) as exc:
            errors.append(dict(file=image_path.name, reason=str(exc)))
    profile = project/'config/vision/material_profile.json'
    profile_sha = None
    if profile.is_file():
        if not profile.resolve().is_relative_to(project):
            raise ValueError('配置路径超出项目')
        raw = profile.read_bytes()
        profile_sha = digest(raw)
        (output/'material_profile_at_export.json').write_bytes(raw)
    manifest = dict(schema_version=1, status='NEEDS_OPERATOR_ANNOTATION', samples=samples,
                    current_profile_sha256=profile_sha,
                    current_profile_relation='仅导出时配置，不保证为每张历史帧的采集配置',
                    physical_acceptance=False)
    report = dict(exported=len(samples), duplicates=duplicates, errors=errors,
                  status='EMPTY' if not samples else 'PARTIAL' if errors else 'EXPORTED_NEEDS_ANNOTATION',
                  credentials_exported=False, source_modified=False)
    (output/'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
    (output/'export_report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    (output/'READ_ME.txt').write_text(
        '仅导出现有JPG与白名单元数据。未导出设备快照、连接信息、口令、自由备注或运行时保存表。\n'
        '请人工补齐manifest中的会话拆分、全部物料框、相机/观察配置和实测条件；不清楚则留null。\n'
        'annotations=null是尚未标注；只有确认整幅无物料才填[]。未填信息不能纳入真实覆盖。\n'
        '高度为镜头到目标平面的实测毫米，角度为光轴相对目标平面法线；不要由图像比例推算。\n', encoding='utf-8')
    with zipfile.ZipFile(output.with_suffix('.zip'), 'x', zipfile.ZIP_DEFLATED) as archive:
        for file in sorted(output.rglob('*')):
            if file.is_file():
                archive.write(file, file.relative_to(output))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project', type=Path, default=ROOT)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(export(args.project, args.output), ensure_ascii=False))
