"""既有六色算法的离线样本、参数扩充与对照；不改算法，不访问设备。"""

import copy
import hashlib
import itertools
import json
import math
from pathlib import Path

import cv2
import numpy as np

from .legacy.material_vision.core import MaterialDetector, sample_color, merge_color_sample, validate_profile
from .task_evidence import observation_context, sha256_text

LIGHTS = ('normal', 'dim', 'bright', 'warm', 'cool', 'shadow', 'glare')
ANGLES = ('front', 'left', 'right')
HEIGHTS = ('low', 'nominal', 'high')
SPLITS = ('calibration', 'validation', 'test')
SOURCES = ('physical_capture', 'historical_capture', 'synthetic_fixture')


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def canonical(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True, allow_nan=False)


def sha(value):
    return hashlib.sha256(value).hexdigest()


def plan(full=False):
    """计划没有原图，不把计划行或合成测试计入真实样本覆盖。"""
    conditions = list(itertools.product(LIGHTS, ANGLES, HEIGHTS)) if full else (
        [(light, 'front', 'nominal') for light in LIGHTS] +
        [('normal', angle, 'nominal') for angle in ('left', 'right')] +
        [('normal', 'front', height) for height in ('low', 'high')])
    cases = [dict(color_id=color, lighting=light, angle_bin=angle, height_bin=height)
             for color in range(1, 7) for light, angle, height in conditions]
    return dict(schema_version=1, domain='capture_plan', images_present=False,
                positive_cases=cases, negative_lights=list(LIGHTS), splits=list(SPLITS),
                measurements='A填写相机光轴相对目标平面法线的角度、镜头到目标平面的实测高度；不得由像素推算',
                physical_acceptance=False)


def checked_image(root, sample):
    name = sample.get('file')
    require(isinstance(name, str) and bool(name) and not Path(name).is_absolute(), '图像必须使用包内相对路径')
    path = (Path(root) / name).resolve()
    require(path.is_relative_to(Path(root).resolve()), '图像路径超出数据包')
    require(sha256_text(sample.get('sha256')), '缺少图像SHA256')
    with path.open('rb') as handle:
        raw = handle.read(4*1024*1024+1)
    require(0 < len(raw) <= 4*1024*1024 and sha(raw) == sample['sha256'], '图像缺失/过大/摘要不匹配')
    image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
    require(image is not None, '图像无法解码')
    return image


def validate_sample(sample, root):
    require(isinstance(sample, dict), '样本必须为对象')
    for key in ('id', 'capture_session'):
        require(isinstance(sample.get(key), str) and 0 < len(sample[key]) <= 128, '缺少'+key)
    require(sample.get('split') in SPLITS, '缺少calibration/validation/test拆分')
    require(sample.get('source_kind') in SOURCES, '缺少真实/历史/合成来源分类')
    context = observation_context(sample.get('context'))
    conditions = sample.get('conditions')
    require(isinstance(conditions, dict), '缺少采集条件')
    for key, choices in (('lighting', LIGHTS), ('angle_bin', ANGLES), ('height_bin', HEIGHTS)):
        require(conditions.get(key) in choices, '缺少条件分组：'+key)
    for key in ('view_angle_deg', 'camera_height_mm'):
        value = conditions.get(key)
        require(type(value) in (int, float) and math.isfinite(value), '缺少实测数值：'+key)
    require(-89 <= conditions['view_angle_deg'] <= 89 and conditions['camera_height_mm'] > 0, '角度或高度范围无效')
    labels = sample.get('annotations')
    require(isinstance(labels, list) and len(labels) <= 18, '必须人工标注框；确认整幅无物料才填空列表')
    width, height = context['image_size']
    for label in labels:
        require(isinstance(label, dict) and type(label.get('color_id')) is int and label['color_id'] in range(1, 7),
                '标注颜色必须为1～6整数')
        box = label.get('bbox_px')
        require(isinstance(box, list) and len(box) == 4 and
                all(type(v) in (int, float) and math.isfinite(v) for v in box), '标注框需为xywh像素坐标')
        x, y, w, h = box
        require(0 <= x < x+w <= width and 0 <= y < y+h <= height, '标注框超出图像')
        if sample['split'] == 'calibration':
            roi = label.get('sample_roi')
            require(isinstance(roi, list) and len(roi) == 4 and all(type(v) is int for v in roi), '标定需整数xyxy表面采样框')
            require(x <= roi[0] < roi[2] <= x+w and y <= roi[1] < roi[3] <= y+h and
                    (roi[2]-roi[0])*(roi[3]-roi[1]) >= 64, '表面采样框不在物料标注内或太小')
    image = checked_image(root, sample)
    require(list(image.shape[:2][::-1]) == context['image_size'], '图像与相机分辨率不一致')
    return copy.deepcopy(sample)


def coverage(samples, full=False):
    spec = plan(full)
    rows = []
    for case in spec['positive_cases']:
        counts = {split: 0 for split in SPLITS}
        for sample in samples:
            if sample['source_kind'] == 'synthetic_fixture':
                continue
            if all(sample['conditions'][key] == case[key] for key in ('lighting', 'angle_bin', 'height_bin')) and any(
                    label['color_id'] == case['color_id'] for label in sample['annotations']):
                counts[sample['split']] += 1
        rows.append(dict(case, counts=counts, missing_splits=[s for s in SPLITS if counts[s] == 0]))
    negative = {light: {split: sum(s['source_kind'] != 'synthetic_fixture' and not s['annotations'] and
                                  s['conditions']['lighting'] == light and s['split'] == split for s in samples)
                        for split in SPLITS} for light in LIGHTS}
    return dict(positive_cells=len(rows), complete_cells=sum(not r['missing_splits'] for r in rows),
                missing=[r for r in rows if r['missing_splits']], cells=rows, negative_counts=negative,
                physical_acceptance=False, scope='真实/历史图像覆盖；不是当前实物验收')


def audit(manifest_path, full=False):
    path = Path(manifest_path)
    data = json.loads(path.read_text(encoding='utf-8'))
    require(data.get('schema_version') == 1 and isinstance(data.get('samples'), list), '样本清单schema无效')
    samples, errors, ids, hashes, sessions, parents = [], [], set(), {}, {}, {}
    for index, raw in enumerate(data['samples']):
        try:
            sample = validate_sample(raw, path.parent)
            require(sample['id'] not in ids, '重复样本ID')
            require(sample['sha256'] not in hashes, '重复图像SHA；不能复制同帧充样本')
            session = sample['capture_session']
            require(session not in sessions or sessions[session] == sample['split'], '同一采集会话跨拆分，存在相邻帧泄漏')
            family = sample.get('parent_sha256', sample['sha256'])
            require(sha256_text(family), '原图家族摘要无效')
            require(family not in parents or parents[family] == sample['split'], '原图和增强图跨拆分')
            ids.add(sample['id']); hashes[sample['sha256']] = sample['split']; sessions[session] = sample['split']
            parents[family] = sample['split']
            samples.append(sample)
        except (ValueError, TypeError, KeyError, OSError) as exc:
            errors.append(dict(index=index, id=raw.get('id') if isinstance(raw, dict) else None, reason=str(exc)))
    # 防止增强图先出现、原图后出现的顺序绕过。
    for sample in samples:
        family = sample.get('parent_sha256', sample['sha256'])
        if family in hashes and hashes[family] != sample['split']:
            errors.append(dict(id=sample['id'], reason='原图和增强图跨拆分'))
    report = dict(schema_version=1, status='INVALID' if errors else 'READY' if samples else 'EMPTY',
                  declared=len(data['samples']), valid=len(samples), errors=errors,
                  sources={source: sum(s['source_kind'] == source for s in samples) for source in SOURCES},
                  coverage=coverage(samples, full), physical_acceptance=False)
    return samples, report


def expanded_profile(profile, samples, root):
    """只用calibration组调用原采样/合并函数，不调阈值，不处理光照，不更新正式文件。"""
    result, used = copy.deepcopy(validate_profile(copy.deepcopy(profile))), []
    for sample in samples:
        if sample['split'] != 'calibration' or not sample['annotations']:
            continue
        require(sample['context']['image_size'] == profile['resolution'], '样本与基线配置分辨率不同')
        image = checked_image(root, sample)
        for label in sample['annotations']:
            color = label['color_id']
            model = sample_color(image, label['sample_roi'], color)
            # 去除墙钟，保证相同输入生成相同离线草稿，不冒充原始曝光时间。
            model.pop('sampled_at_unix', None)
            model.update(source_sha256=sample['sha256'], capture_session=sample['capture_session'])
            result['colors'][str(color)] = merge_color_sample(result['colors'][str(color)], model)
            used.append(dict(id=sample['id'], sha256=sample['sha256'], color_id=color))
    return validate_profile(result), used


def eligible(items, profile):
    """两端同一候选门槛；黑色形状与边缘条件沿用原观测器要求。"""
    return [item for item in items if item['color_calibrated'] and not item['clipped'] and
            item['quality_score'] >= profile['min_score'] and
            (item['color_id'] != 5 or item['shape_consistent'])]


def iou(first, second):
    x, y, w, h = first
    a, b, c, d = second
    intersection = max(0, min(x+w, a+c)-max(x, a)) * max(0, min(y+h, b+d)-max(y, b))
    return intersection / max(1e-12, w*h+c*d-intersection)


def confusion(labels, predictions):
    """按IoU降序一对一匹配，不利用颜色偷选匹配；0表示漏检/背景。"""
    matrix = [[0]*7 for _ in range(7)]
    pairs = sorted(((iou(a['bbox_px'], b['bbox_px']), i, j) for i, a in enumerate(labels)
                    for j, b in enumerate(predictions)), reverse=True)
    matched_labels, matched_predictions = set(), set()
    for overlap, i, j in pairs:
        if overlap < .5:
            break
        if i not in matched_labels and j not in matched_predictions:
            matrix[labels[i]['color_id']][predictions[j]['color_id']] += 1
            matched_labels.add(i); matched_predictions.add(j)
    for i, label in enumerate(labels):
        if i not in matched_labels:
            matrix[label['color_id']][0] += 1
    for j, prediction in enumerate(predictions):
        if j not in matched_predictions:
            matrix[0][prediction['color_id']] += 1
    return matrix


def metrics(matrix):
    colors = {}
    for color in range(1, 7):
        tp = matrix[color][color]
        fn = sum(matrix[color])-tp
        fp = sum(row[color] for row in matrix)-tp
        colors[str(color)] = dict(tp=tp, fp=fp, fn=fn, precision=tp/(tp+fp) if tp+fp else None,
                                  recall=tp/(tp+fn) if tp+fn else None)
    return dict(confusion_matrix=matrix, color_order=['miss/background', '红', '黄', '蓝', '绿', '黑', '浅蓝'],
                per_color=colors, tp=sum(v['tp'] for v in colors.values()),
                fp=sum(v['fp'] for v in colors.values()), fn=sum(v['fn'] for v in colors.values()))


def evaluate(profile, samples, root):
    detector = MaterialDetector(profile)
    groups, records = {}, []
    for sample in samples:
        if sample['split'] not in ('validation', 'test'):
            continue
        image = checked_image(root, sample)
        require(list(image.shape[:2][::-1]) == profile['resolution'], '评估样本分辨率与配置不同')
        predictions = eligible(detector.detect(image), profile)
        matrix = confusion(sample['annotations'], predictions)
        record = dict(id=sample['id'], split=sample['split'], source_kind=sample['source_kind'],
                      conditions=sample['conditions'], **metrics(matrix))
        records.append(record)
        cond = sample['conditions']
        # 合成与真实/历史分别分组，不合并成一个“现场准确率”。
        for key in (f"{sample['source_kind']}/{sample['split']}/all",
                    f"{sample['source_kind']}/{sample['split']}/light:{cond['lighting']}",
                    f"{sample['source_kind']}/{sample['split']}/angle:{cond['angle_bin']}",
                    f"{sample['source_kind']}/{sample['split']}/height:{cond['height_bin']}"):
            total = groups.setdefault(key, [[0]*7 for _ in range(7)])
            for i in range(7):
                for j in range(7):
                    total[i][j] += matrix[i][j]
    return dict(records=records, groups={key: metrics(value) for key, value in groups.items()})


def compare(manifest_path, profile_path):
    samples, inspection = audit(manifest_path)
    require(not inspection['errors'], '样本审计失败；先补全元数据/修复文件，不丢弃错误后报通过')
    for split in SPLITS:
        require(any(s['split'] == split for s in samples), '缺少'+split+'组；不能计算对照')
    cameras = {(s['context']['camera_id'], s['context']['camera_config_sha256']) for s in samples}
    require(len(cameras) == 1, '不同相机/配置的数据应分别评估，不能混合标定')
    require(len({s['source_kind'] for s in samples}) == 1, '真实/历史/合成数据请分别建包，不混合扩充参数')
    raw_profile = Path(profile_path).read_bytes()
    profile = json.loads(raw_profile.decode('utf-8'))
    candidate, used = expanded_profile(profile, samples, Path(manifest_path).parent)
    require({r['color_id'] for r in used} == set(range(1, 7)), '标定组尚未覆盖六色，不生成完整六色对照')
    implementation = Path(__file__).parent / 'legacy/material_vision/core.py'
    report = dict(schema_version=1, domain='offline_profile_comparison', algorithm='unchanged MaterialDetector',
                  algorithm_sha256=sha(implementation.read_bytes()), baseline_profile_sha256=sha(raw_profile),
                  candidate_profile_sha256=sha(canonical(candidate).encode()), calibration_used=used,
                  inherited_baseline_provenance='原配置样本会话未知；不能据此宣称完全独立的现场测试集',
                  manifest_sha256=sha(Path(manifest_path).read_bytes()), audit=inspection,
                  baseline=evaluate(profile, samples, Path(manifest_path).parent),
                  expanded=evaluate(candidate, samples, Path(manifest_path).parent),
                  applied=False, physical_acceptance=False, opencv=cv2.__version__, numpy=np.__version__)
    return candidate, report
