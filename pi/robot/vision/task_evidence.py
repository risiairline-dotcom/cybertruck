"""任务证据边界：旧视觉只产候选；离线断言必须显式标注来源和观察上下文。"""

import copy
import math
import re

from ..task_check import require
from ..ledger import canonical


def same_json(first, second):
    """合同按JSON内容比较，防止True与1、1.0与1被Python相等规则混淆。"""
    try:
        return canonical(first) == canonical(second)
    except (TypeError, ValueError):
        return False


def sha256_text(value):
    return isinstance(value, str) and re.fullmatch('[0-9a-f]{64}', value) is not None


def observation_context(value):
    """验证观察姿态合同，不把图像中心转换成夹持中心。"""
    require(isinstance(value, dict), '缺少观察上下文')
    for key in ('camera_id', 'pose_id', 'target_plane', 'view'):
        require(isinstance(value.get(key), str) and 0 < len(value[key]) <= 128,
                '观察上下文字段无效：' + key)
    for key in ('camera_config_sha256', 'pose_sha256'):
        require(sha256_text(value.get(key)), '观察摘要无效：' + key)
    require(type(value.get('pose_revision')) is int and value['pose_revision'] > 0,
            '观察姿态版本无效')
    size = value.get('image_size')
    require(isinstance(size, list) and len(size) == 2 and
            all(type(v) is int and v > 0 for v in size), '相机分辨率无效')
    return copy.deepcopy(value)


def candidate_summary(observation):
    """保留旧观测器结果供离线审查；ready只表示候选，不推断物料事件。"""
    require(isinstance(observation, dict), '观测格式错误')
    return dict(domain='historical_replay', observation=copy.deepcopy(observation),
                candidate_ready=any(observation.get(key) is True for key in
                                    ('local_target_ready', 'ring_target_ready',
                                     'occupancy_observation_ready', 'same_color_target_observed')),
                material_event='UNKNOWN', confirmation_allowed=False)


class CandidateGate:
    """复用FrameEngine观测的任务边界；检查目标与时序，但不制造抓放事件。"""

    def __init__(self, context, task_code, batch, order, mode, color=None, ring=None):
        from .task_code import parse_task
        parsed = parse_task(task_code)
        require(type(batch) is int and batch in (1, 2) and type(order) is int and order in (1, 2, 3),
                '候选任务索引无效')
        require(mode in ('material_local', 'ring_number', 'occupancy'), '未支持的任务候选模式')
        item = parsed[f'batch{batch}'][order - 1]
        if mode != 'ring_number':
            require(type(color) is int and color == item['color_id'], '任务目标颜色不符')
        if mode != 'material_local':
            require(type(ring) is int and ring in (1, 2, 3), '任务目标环号无效')
        self.context = observation_context(context)
        view = self.context['view']
        if mode != 'material_local':
            station = 'rough' if view.startswith('rough:') else 'storage' if view.startswith('stage:') else None
            require(station is not None, '环位视图必须绑定工位')
            expected = item['rough_processing_ring' if station == 'rough' else 'temporary_storage_ring']
            require(ring == expected and view.split(':')[-1] == str(ring), '环号与任务或视图不符')
        self.station = None if mode == 'material_local' else station
        self.code, self.batch, self.order = parsed['raw'], batch, order
        self.mode, self.color, self.ring = mode, color, ring
        self.last_sequence, self.last_capture = -1, None

    def observe(self, envelope, now):
        require(isinstance(envelope, dict) and isinstance(envelope.get('observation'), dict), '候选封装无效')
        value = envelope['observation']
        reasons = []
        if envelope.get('domain') not in ('historical_replay', 'offline_fixture'):
            reasons.append('OFFLINE_SOURCE_REQUIRED')
        if not same_json(envelope.get('context'), self.context):
            reasons.append('OBSERVATION_CONTEXT_MISMATCH')
        if (value.get('task_raw'), value.get('batch'), value.get('order')) != (self.code, self.batch, self.order):
            reasons.append('TASK_TARGET_MISMATCH')
        if type(value.get('batch')) is not int or type(value.get('order')) is not int:
            reasons.append('TASK_INDEX_TYPE_INVALID')
        if self.station is not None and value.get('station') != self.station:
            reasons.append('STATION_MISMATCH')
        stamp, sequence = value.get('captured_monotonic'), value.get('frame_sequence')
        valid_time = (type(stamp) in (int, float) and math.isfinite(stamp) and
                      type(now) in (int, float) and math.isfinite(now) and 0 <= now - stamp <= .25)
        if not valid_time:
            reasons.append('STALE_OR_INVALID_CAPTURE')
        if type(sequence) is not int or sequence <= self.last_sequence or (
                self.last_capture is not None and (not valid_time or stamp <= self.last_capture)):
            reasons.append('DUPLICATE_OR_REORDERED_FRAME')
        # 错误上下文不污染本上下文的帧序列；本流旧帧不能用于下一次确认。
        if same_json(envelope.get('context'), self.context) and type(sequence) is int:
            self.last_sequence = max(self.last_sequence, sequence)
            if valid_time:
                self.last_capture = stamp if self.last_capture is None else max(self.last_capture, stamp)
        raw_reasons = value.get('reasons', [])
        if not isinstance(raw_reasons, list) or raw_reasons:
            reasons.append('OBSERVER_REJECTED')
        target = value.get('target')
        if not isinstance(target, dict):
            target = {}
            reasons.append('TARGET_MISSING')
        if self.mode == 'material_local':
            ready = value.get('local_target_ready') is True
            if type(target.get('color_id')) is not int or target.get('color_id') != self.color:
                reasons.append('WRONG_COLOR')
        elif self.mode == 'ring_number':
            ready = value.get('ring_target_ready') is True
            if type(target.get('position_id')) is not int or target.get('position_id') != self.ring:
                reasons.append('WRONG_RING')
        else:
            ready = value.get('occupancy_observation_ready') is True
            if type(target.get('position_id')) is not int or target.get('position_id') != self.ring:
                reasons.append('WRONG_RING')
            if target.get('occupancy') != 'OCCUPIED' or type(target.get('observed_color_id')) is not int or target.get('observed_color_id') != self.color:
                reasons.append('COLOR_OR_OCCUPANCY_UNKNOWN')
            if target.get('reasons') or not value.get('reference_id'):
                reasons.append('CURRENT_SCENE_EVIDENCE_REQUIRED')
        if not ready:
            reasons.append('CANDIDATE_NOT_READY')
        summary = candidate_summary(value)
        summary.update(candidate_ready=not reasons, reasons=list(dict.fromkeys(reasons)),
                       context=copy.deepcopy(self.context))
        return summary
