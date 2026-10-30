"""尺寸无关的任务决策与状态投影；当前只接收模拟域证据，没有硬件输出。"""

import copy
import math
import uuid

from .task_check import TaskError, check_frames, phases, require
from .vision.task_code import parse_task

TERMINAL = ('COMPLETE', 'ABORTED')


def build_plan(task, start_zone):
    """语义工位与取放顺序；不包含未经标定的坐标、行程或速度。"""
    plan = []
    for batch in (1, 2):
        items = task[f'batch{batch}']
        prefix = f'B{batch}'
        plan.append(dict(kind='NAV', stage=f'{prefix}_RAW_NAV', station='raw', batch=batch))
        for item in items:
            slot, identity = item['order'], f'b{batch}-{item["order"]}'
            plan.append(dict(kind='PICK', stage=f'{prefix}_RAW_PICK', batch=batch, item=identity,
                             slot=slot, source='raw', destination=f'tray:{slot}'))
        plan.append(dict(kind='NAV', stage=f'{prefix}_ROUGH_NAV', station='rough', batch=batch))
        for kind in ('PLACE', 'PICK'):
            for item in items:
                slot, ring = item['order'], item['rough_processing_ring']
                tray, target = f'tray:{slot}', f'rough:{ring}'
                plan.append(dict(kind=kind, stage=f'{prefix}_ROUGH_{kind}', batch=batch,
                                 item=f'b{batch}-{slot}', slot=slot, ring=ring,
                                 source=tray if kind == 'PLACE' else target,
                                 destination=target if kind == 'PLACE' else tray))
        plan.append(dict(kind='NAV', stage=f'{prefix}_STAGE_NAV', station='stage', batch=batch))
        for item in items:
            slot, ring = item['order'], item['temporary_storage_ring']
            plan.append(dict(kind='PLACE', stage=f'{prefix}_STAGE_PLACE', batch=batch,
                             item=f'b{batch}-{slot}', slot=slot, ring=ring, layer=batch,
                             source=f'tray:{slot}', destination=f'stage:{ring}'))
    plan.append(dict(kind='NAV', stage='RETURN', station=f'start:{start_zone}', batch=0))
    return plan


def initial(run_id, owner, start_zone):
    require(start_zone in (1, 2) and type(start_zone) is int, '启停区必须为1或2')
    require(isinstance(owner, str) and 0 < len(owner) <= 64, '任务控制会话不能为空')
    return dict(run_id=run_id, domain='simulation', owner=owner, start_zone=start_zone,
                status='READY', code=None, step=0, started_s=None, elapsed_s=0, last_at_s=0,
                station=f'start:{start_zone}', base_stopped=True, transport_safe=True,
                claw_empty=True, pose_serial=0, last_frames={}, pending=None, last_action=None, used_actions=[],
                counts=dict(grabs=0, places=0), items={}, tray={'1': None, '2': None, '3': None},
                rough={}, lower={}, upper={}, first_layer_by_color={}, reason='',
                plan=[dict(kind='SCAN', stage='START_SCAN', station=f'start:{start_zone}', batch=0),
                      dict(kind='NAV', stage='QR_NAV', station='qr', batch=0),
                      dict(kind='QR', stage='SCAN_CODE', station='qr', batch=0)])


def active(state, action_id):
    action = state['pending']
    require(action is not None and action['id'] == action_id, '结果不属于当前动作')
    return action


def precondition(state, goal):
    require(state['claw_empty'] is True and state['transport_safe'] is True, '夹持或运输姿态状态不明确')
    if goal['kind'] == 'NAV':
        require(all(value != 'UNKNOWN' for value in state['tray'].values()), '料位存在未知状态')
        require(all(item['location'] != 'UNKNOWN' for item in state['items'].values()), '物料位置存在未知状态')
        return
    require(state['base_stopped'] is True, '底盘未停稳')
    if goal['kind'] not in ('PICK', 'PLACE'):
        require(state['station'] == goal['station'], '工位不匹配')
        return
    item = state['items'][goal['item']]
    require(item['location'] == goal['source'], '物料不在预期来源位置')
    station = goal['destination'].split(':')[0] if goal['kind'] == 'PLACE' else goal['source'].split(':')[0]
    require(state['station'] == station, '不在正确取放工位')
    slot = str(goal['slot'])
    if goal['kind'] == 'PICK':
        require(state['tray'][slot] is None, '目标车载料位未确认空置')
        if station == 'rough':
            require(state['rough'].get(str(goal['ring'])) == goal['item'], '粗加工物料身份不符')
    else:
        require(state['tray'][slot] == goal['item'], '车载料位与物料不符')
        ring = str(goal['ring'])
        if station == 'rough':
            require(ring not in state['rough'], '粗加工目标未确认空置')
        elif goal['layer'] == 1:
            require(ring not in state['lower'], '第一层目标已占用')
        else:
            bottom = state['lower'].get(ring)
            require(bottom is not None and state['items'][bottom]['color'] == item['color'] and
                    state['first_layer_by_color'].get(str(item['color'])) == goal['ring'] and
                    ring not in state['upper'], '缺少已确认的同色第一层或码垛目标已占用')


def uncertain(state, action):
    goal = action['goal']
    if goal['kind'] == 'NAV':
        state['base_stopped'] = False
    if goal['kind'] in ('PICK', 'PLACE'):
        state['items'][goal['item']]['location'] = 'UNKNOWN'
        state['tray'][str(goal['slot'])] = 'UNKNOWN'
        state['claw_empty'] = None
        state['transport_safe'] = False


def apply_success(state, action):
    goal = action['goal']
    if goal['kind'] == 'NAV':
        state['station'] = goal['station']
        state['base_stopped'] = True
    elif goal['kind'] == 'QR':
        task = parse_task(action['checks'][0]['code'])
        state['code'] = task['raw']
        for batch in (1, 2):
            for item in task[f'batch{batch}']:
                state['items'][f'b{batch}-{item["order"]}'] = dict(
                    color=item['color_id'], batch=batch, order=item['order'], location='raw')
        state['plan'].extend(build_plan(task, state['start_zone']))
    elif goal['kind'] in ('PICK', 'PLACE'):
        identity, slot = goal['item'], str(goal['slot'])
        state['items'][identity]['location'] = goal['destination']
        state['claw_empty'] = state['transport_safe'] = True
        if goal['kind'] == 'PICK':
            state['tray'][slot] = identity
            if goal['source'].startswith('rough:'):
                del state['rough'][str(goal['ring'])]
            state['counts']['grabs'] += 1
        else:
            state['tray'][slot] = None
            ring = str(goal['ring'])
            if goal['destination'].startswith('rough:'):
                state['rough'][ring] = identity
            elif goal['layer'] == 1:
                state['lower'][ring] = identity
                state['first_layer_by_color'][str(state['items'][identity]['color'])] = goal['ring']
            else:
                state['upper'][ring] = identity
            state['counts']['places'] += 1
    state['last_action'] = dict(id=action['id'], goal=goal, status='CONFIRMED')
    state['pending'] = None
    state['step'] += 1
    if state['step'] == len(state['plan']):
        require(state['counts'] == dict(grabs=12, places=12), '完整任务计数不符')
        require(len(state['lower']) == len(state['upper']) == 3 and
                all(value is None for value in state['tray'].values()), '最终物料分布不符')
        state['status'] = 'COMPLETE'


def reduce(state, kind, payload, at_s):
    if kind == 'RECOVER':
        require(state['status'] not in TERMINAL, '已结束任务不需要恢复')
        state['owner'] = payload['session_id']
        state['status'] = 'RECONCILE_REQUIRED'
        state['reason'] = '核心重启：先对账，不自动续跑或重发动作'
        if state['pending']:
            state['pending']['status'] = 'UNKNOWN'
            uncertain(state, state['pending'])
        return state
    require(payload.get('session_id') == state['owner'], '任务控制会话不匹配')
    if kind == 'ABORT':
        require(state['status'] not in TERMINAL, '任务已结束')
        state['status'] = 'ABORTED'
        state['reason'] = payload['reason']
        if state['pending']:
            state['pending']['status'] = 'UNKNOWN'
            uncertain(state, state['pending'])
        return state
    require(at_s >= state['last_at_s'], '任务时钟倒退，停止推进')
    state['last_at_s'] = at_s
    if kind == 'START':
        require(state['status'] == 'READY' and payload.get('trigger') == 'sim_button', '启动条件不满足')
        state['status'] = 'RUNNING'
        state['started_s'] = at_s
        return state
    require(state['status'] == 'RUNNING', '任务未运行或需要对账')
    state['elapsed_s'] = at_s - state['started_s']
    require(state['elapsed_s'] < 180, '任务超过180秒逻辑期限')
    if kind == 'INTENT':
        require(state['pending'] is None, '已有未结束动作，不允许并发或重复发起')
        require(isinstance(payload.get('action_id'), str) and 0 < len(payload['action_id']) <= 64 and
                payload['action_id'] not in state['used_actions'], '动作编号无效或重复使用')
        goal = copy.deepcopy(state['plan'][state['step']])
        precondition(state, goal)
        state['pending'] = dict(id=payload['action_id'], goal=goal, status='INTENT', checks=[],
                                started_s=at_s, session_id=state['owner'], pose=None)
        state['used_actions'].append(payload['action_id'])
        return state
    action = active(state, payload.get('action_id'))
    if kind == 'ACCEPTED':
        require(action['status'] == 'INTENT', '动作接受顺序错误')
        action['status'] = 'ACCEPTED'
    elif kind == 'RUNNING':
        require(action['status'] == 'ACCEPTED', '动作运行顺序错误')
        action['status'] = 'RUNNING'
        uncertain(state, action)
    elif kind == 'POSE':
        require(action['status'] in ('RUNNING', 'EXECUTED'), '当前不能更新观察姿态')
        rules = phases(action['goal'], state)
        require(len(action['checks']) < len(rules) and payload['phase'] == rules[len(action['checks'])][0],
                '确认阶段顺序错误')
        state['pose_serial'] += 1
        action['pose'] = dict(phase=payload['phase'], epoch=state['pose_serial'], settled_s=at_s)
    elif kind == 'CHECK':
        require(action['status'] in ('RUNNING', 'EXECUTED'), '当前不能接收视觉确认')
        check = check_frames(state, action, payload['frames'], at_s)
        action['checks'].append(check)
        state['last_frames'][check['camera']] = check['last_frame']
    elif kind == 'EXECUTED':
        require(action['status'] == 'RUNNING', '执行结果顺序错误')
        result = payload['result']
        require(isinstance(result, dict), '执行结果格式错误')
        require((result.get('domain'), result.get('run_id'), result.get('session_id'), result.get('action_id')) ==
                ('simulation', state['run_id'], state['owner'], action['id']), '执行结果身份或证据域不匹配')
        require(result.get('ok') is True and result.get('stopped') is True and
                result.get('claw_empty') is True and result.get('transport_safe') is True,
                '执行结果缺少必要完成条件')
        observed = result.get('observed_s')
        require(type(observed) in (int, float) and math.isfinite(observed) and
                action['started_s'] <= observed <= at_s and at_s - observed <= .25,
                '执行结果时间无效')
        goal = action['goal']
        if goal['kind'] in ('NAV', 'SCAN'):
            require(result.get('station') == goal['station'], '到达工位不符')
        if goal['kind'] == 'SCAN':
            require(result.get('complete') is True and result.get('coverage_valid') is True,
                    '扫描不完整或覆盖未知')
        action['result'] = result
        action['status'] = 'EXECUTED'
    elif kind == 'CONFIRM':
        require(action['status'] == 'EXECUTED', '执行结束不能直接冒充证据确认')
        require(len(action['checks']) == len(phases(action['goal'], state)), '分阶段证据尚不完整')
        if action['checks']:
            require(0 <= at_s - action['checks'][-1]['captured_s'] <= .25, '最终确认观测已过期')
        else:
            require(0 <= at_s - action['result']['observed_s'] <= .25, '执行结果已过期')
        apply_success(state, action)
    elif kind in ('FAILED', 'UNKNOWN'):
        require(action['status'] not in ('CONFIRMED', 'UNKNOWN'), '动作已结束')
        action['status'] = 'UNKNOWN' if kind == 'UNKNOWN' else 'FAILED'
        state['status'] = 'RECONCILE_REQUIRED'
        state['reason'] = payload['reason']
        uncertain(state, action)
    else:
        raise TaskError('不支持的任务事件')
    return state


class Task:
    def __init__(self, ledger, run_id, owner, clock):
        self.ledger, self.run_id, self.owner, self.clock = ledger, run_id, owner, clock

    @classmethod
    def create(cls, ledger, owner, clock, start_zone=1):
        previous = ledger.latest()
        require(previous is None or previous['status'] in TERMINAL, '存在未对账任务，不创建新任务')
        run_id = uuid.uuid4().hex
        ledger.create(initial(run_id, owner, start_zone), clock())
        return cls(ledger, run_id, owner, clock)

    @property
    def state(self):
        return self.ledger.load(self.run_id)

    def emit(self, kind, payload=None, event_id=None):
        payload = dict(payload or {}, session_id=self.owner)
        return self.ledger.append(self.run_id, event_id or uuid.uuid4().hex, kind, payload, self.clock(), reduce)

    def start(self):
        return self.emit('START', dict(trigger='sim_button'), 'start')

    def intent(self):
        return self.emit('INTENT', dict(action_id=uuid.uuid4().hex))

    def action(self, kind, action_id, **payload):
        return self.emit(kind, dict(payload, action_id=action_id), f'{action_id}:{kind}')

    def pose(self, action_id, phase):
        return self.emit('POSE', dict(action_id=action_id, phase=phase), f'{action_id}:POSE:{phase}')

    def checkpoint(self, action_id, phase, frames):
        return self.emit('CHECK', dict(action_id=action_id, frames=frames), f'{action_id}:CHECK:{phase}')

    def recover(self):
        return self.emit('RECOVER', event_id=f'recover:{self.owner}')

    def abort(self, reason):
        return self.emit('ABORT', dict(reason=reason))

    def tick(self):
        state = self.state
        if state['status'] == 'RUNNING' and (self.clock() < state['last_at_s'] or
                                           self.clock() - state['started_s'] >= 180):
            return self.abort('逻辑期限耗尽或时钟失效，停止本轮')
        return state

    def snapshot(self):
        state = self.state
        goal = state['pending']['goal'] if state['pending'] else (
            state['plan'][state['step']] if state['step'] < len(state['plan']) else None)
        return dict(run_id=self.run_id, domain='simulation', status=state['status'],
                    stage=None if goal is None else goal['stage'], code=state['code'],
                    start_zone=state['start_zone'], simulation_counts=state['counts'],
                    tray=state['tray'], items=state['items'], lower=state['lower'], upper=state['upper'],
                    first_layer_by_color=state['first_layer_by_color'], reason=state['reason'],
                    elapsed_sim_s=state['elapsed_s'], revision=state['revision'],
                    action=None if state['pending'] is None else dict(
                        id=state['pending']['id'], status=state['pending']['status'],
                        kind=state['pending']['goal']['kind']),
                    confirmed_steps=state['step'], planned_steps=len(state['plan']))
