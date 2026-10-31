"""线性任务执行器：按步骤表调用机器人原语，维护唯一的物料与计数账本。

机器人原语约定：
- 正常返回表示该步骤已有实物证据确认成功（到站、夹起、放稳）。
- 抛出 StepFailed 表示本物料放弃，但机构已回到空爪运输姿态、底盘已停，可以继续后续步骤。
  取料失败时物料仍留在来源处；放置失败时物料已离开车载料位，该料位视为空。
- 抛出 MissionFault（或任何其他异常）表示安全状态不明，本轮结束，不自动续跑。
"""

import time


class StepFailed(RuntimeError):
    pass


class MissionFault(RuntimeError):
    pass


class Runner:
    def __init__(self, robot, *, clock=time.monotonic, code=None, batches=(1, 2), budget_s=180.,
                 return_reserve_s=25., skip_mechanism=False):
        if budget_s <= 0 or not 0 <= return_reserve_s < budget_s:
            raise ValueError('时间预算或返航预留无效')
        self.robot, self.clock = robot, clock
        self.given_code, self.batches = code, tuple(batches)
        self.budget_s, self.return_reserve_s = budget_s, return_reserve_s
        self.skip_mechanism = skip_mechanism
        self.code = None
        self.items = {}
        self.counts = dict(grabs=0, places=0)
        self.events = []
        self.status = 'READY'
        self.reason = ''
        self.started = None

    def elapsed(self):
        return self.clock() - self.started

    def run(self):
        from .plan import build_steps, item_id
        if self.status != 'READY':
            raise RuntimeError('执行器只能运行一次')
        self.status = 'RUNNING'
        self.started = self.clock()
        try:
            zone = self.robot.start_zone()
            self.attempt('survey', dict(kind='survey'), self.robot.survey)
            if self.given_code is None:
                self.call(dict(kind='goto', station='qr', batch=0), self.robot.goto, 'qr')
                self.code = self.call(dict(kind='read_task'), self.robot.read_task)
            else:
                self.code = self.given_code
            task, steps = build_steps(self.code, zone, self.batches)
            self.code = task['raw']
            for batch in (1, 2):
                for item in task[f'batch{batch}']:
                    self.items[item_id(batch, item['order'])] = dict(color=item['color_id'], location='raw')
            self.show()
            final = steps[-1]
            for step in steps[:-1]:
                if self.elapsed() + self.return_reserve_s >= self.budget_s:
                    self.status = 'RETURNED_EARLY'
                    self.reason = f'剩余时间不足，第{len(self.events)}个事件后提前返航'
                    break
                self.execute(step)
            self.call(final, self.robot.goto, final['station'])
            if self.status == 'RUNNING':
                self.status = 'COMPLETE'
        except Exception as error:
            self.status = 'ABORTED'
            self.reason = f'{type(error).__name__}: {error}'
        return self.report()

    def execute(self, step):
        from .plan import item_id
        kind = step['kind']
        if kind == 'goto':
            self.call(step, self.robot.goto, step['station'])
            return
        if self.skip_mechanism:
            return
        identity = item_id(step['batch'], step['slot'])
        item = self.items[identity]
        tray = f'tray:{step["slot"]}'
        if kind == 'pick_raw':
            if item['location'] == 'raw' and self.attempt(identity, step, self.robot.pick_raw,
                                                          step['color'], step['slot']):
                self.grabbed(identity, tray)
        elif kind == 'pick_ring':
            source = f'{step["area"]}:{step["ring"]}'
            if item['location'] == source and self.attempt(identity, step, self.robot.pick_ring,
                                                           step['slot'], step['area'], step['ring']):
                self.grabbed(identity, tray)
        elif kind == 'place':
            if item['location'] != tray:
                return
            layer = step['layer']
            if layer == 2 and not self.first_layer_ready(identity, step['ring']):
                layer = 1  # 同色第一层缺失时退为平面放置，不在空位上码垛
            if self.attempt(identity, dict(step, layer=layer), self.robot.place,
                            step['slot'], step['area'], step['ring'], layer):
                item['location'] = f'{step["area"]}:{step["ring"]}' + (':2' if layer == 2 else '')
                self.counts['places'] += 1
                self.show()
        else:
            raise ValueError(f'未知步骤{kind}')

    def first_layer_ready(self, identity, ring):
        color = self.items[identity]['color']
        return any(other['location'] == f'stage:{ring}' and other['color'] == color
                   for key, other in self.items.items() if key != identity)

    def grabbed(self, identity, tray):
        self.items[identity]['location'] = tray
        self.counts['grabs'] += 1
        self.show()

    def call(self, step, action, *args):
        """必须成功的步骤；失败直接结束本轮。"""
        started = self.clock()
        try:
            result = action(*args)
        except Exception as error:
            self.log(step, started, False, f'{type(error).__name__}: {error}')
            raise
        self.log(step, started, True)
        return result

    def attempt(self, identity, step, action, *args):
        """可放弃的步骤：StepFailed 只放弃该物料，其他异常照常结束本轮。"""
        started = self.clock()
        try:
            action(*args)
        except StepFailed as error:
            self.log(step, started, False, str(error))
            if identity in self.items:
                self.items[identity]['location'] = 'lost'
            return False
        except Exception as error:
            self.log(step, started, False, f'{type(error).__name__}: {error}')
            raise
        self.log(step, started, True)
        return True

    def show(self):
        try:
            self.robot.show(self.code, self.counts['grabs'], self.counts['places'])
        except StepFailed:
            pass  # 屏幕失败不影响搬运；下一次计数变化再刷新

    def log(self, step, started, ok, reason=''):
        self.events.append(dict(step=dict(step), started_s=round(started - self.started, 3),
                                duration_s=round(self.clock() - started, 3), ok=ok, reason=reason))

    def report(self):
        return dict(status=self.status, reason=self.reason, code=self.code, counts=dict(self.counts),
                    elapsed_s=round(self.elapsed(), 3), items={k: dict(v) for k, v in self.items.items()},
                    events=list(self.events))
