"""模拟机器人：独立维护物料真值，用于离线验证执行器流程、计数与时间预算。"""

import argparse
import json

from .plan import item_id
from .runner import MissionFault, Runner, StepFailed
from ..vision.task_code import parse_task

# 各原语的名义耗时（秒）；只用于流程与预算推演，不代表实测。
DURATIONS = dict(survey=3., goto=6., read_task=1.5, pick_raw=4., pick_ring=3.5, place=3.5, show=0.)


class SimClock:
    def __init__(self):
        self.value = 0.

    def __call__(self):
        return self.value

    def advance(self, seconds):
        self.value += seconds


class SimRobot:
    def __init__(self, code, *, zone=1, clock=None, durations=None, failures=(), faults=()):
        """failures/faults 为 (原语名, 键) 集合；键为物料编号或工位名，命中时分别抛 StepFailed/MissionFault。"""
        self.task = parse_task(code)
        self.zone = zone
        self.clock = clock or SimClock()
        self.durations = dict(DURATIONS, **(durations or {}))
        self.failures, self.faults = set(failures), set(faults)
        self.station = f'start:{zone}'
        self.colors = {item_id(b, i['order']): i['color_id'] for b in (1, 2) for i in self.task[f'batch{b}']}
        # 转盘上按批次投放：第一批取完后才放第二批，同色两件靠批次区分。
        self.turntable = {key: self.colors[key] for key in self.colors if key.startswith('b1-')}
        self.raw_visits = 0
        self.tray = {1: None, 2: None, 3: None}
        self.rough = {}
        self.stage = {}
        self.display = None
        self.trace = []

    def spend(self, name, key):
        self.clock.advance(self.durations[name])
        self.trace.append((name, key))
        if (name, key) in self.faults:
            raise MissionFault(f'模拟故障：{name} {key}')
        if (name, key) in self.failures:
            raise StepFailed(f'模拟失败：{name} {key}')

    def start_zone(self):
        return self.zone

    def survey(self):
        self.spend('survey', 'lidar')

    def goto(self, station):
        self.spend('goto', station)
        self.station = station
        if station == 'raw':
            self.raw_visits += 1
            if self.raw_visits == 2:  # 裁判在第二次到达前换上第二批，未取走的第一批物料不再计入
                self.turntable = {key: self.colors[key] for key in self.colors if key.startswith('b2-')}

    def read_task(self):
        self.require('qr')
        self.spend('read_task', 'qr')
        return self.task['raw']

    def show(self, code, grabs, places):
        self.display = (code, grabs, places)

    def pick_raw(self, color, slot):
        self.require('raw')
        key = next((k for k, c in self.turntable.items() if c == color), None)
        if key is None or self.tray[slot] is not None:
            raise MissionFault('转盘上没有该颜色或车载料位被占用')
        self.spend('pick_raw', key)
        self.tray[slot] = self.turntable.pop(key)

    def pick_ring(self, slot, area, ring):
        self.require(area)
        if area != 'rough' or ring not in self.rough or self.tray[slot] is not None:
            raise MissionFault('粗加工环上没有物料或车载料位被占用')
        self.spend('pick_ring', f'{area}:{ring}')
        self.tray[slot] = self.rough.pop(ring)

    def place(self, slot, area, ring, layer):
        self.require(area)
        color = self.tray[slot]
        if color is None:
            raise MissionFault('车载料位为空')
        self.tray[slot] = None  # 放置失败时物料已离开料盘（掉落或偏离），按丢失处理
        self.spend('place', f'{area}:{ring}')
        if area == 'rough':
            if ring in self.rough:
                raise MissionFault('粗加工环已被占用')
            self.rough[ring] = color
        else:
            stack = self.stage.setdefault(ring, [])
            if len(stack) != layer - 1 or (stack and stack[0] != color):
                raise MissionFault('暂存区层数或码垛颜色不符')
            stack.append(color)

    def require(self, station):
        if self.station != station:
            raise MissionFault(f'当前在{self.station}，不在{station}')


def main():
    parser = argparse.ArgumentParser(description='模拟一整轮比赛流程，输出计数与分段耗时')
    parser.add_argument('code', help='任务码，例如 156+123+516+231')
    parser.add_argument('--zone', type=int, choices=(1, 2), default=1)
    parser.add_argument('--given-code', action='store_true', help='跳过扫码，直接使用给定任务码')
    parser.add_argument('--fail', action='append', default=[], help='注入单步失败，格式 原语:键，如 pick_raw:b1-2')
    args = parser.parse_args()
    failures = [tuple(value.split(':', 1)) for value in args.fail]
    robot = SimRobot(args.code, zone=args.zone, failures=failures)
    report = Runner(robot, clock=robot.clock, code=args.code if args.given_code else None).run()
    print(json.dumps(report, ensure_ascii=False, indent=1))


if __name__ == '__main__':
    main()
