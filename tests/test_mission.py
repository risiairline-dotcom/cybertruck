import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'pi'))

from robot.mission.plan import build_steps
from robot.mission.runner import Runner
from robot.mission.sim import SimRobot

CODE = '156+123+516+231'


def run(code=CODE, **options):
    runner_options = {k: options.pop(k) for k in ('budget_s', 'return_reserve_s', 'batches', 'skip_mechanism')
                      if k in options}
    given = options.pop('given', False)
    robot = SimRobot(code, **options)
    report = Runner(robot, clock=robot.clock, code=code if given else None, **runner_options).run()
    return robot, report


class PlanTests(unittest.TestCase):
    def test_steps_follow_task_code_order(self):
        _, steps = build_steps(CODE, 2)
        kinds = [s['kind'] for s in steps]
        self.assertEqual(kinds.count('pick_raw'), 6)
        self.assertEqual(kinds.count('pick_ring'), 6)
        self.assertEqual(kinds.count('place'), 12)
        self.assertEqual(steps[-1], dict(kind='goto', station='start:2', batch=0))
        raw = [s['color'] for s in steps if s['kind'] == 'pick_raw']
        self.assertEqual(raw, [1, 5, 6, 5, 1, 6])
        stage2 = [(s['ring'], s['layer']) for s in steps if s['kind'] == 'place' and s['area'] == 'stage'][3:]
        # 第二批黑、红、浅蓝码到第一批同色所在的2、1、3号环
        self.assertEqual(stage2, [(2, 2), (1, 2), (3, 2)])

    def test_rejects_bad_zone_and_batches(self):
        with self.assertRaises(ValueError):
            build_steps(CODE, 3)
        with self.assertRaises(ValueError):
            build_steps(CODE, 1, batches=(2, 1))


class RunnerTests(unittest.TestCase):
    def test_full_run_counts_and_final_layout(self):
        robot, report = run()
        self.assertEqual(report['status'], 'COMPLETE', report['reason'])
        self.assertEqual(report['counts'], dict(grabs=12, places=12))
        self.assertEqual(robot.display, (CODE, 12, 12))
        self.assertEqual(robot.station, 'start:1')
        self.assertEqual(robot.stage, {1: [1, 1], 2: [5, 5], 3: [6, 6]})
        self.assertTrue(all(v is None for v in robot.tray.values()))
        self.assertLess(report['elapsed_s'], 180)

    def test_given_code_skips_qr(self):
        robot, report = run(given=True)
        self.assertEqual(report['status'], 'COMPLETE')
        self.assertNotIn(('goto', 'qr'), robot.trace)

    def test_failed_raw_pick_skips_that_item_only(self):
        robot, report = run(failures=[('pick_raw', 'b1-2')])
        self.assertEqual(report['status'], 'COMPLETE')
        self.assertEqual(report['counts'], dict(grabs=10, places=10))
        self.assertEqual(report['items']['b1-2']['location'], 'lost')
        # 第一批黑色缺失，第二批黑色退为平面放置，不在空环上码垛
        self.assertEqual(robot.stage[2], [5])
        self.assertEqual(report['items']['b2-1']['location'], 'stage:2')

    def test_failed_rough_place_skips_its_pick_back(self):
        robot, report = run(failures=[('place', 'rough:3')])
        self.assertEqual(report['status'], 'COMPLETE')
        # 两批各有一件放粗加工3号环失败：每件少一次取回、少粗加工和暂存两次放置
        self.assertEqual(report['counts'], dict(grabs=10, places=8))
        self.assertEqual(report['items']['b1-3']['location'], 'lost')
        self.assertEqual(report['items']['b2-2']['location'], 'lost')
        self.assertNotIn(('pick_ring', 'rough:3'), robot.trace[:robot.trace.index(('goto', 'stage'))])

    def test_failed_survey_continues(self):
        _, report = run(failures=[('survey', 'lidar')])
        self.assertEqual(report['status'], 'COMPLETE')
        self.assertFalse(report['events'][0]['ok'])

    def test_fault_aborts_without_returning(self):
        robot, report = run(faults=[('pick_ring', 'rough:2')])
        self.assertEqual(report['status'], 'ABORTED')
        self.assertIn('MissionFault', report['reason'])
        self.assertEqual(robot.station, 'rough')
        self.assertEqual(robot.trace[-1], ('pick_ring', 'rough:2'))

    def test_navigation_failure_aborts(self):
        _, report = run(failures=[('goto', 'rough')])
        self.assertEqual(report['status'], 'ABORTED')

    def test_short_budget_returns_home_early(self):
        robot, report = run(budget_s=90)
        self.assertEqual(report['status'], 'RETURNED_EARLY')
        self.assertEqual(robot.station, 'start:1')
        self.assertLessEqual(report['elapsed_s'], 90)
        self.assertGreater(report['counts']['places'], 0)

    def test_first_batch_only(self):
        robot, report = run(batches=(1,))
        self.assertEqual(report['status'], 'COMPLETE')
        self.assertEqual(report['counts'], dict(grabs=6, places=6))
        self.assertEqual(robot.station, 'start:1')

    def test_navigation_only_rehearsal(self):
        robot, report = run(skip_mechanism=True)
        self.assertEqual(report['status'], 'COMPLETE')
        self.assertEqual(report['counts'], dict(grabs=0, places=0))
        self.assertEqual([k for k, _ in robot.trace if k != 'goto'], ['survey', 'read_task'])

    def test_events_record_durations(self):
        _, report = run()
        picks = [e for e in report['events'] if e['step']['kind'] == 'pick_raw']
        self.assertEqual(len(picks), 6)
        self.assertTrue(all(e['duration_s'] == 4. for e in picks))


if __name__ == '__main__':
    unittest.main()
