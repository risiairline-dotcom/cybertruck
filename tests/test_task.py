"""任务规则、证据、事务和恢复回归；不导入真实设备。"""

import copy
import hashlib
import itertools
import os
from pathlib import Path
import sqlite3
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pi'))

from robot.ledger import Ledger, LedgerError, canonical
from robot.task import Task, TaskError, build_plan, initial, phases, reduce
from robot.task_sim import TaskDemo
from robot.vision.task_code import TaskLatch, parse_task

CODE = '136+321+631+312'


class CodeTests(unittest.TestCase):
    def test_success_case_preserves_four_group_meaning(self):
        task = parse_task(CODE)
        self.assertEqual([item['rough_processing_ring'] for item in task['batch2']], [3, 1, 2])
        self.assertEqual([item['temporary_storage_ring'] for item in task['batch2']], [1, 2, 3])
        self.assertEqual(parse_task(' ' + CODE + '\n'), task)

    def test_all_25920_legal_codes_and_plans(self):
        positions = list(itertools.permutations('123'))
        count = 0
        for colors in itertools.combinations('123456', 3):
            for first in itertools.permutations(colors):
                for second in itertools.permutations(colors):
                    for first_pos in positions:
                        for second_pos in positions:
                            code = '+'.join(map(''.join, (first, first_pos, second, second_pos)))
                            task = parse_task(code)
                            mapping = dict(zip(first, map(int, first_pos)))
                            self.assertEqual([item['temporary_storage_ring'] for item in task['batch2']],
                                             [mapping[color] for color in second])
                            zone = count % 2 + 1
                            plan = build_plan(task, zone)
                            self.assertEqual(len(plan), 31)
                            self.assertEqual(sum(goal['kind'] == 'PICK' for goal in plan), 12)
                            self.assertEqual(sum(goal['kind'] == 'PLACE' for goal in plan), 12)
                            self.assertEqual(plan[-1]['station'], f'start:{zone}')
                            count += 1
        self.assertEqual(count, 25920)

    def test_invalid_formats_colors_and_positions(self):
        for code in (None, 123, '', CODE + '+1', '111+123+111+123', '136+112+631+312',
                     '136+321+632+312', '137+321+731+312', '１３６+321+631+312', 'x' * 65):
            with self.subTest(code=code), self.assertRaises(ValueError):
                parse_task(code)

    def test_latch_requires_distinct_fresh_spanning_frames(self):
        latch = TaskLatch()
        def observe(frame_id, captured, now=None, code=CODE):
            return latch.observe([code], captured if now is None else now,
                                 frame_id=frame_id, captured_s=captured, stream='camera-1')
        self.assertEqual(observe(1, 0)[0], 'CONFIRMING')
        self.assertEqual(observe(1, 0)[0], 'STALE')
        self.assertEqual(observe(2, .01)[0], 'CONFIRMING')
        self.assertEqual(observe(3, .02)[0], 'CONFIRMING')
        self.assertEqual(observe(4, .1)[0], 'ACCEPTED')
        self.assertEqual(observe(5, .2, code='123+123+321+321'), ('ACCEPTED', None))
        self.assertEqual(latch.accepted['raw'], CODE)

    def test_latch_rejects_old_future_ambiguous_and_slow_windows(self):
        latch = TaskLatch()
        for now, captured in ((1, 0), (0, 1), (float('nan'), 0)):
            self.assertEqual(latch.observe([CODE], now, frame_id=1, captured_s=captured,
                                           stream='camera-1')[0], 'STALE')
        self.assertEqual(latch.observe([CODE, '123+123+321+321'], 0, frame_id=2,
                                      captured_s=0, stream='camera-1')[0], 'AMBIGUOUS')
        for index, stamp in enumerate((.1, .5, .9), start=3):
            self.assertEqual(latch.observe([CODE], stamp, frame_id=index, captured_s=stamp,
                                           stream='camera-1')[0], 'CONFIRMING')


class TaskTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.runtime = Path(self.directory.name)
        self.demo = TaskDemo(self.runtime, 'owner-a', CODE)

    def tearDown(self):
        self.demo.close()
        self.directory.cleanup()

    def advance_until(self, predicate):
        for _ in range(1024):
            state = self.demo.task.state
            if predicate(state):
                return state
            self.demo.step(force=True)
        self.fail('模拟未到达指定状态')

    def first_transfer(self, kind='PICK'):
        return self.advance_until(lambda state: state['pending'] is not None and
                                  state['pending']['goal']['kind'] == kind and
                                  state['pending']['status'] == 'RUNNING' and
                                  not state['pending']['checks'])

    def phase_frames(self, phase):
        action = self.demo.task.state['pending']
        self.demo.task.pose(action['id'], phase)
        return self.demo.world.frames_for(self.demo.task, self.demo.clock, phase)

    def test_complete_inventory_mapping_counts_and_replay(self):
        state = self.advance_until(lambda value: value['status'] == 'COMPLETE')
        self.assertEqual(state['counts'], dict(grabs=12, places=12))
        self.assertEqual(state['station'], 'start:1')
        self.assertEqual(state['step'], 34)
        self.assertTrue(all(value is None for value in state['tray'].values()))
        self.assertEqual(state['first_layer_by_color'], {'1': 3, '3': 2, '6': 1})
        for ring, top in state['upper'].items():
            self.assertEqual(state['items'][top]['color'], state['items'][state['lower'][ring]]['color'])
        self.assertEqual(self.demo.ledger.replay(state['run_id'], reduce), state)
        self.demo.ledger.audit(state['run_id'], reduce)

    def test_acceptance_and_execution_do_not_increment_success(self):
        state = self.first_transfer()
        task = self.demo.task
        identity = state['pending']['id']
        task.action('EXECUTED', identity, result=self.demo.world.result(task))
        self.assertEqual(task.state['counts'], dict(grabs=0, places=0))
        with self.assertRaises(TaskError):
            task.action('CONFIRM', identity)
        for phase in ('EMPTY', 'SOURCE', 'HELD', 'STORED'):
            task.checkpoint(identity, phase, self.phase_frames(phase))
        task.action('CONFIRM', identity)
        self.assertEqual(task.state['counts'], dict(grabs=1, places=0))

    def test_duplicate_confirmation_and_conflicting_retries(self):
        state = self.advance_until(lambda value: value['counts']['grabs'] == 1 and value['pending'] is None)
        identity = state['last_action']['id']
        revision = state['revision']
        self.demo.task.action('CONFIRM', identity)
        self.assertEqual(self.demo.task.state['revision'], revision)
        with self.assertRaises(LedgerError):
            self.demo.task.action('CONFIRM', identity, altered=True)
        with self.assertRaises(TaskError):
            self.demo.task.emit('CONFIRM', dict(action_id=identity), 'new-duplicate-event')
        self.assertEqual(self.demo.task.state['counts']['grabs'], 1)

    def test_invalid_observations_are_atomic_and_do_not_count(self):
        state = self.first_transfer()
        identity = state['pending']['id']
        frames = self.phase_frames('EMPTY')
        baseline = self.demo.task.state
        variants = [dict(domain='physical'), dict(run_id='other'), dict(session_id='old'),
                    dict(action_id='late'), dict(pose_epoch=0), dict(camera='other'),
                    dict(calibration='old'), dict(view='tray:2'), dict(state='unknown'),
                    dict(quality=.1), dict(captured_s=-1), dict(captured_s=999),
                    dict(frame_id=0), dict(color=1)]
        for changes in variants:
            invalid = copy.deepcopy(frames)
            invalid[-1].update(changes)
            with self.subTest(changes=changes), self.assertRaises(TaskError):
                self.demo.task.checkpoint(identity, 'EMPTY', invalid)
            self.assertEqual(self.demo.task.state, baseline)
        self.demo.task.checkpoint(identity, 'EMPTY', frames)
        self.assertEqual(self.demo.task.state['counts']['grabs'], 0)

    def test_wrong_color_and_repeated_frame_are_rejected(self):
        state = self.first_transfer()
        identity = state['pending']['id']
        self.demo.task.checkpoint(identity, 'EMPTY', self.phase_frames('EMPTY'))
        frames = self.phase_frames('SOURCE')
        frames[1]['color'] = 5
        with self.assertRaises(TaskError):
            self.demo.task.checkpoint(identity, 'SOURCE', frames)
        frames[1]['color'] = frames[0]['color']
        frames[1]['frame_id'] = frames[0]['frame_id']
        with self.assertRaises(TaskError):
            self.demo.task.checkpoint(identity, 'SOURCE', frames)
        self.assertEqual(self.demo.task.state['counts']['grabs'], 0)

    def test_partial_grip_cancel_preserves_unknown_and_does_not_release(self):
        state = self.first_transfer()
        identity = state['pending']['id']
        for phase in ('EMPTY', 'SOURCE', 'HELD'):
            self.demo.task.checkpoint(identity, phase, self.phase_frames(phase))
        held = self.demo.world.held
        self.demo.task.abort('人工结束模拟')
        state = self.demo.task.state
        self.assertEqual(state['status'], 'ABORTED')
        self.assertEqual(state['pending']['status'], 'UNKNOWN')
        self.assertEqual(state['tray']['1'], 'UNKNOWN')
        self.assertEqual(state['counts'], dict(grabs=0, places=0))
        self.assertEqual(self.demo.world.held, held)

    def test_placement_requires_release_validity_and_stability(self):
        state = self.first_transfer('PLACE')
        identity = state['pending']['id']
        for phase in ('TARGET', 'HELD', 'RELEASED'):
            self.demo.task.checkpoint(identity, phase, self.phase_frames(phase))
        frames = self.phase_frames('PLACED')
        for field in ('claw_empty', 'placement_valid', 'stable'):
            invalid = copy.deepcopy(frames)
            invalid[1][field] = False
            with self.subTest(field=field), self.assertRaises(TaskError):
                self.demo.task.checkpoint(identity, 'PLACED', invalid)
        self.assertEqual(self.demo.task.state['counts'], dict(grabs=3, places=0))

    def test_wrong_ring_and_phase_order_are_rejected(self):
        state = self.first_transfer('PLACE')
        identity = state['pending']['id']
        with self.assertRaises(TaskError):
            self.demo.task.pose(identity, 'RELEASED')
        frames = self.phase_frames('TARGET')
        frames[0]['view'] = 'rough:wrong'
        with self.assertRaises(TaskError):
            self.demo.task.checkpoint(identity, 'TARGET', frames)

    def test_reboot_blocks_pending_action_and_keeps_prior_counts(self):
        self.advance_until(lambda value: value['counts']['grabs'] == 1 and value['pending'] is None)
        self.demo.task.intent()
        old = self.demo.task.state
        self.demo.close()
        self.demo = TaskDemo(self.runtime, 'owner-new', CODE)
        state = self.demo.task.state
        self.assertFalse(self.demo.enabled)
        self.assertEqual(state['run_id'], old['run_id'])
        self.assertEqual(state['status'], 'RECONCILE_REQUIRED')
        self.assertEqual(state['counts'], dict(grabs=1, places=0))
        revision = state['revision']
        self.demo.step(force=True)
        self.assertEqual(self.demo.task.state['revision'], revision)
        with self.assertRaises(TaskError):
            self.demo.task.intent()

    def test_deadline_and_clock_rollback_end_run(self):
        self.first_transfer()
        self.demo.clock.value = 181
        self.demo.task.tick()
        self.assertEqual(self.demo.task.state['status'], 'ABORTED')
        self.assertEqual(self.demo.task.state['counts'], dict(grabs=0, places=0))

    def test_clock_rollback_ends_run(self):
        self.first_transfer()
        self.demo.clock.value = -1
        self.demo.task.tick()
        self.assertEqual(self.demo.task.state['status'], 'ABORTED')

    def test_second_layer_requires_same_color_bottom(self):
        state = self.advance_until(lambda value: value['pending'] is not None and
                                   value['pending']['goal'].get('layer') == 2 and
                                   value['pending']['status'] == 'RUNNING')
        identity = state['pending']['id']
        ring = str(state['pending']['goal']['ring'])
        bottom = self.demo.world.lower[ring]
        self.demo.world.items[bottom]['color'] = 5
        frames = self.phase_frames('TARGET')
        with self.assertRaises(TaskError):
            self.demo.task.checkpoint(identity, 'TARGET', frames)
        self.assertEqual(self.demo.task.state['counts'], dict(grabs=12, places=9))

    def test_lost_link_ends_started_demo_without_replay(self):
        self.first_transfer()
        self.demo.step(force=True, link_ok=False)
        state = self.demo.task.state
        self.assertEqual(state['status'], 'ABORTED')
        self.demo.step(force=True, link_ok=True)
        self.assertEqual(self.demo.task.state['revision'], state['revision'])

    def test_failed_action_waits_for_reconciliation(self):
        state = self.first_transfer()
        self.demo.task.action('FAILED', state['pending']['id'], reason='模拟反馈失效')
        self.assertEqual(self.demo.task.state['status'], 'RECONCILE_REQUIRED')
        self.assertEqual(self.demo.task.state['counts']['grabs'], 0)

    def test_unknown_result_is_kept_distinct_from_failure(self):
        state = self.first_transfer()
        action_id = state['pending']['id']
        self.demo.task.action('UNKNOWN', action_id, reason='回复丢失')
        state = self.demo.task.state
        self.assertEqual(state['status'], 'RECONCILE_REQUIRED')
        self.assertEqual(state['counts']['grabs'], 0)
        self.assertEqual(state['pending']['status'], 'UNKNOWN')
        with self.assertRaises(TaskError):
            self.demo.task.action('FAILED', action_id, reason='重复结束')

    def test_old_success_replayed_after_recovery_does_not_count_again(self):
        state = self.advance_until(lambda value: value['counts']['grabs'] == 1 and value['pending'] is None)
        event = self.demo.ledger.events(state['run_id'])[-1]
        self.demo.close()
        self.demo = TaskDemo(self.runtime, 'new-session')
        state = self.demo.task.state
        self.demo.ledger.append(state['run_id'], event['event_id'], event['kind'], event['payload'],
                                self.demo.clock(), reduce)
        self.assertEqual(self.demo.task.state, state)


class LedgerTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.path = Path(self.directory.name) / 'task.sqlite3'
        self.ledger = Ledger(self.path)
        self.state = initial('run', 'owner', 1)
        self.ledger.create(self.state, 0)

    def tearDown(self):
        self.ledger.close()
        self.directory.cleanup()

    def test_projection_failure_rolls_back_event_insert(self):
        self.ledger.db.executescript("CREATE TRIGGER fail_update BEFORE UPDATE ON runs BEGIN SELECT RAISE(ABORT,'injected'); END;")
        before = self.ledger.load('run')
        with self.assertRaises(sqlite3.IntegrityError):
            self.ledger.append('run', 'start', 'START', dict(session_id='owner', trigger='sim_button'), 1, reduce)
        self.assertEqual(self.ledger.load('run'), before)
        self.assertEqual(len(self.ledger.events('run')), 1)

    def test_only_one_unfinished_run(self):
        with self.assertRaises(LedgerError):
            self.ledger.create(initial('another', 'owner2', 2), 0)
        self.assertEqual(len(self.ledger.events('run')), 1)

    def test_audit_detects_modified_projection(self):
        state = self.ledger.load('run')
        state['counts']['grabs'] = 99
        self.ledger.db.execute('UPDATE runs SET state=? WHERE id=?', (canonical(state), 'run'))
        with self.assertRaises(LedgerError):
            self.ledger.audit('run', reduce)

    def test_unknown_schema_is_preserved(self):
        other = self.path.with_name('future.sqlite3')
        db = sqlite3.connect(other)
        db.execute('PRAGMA user_version=99')
        db.close()
        before = hashlib.sha256(other.read_bytes()).hexdigest()
        with self.assertRaises(LedgerError):
            Ledger(other)
        self.assertEqual(hashlib.sha256(other.read_bytes()).hexdigest(), before)


class IntegrationTests(unittest.TestCase):
    def test_both_starts_new_run_reset_and_all_colors(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Path(directory)
            previous_id = None
            for zone, code in ((1, CODE), (2, '245+123+542+321')):
                demo = TaskDemo(runtime, f'owner-{zone}', code, zone)
                try:
                    self.assertEqual(demo.task.state['counts'], dict(grabs=0, places=0))
                    for _ in range(1024):
                        demo.step(force=True)
                        if demo.task.state['status'] == 'COMPLETE':
                            break
                    state = demo.task.state
                    self.assertEqual(state['status'], 'COMPLETE')
                    self.assertEqual(state['station'], f'start:{zone}')
                    self.assertEqual(state['counts'], dict(grabs=12, places=12))
                    self.assertNotEqual(state['run_id'], previous_id)
                    previous_id = state['run_id']
                    demo.ledger.audit(state['run_id'], reduce)
                finally:
                    demo.close()

    def test_abrupt_process_exit_recovers_without_autorun(self):
        with tempfile.TemporaryDirectory() as directory:
            code = """
import os,sys
from robot.task_sim import TaskDemo
demo=TaskDemo(sys.argv[1],'old-core','136+321+631+312')
for unused in range(256):
    demo.step(force=True)
    state=demo.task.state
    if state['pending'] and state['pending']['goal']['kind']=='PICK' and state['pending']['status']=='RUNNING':
        os._exit(0)
raise RuntimeError('no pending action')
"""
            env = dict(os.environ, PYTHONPATH=str(ROOT / 'pi'))
            subprocess.run([sys.executable, '-B', '-c', code, directory], env=env, check=True, timeout=10)
            demo = TaskDemo(Path(directory), 'new-core', CODE)
            try:
                self.assertEqual(demo.task.state['status'], 'RECONCILE_REQUIRED')
                self.assertEqual(demo.task.state['pending']['status'], 'UNKNOWN')
                self.assertEqual(demo.task.state['counts'], dict(grabs=0, places=0))
                self.assertFalse(demo.enabled)
                demo.ledger.audit(demo.task.run_id, reduce)
            finally:
                demo.close()

    def test_core_web_snapshot_contains_only_simulation_counts(self):
        from robot.runtime import load_snapshot
        with tempfile.TemporaryDirectory() as directory:
            command = [sys.executable, '-m', 'robot.main', '--mode', 'sim', '--task-demo', CODE,
                       '--runtime', directory]
            core = subprocess.Popen(command, cwd=ROOT / 'pi', stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            try:
                deadline = time.monotonic() + 5
                state = None
                while time.monotonic() < deadline:
                    state = load_snapshot(Path(directory) / 'state.json')
                    if (state.get('task') or {}).get('status') == 'RUNNING':
                        break
                    time.sleep(.05)
                self.assertEqual(state['task']['domain'], 'simulation')
                self.assertIsNone(state['grabs'])
                self.assertIsNone(state['places'])
                self.assertFalse(state['ready'])
            finally:
                core.terminate()
                core.communicate(timeout=5)


if __name__ == '__main__':
    unittest.main(verbosity=2)
