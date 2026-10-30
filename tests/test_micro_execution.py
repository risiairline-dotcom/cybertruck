import math
import unittest
import struct
import test_execution as native
from robot.device.arrival import micro_goal


class MicroExecutionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        native.ExecutionTests.setUpClass()
        cls.lib=native.ExecutionTests.lib

    def test_four_small_directions_and_turns_use_real_controller_and_zero_ack(self):
        cases=[(a,m) for a in ('x','y') for m in (-5,-1,1,5)]+[('yaw',d) for d in (-3,-.5,.5,3)]
        for axis,mm in cases:
            with self.subTest(axis=axis,mm=mm):
                d=native.SimTransport(self.lib);d.open()
                profile=native.profile();profile.update(version=10,max_rpm=6)
                d.assert_ok(d.call(native.CONFIG,native.config_payload(profile)))
                d.sleep(.1);d.assert_ok(d.call(native.ARM));d.sleep(.1)
                d.assert_ok(d.call(native.GOAL,micro_goal(1,axis,mm)))
                complete=False
                for n in range(900):
                    if n%10==0:d.call(native.p.HEART,struct.pack('<I',n+1))
                    d.sleep(.01)
                    state=native.STATUS_DATA.unpack(d.call(native.STATUS).payload[1:])
                    self.assertEqual(state[1],0)
                    if state[3]==4:
                        complete=True;break
                self.assertTrue(complete)
                if axis=='yaw':
                    self.assertAlmostEqual(d.plant.yaw,math.radians(mm),delta=math.radians(.2))
                    self.assertLess(math.hypot(d.plant.x,d.plant.y),.001)
                else:
                    value=d.plant.x if axis=='x' else d.plant.y
                    self.assertAlmostEqual(value,mm/1000,delta=.001)
                self.assertGreaterEqual(d.position_count,1)

    def test_old_version_does_not_gain_one_millimeter_goal(self):
        d=native.Device(self.lib);d.arm()
        self.assertNotEqual(d.call(native.GOAL,micro_goal(1,'x',1)).payload[0],native.p.OK)
