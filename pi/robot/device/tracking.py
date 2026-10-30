"""正式C跟踪器的离线绑定；没有串口、CAN或设备发送接口。"""

import ctypes as c

from .odometry import Geometry, Stamp, Pose


class Limit(c.Structure):
    _fields_ = [(key, c.c_float) for key in ('velocity', 'acceleration', 'jerk')]


class PathSpec(c.Structure):
    _fields_ = [(key, c.c_float) for key in ('lead_m', 'tail_m', 'radius_m', 'blend_m')] + [
        ('turn', c.c_int), ('reverse', c.c_int)]


class Spec(c.Structure):
    _fields_ = [('kind', c.c_uint32), ('x_m', c.c_float), ('y_m', c.c_float), ('yaw_rad', c.c_float),
                ('path', PathSpec), ('linear', Limit), ('angular', Limit), ('tangent', c.c_uint32)] + [
                    (key,c.c_float) for key in ('path_heading_rad','curve_velocity_m_s','yaw_begin_m',
                                               'yaw_end_m','tail_velocity_m_s')]


class Goal(c.Structure):
    _fields_ = [('id', c.c_uint32), ('timeout_ms', c.c_uint32), ('reference', Spec)]


class Reference(c.Structure):
    _fields_ = [(key, c.c_double) for key in ('x_m', 'y_m', 'yaw_rad', 'vx_m_s', 'vy_m_s',
                                             'wz_rad_s', 'angular_accel_rad_s2', 'angular_jerk_rad_s3',
                                             'ax_m_s2','ay_m_s2','jx_m_s3','jy_m_s3')] + [('done', c.c_uint32)]


class Config(c.Structure):
    _fields_ = ([('geometry', Geometry)] + [(key, c.c_double) for key in (
        'kp_xy', 'kp_yaw', 'kd_gyro', 'correction_m_s', 'correction_rad_s', 'max_rpm', 'slew_rpm_s',
        'position_tol_m', 'yaw_tol_rad', 'still_rpm', 'still_m_s', 'still_rad_s')]
        + [('count_drift', c.c_double * 4)] + [(key, c.c_uint32) for key in (
            'fresh_ms', 'feedback_gap_ms', 'control_gap_ms', 'stable_ms', 'can_epoch', 'imu_epoch',
            'imu_source_epoch', 'calibration')] + [('lag_ff_s', c.c_double), ('micro_steps', c.c_uint32)])


class Observation(c.Structure):
    _fields_ = [('pose', Pose), ('position', Stamp * 4), ('speed', Stamp * 4),
                ('yaw_stamp', Stamp), ('gyro_stamp', Stamp), ('counts', c.c_double * 4),
                ('rpm', c.c_double * 4), ('yaw_rad', c.c_double), ('gyro_rad_s', c.c_double),
                ('imu_source_epoch', c.c_uint32)]


class Output(c.Structure):
    _fields_ = [('rpm', c.c_double * 4), ('reference', Reference), ('position_error_m', c.c_double),
                ('yaw_error_rad', c.c_double), ('scale', c.c_double)] + [(key, c.c_uint32) for key in (
                    'state', 'fault', 'id', 'output_id', 'zero_id', 'zero_ack_ms', 'stable_elapsed_ms',
                    'blockers', 'done_ms')] + [('micro_rev', c.c_double * 4), ('micro_rpm', c.c_uint32),
                                               ('micro_hold', c.c_uint32)]


def stamp(sample):
    return Stamp(sample.first_ms, sample.last_ms, sample.handled_ms, sample.sequence, sample.epoch)


class Tracker:
    def __init__(self, odometry, config):
        self.odometry = odometry
        self.lib = odometry.lib
        try:
            self.lib.Track_ConfigBytes.restype = c.c_uint32
            if self.lib.Track_ConfigBytes() != c.sizeof(Config):
                raise ValueError('跟踪器配置ABI不匹配，请重建主机模型库')
        except AttributeError as error:
            raise ValueError('主机模型库版本过旧，请重建后再模拟') from error
        self.lib.Track_Bytes.restype = c.c_size_t
        self.lib.Track_Init.argtypes = [c.c_void_p, c.POINTER(Config)]
        self.lib.Track_Start.argtypes = [c.c_void_p, c.POINTER(Goal), c.POINTER(Observation), c.c_uint32]
        self.lib.Track_Step.argtypes = [c.c_void_p, c.POINTER(Observation), c.c_uint32]
        self.lib.Track_Read.argtypes = [c.c_void_p, c.POINTER(Output)]
        self.lib.Track_Read.restype = None
        self.lib.Track_Sent.argtypes = [c.c_void_p, c.c_uint32, c.c_uint32, c.c_uint32]
        self.lib.Track_Token.argtypes = [c.c_void_p]
        self.lib.Track_Token.restype = c.c_uint32
        self.lib.Track_Cancel.argtypes = [c.c_void_p]
        self.lib.Track_Cancel.restype = None
        self.lib.Track_Reference.argtypes = [c.c_void_p, c.c_uint32, c.POINTER(Reference)]
        self.lib.Odom_Heading.argtypes = [c.c_void_p, c.c_uint32, c.POINTER(c.c_double), c.POINTER(Stamp)]
        self.storage = (c.c_double * ((self.lib.Track_Bytes() + 7) // 8))()
        if not self.lib.Track_Init(self.storage, c.byref(config)):
            raise ValueError('跟踪配置无效')

    def observation(self, cache, now_ms, gyro_scale):
        observation = Observation()
        for key, value in self.odometry.snapshot(now_ms).items():
            setattr(observation.pose, key, value)
        yaw, yaw_stamp = c.c_double(), Stamp()
        if not self.lib.Odom_Heading(self.odometry.storage, now_ms, c.byref(yaw), c.byref(yaw_stamp)):
            observation.pose.valid = 0
        observation.yaw_rad = yaw.value
        observation.yaw_stamp = yaw_stamp
        for index, address in enumerate(self.odometry.config.address):
            position = cache.samples.get((2, address))
            speed = cache.samples.get((1, address))
            if position is None or speed is None:
                observation.pose.valid = 0
                continue
            observation.position[index] = stamp(position)
            observation.speed[index] = stamp(speed)
            observation.counts[index] = position.values[0]
            observation.rpm[index] = speed.values[0] * self.odometry.config.sign[index]
        gyro = cache.samples.get((5, 0))
        if gyro is None or cache.fault:
            observation.pose.valid = 0
        else:
            observation.gyro_stamp = stamp(gyro)
            observation.gyro_rad_s = gyro.values[2] * gyro_scale
            observation.imu_source_epoch = gyro.source_epoch
        return observation

    def start(self, goal, observation, now_ms):
        return self.lib.Track_Start(self.storage, c.byref(goal), c.byref(observation), now_ms)

    def step(self, observation, now_ms):
        return self.lib.Track_Step(self.storage, c.byref(observation), now_ms)

    def token(self):
        return self.lib.Track_Token(self.storage)

    def sent(self, goal_id, output_id, now_ms):
        return self.lib.Track_Sent(self.storage, goal_id, output_id, now_ms)

    def cancel(self):
        self.lib.Track_Cancel(self.storage)

    def reference(self, tick_ms):
        point = Reference()
        if not self.lib.Track_Reference(self.storage, tick_ms, c.byref(point)):
            raise ValueError('参考采样失败')
        return {key: getattr(point, key) for key, _ in Reference._fields_}

    def output(self):
        output = Output()
        self.lib.Track_Read(self.storage, c.byref(output))
        result = {key: getattr(output, key) for key, _ in Output._fields_
                  if key not in ('rpm', 'reference', 'micro_rev')}
        result['rpm'] = list(output.rpm)
        result['micro_rev'] = list(output.micro_rev)
        result['reference'] = {key: getattr(output.reference, key) for key, _ in Reference._fields_}
        return result
