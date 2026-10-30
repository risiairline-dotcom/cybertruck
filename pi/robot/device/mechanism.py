"""实车机构协议：唯一会话、显式整数目标、无自动建零或动作重放。"""

import math
import struct
from collections import deque
from .capabilities import firmware_since
from .link import Link
from . import protocol as p
from .execution import STATUS_DATA
from .axis_profile import acceleration, travel_s

FIELDS = ('schema uptime_ms attached fault zero_version zero_valid config_mask base_stopped '
          'axis_state axis_reason axis_id servo_state servo_reason servo_id pose_state pose_reason '
          'pose_id revision step_index step_count channel target_mdeg pwm_mask stop_confirmed '
          'lift_um reach_um stable_samples motion_uncertain').split()

GRIP_HOLD_MDEG = 98000  # 默认夹持目标；不能再用安全最大角替代夹持语义。
AXIS_ZERO_ZONE_UM = 1000
AXIS_ZERO_MAX_RPM = 30  # 近零位使用旧轴低速档，保存表中的原速度不改写。
AXIS_TRAVEL_UM = dict(lift=140000, reach=107000)  # 机械行程，与板端 MECH_*_TRAVEL_UM 一致；无硬限位。


def grip_limit(firmware):
    """新板复用旧工程100度上限；旧固件继续按98度限制，不提前放行。"""
    return 100000 if firmware_since(firmware, 38) else 98000


def payload(action, args, state, token):
    """网页SI单位只在此转换一次；绝不读取模拟零点或模型位置。"""
    version = args.get('zero_version', 0)
    if type(version) is not int or not 0 <= version <= 0xFFFFFFFF:
        raise ValueError('实车零点版本无效')
    prefix = struct.pack('<II', token, version)
    action_id = max(state.get(key, 0) for key in ('axis_id', 'servo_id', 'pose_id', 'group_id','home_id')) + 1
    if action_id >= 0xFFFFFFF0:
        raise ValueError('动作编号耗尽，禁止回绕')
    if action == 'attach':
        return 48, struct.pack('<II', token, 0x4D454348)
    if action in ('home','home_confirm'):
        if args.get('confirmed') is not True or (action=='home' and args.get('safe_zero') is not True):
            raise ValueError('同时回零需确认三路命令0度安全可达；首次运动不能保证限速')
        home_id=action_id if action=='home' else args.get('home_id')
        if type(home_id) is not int or not 0<home_id<0xfffffff0:
            raise ValueError('舵机回零动作编号无效')
        return (66 if action=='home' else 67),prefix+struct.pack('<IB',home_id,1)
    if action == 'mode':
        if args.get('confirmed') is not True or type(args.get('manual_debug')) is not bool:
            raise ValueError('切换手动调试模式需要明确现场确认')
        return 62, prefix + bytes([args['manual_debug'], 1])
    if action == 'enable':
        if args.get('confirmed') is not True or args.get('axis') not in ('lift', 'reach'):
            raise ValueError('轴使能需要独立现场确认')
        return 49, prefix + bytes([int(args['axis'] == 'reach')])
    if action == 'zero':
        return 50, prefix + bytes([args.get('confirmed') is True])
    if action == 'anchor':
        targets = args.get('targets', {})
        angles = [integer(targets.get(key), 1000 * 180 / math.pi) for key in ('tray', 'arm', 'gripper')]
        if not 0 <= angles[2] <= grip_limit(args.get('_firmware')):
            raise ValueError('夹爪参考角超出当前固件安全上限')
        return 51, prefix + struct.pack('<3iB', *angles, args.get('confirmed') is True)
    if action == 'invalidate':
        return 57, prefix
    if action == 'move':
        actuator = args.get('axis')
        if actuator in ('lift', 'reach'):
            rpm = args.get('rpm', 5)
            accel = acceleration(args.get('accel',1))
            if accel!=1 and not firmware_since(args.get('_firmware'), 42):
                raise ValueError('可调轴加速度需要GC-BASE-0.3.42；旧固件固定acc=1')
            tuned = args.get('_manual_tuning') is True
            limit = 3000 if tuned else 30
            if type(rpm) is not int or not 1 <= rpm <= limit:
                raise ValueError(f'单轴转速必须为1～{limit}RPM整数')
            target = integer(args.get('target_m'), 1e6)
            if not 0 <= target <= AXIS_TRAVEL_UM[actuator]:
                raise ValueError('升降0～140mm、伸缩0～107mm；最高/最近端为0，下降和伸出为正')
            if tuned and target <= AXIS_ZERO_ZONE_UM and not firmware_since(args.get('_firmware'), 39):
                rpm = min(rpm, AXIS_ZERO_MAX_RPM)
            timeout = 15000
            if tuned:
                # 沿用板端4.098/12.4mm每320脉冲；留出加减速和反馈确认时间。
                current = state.get('lift_um' if actuator == 'lift' else 'reach_um', 0)
                timeout = max(3000, math.ceil(travel_s(actuator,target-current,rpm,accel)*1500+3000))
                if timeout > 300000:
                    raise ValueError('本次位移按所填速度预计超过300秒，请分段调试')
            return 52, prefix + struct.pack('<IiIHB', action_id,
                target, timeout, rpm, int(actuator == 'reach')) + (bytes([accel]) if firmware_since(args.get('_firmware'), 42) else b'')
        channel = dict(tray=1, arm=2, gripper=3).get(actuator)
        if args.get('accel',1)!=1:
            raise ValueError('轴加速度档位只适用于升降和伸缩')
        if not channel:
            raise ValueError('执行器无效')
        target = integer(args.get('target_rad'), 180000 / math.pi)
        arm_max = 359000 if firmware_since(args.get('_firmware'), 32) else 270000
        if (args.get('_manual_tuning') is True or channel == 3) and not 0 <= target <= {1:270000,2:arm_max,3:grip_limit(args.get('_firmware'))}[channel]:
            raise ValueError('舵机命令角超出允许范围，0度不加偏置')
        speed = args.get('speed_rad_s')
        timeout, tail = 30000, b''
        if speed is not None:
            rate = integer(speed, 180000 / math.pi)
            if not 1000 <= rate <= 360000:
                raise ValueError('舵机指令速度必须为1～360度/秒')
            span = 359000 if channel == 2 and firmware_since(args.get('_firmware'), 32) else 270000
            # .35/.36三路平滑，.37仅PE11；PE9/PE13回到匀速期限。
            smooth = (args.get('_firmware') in ('GC-BASE-0.3.35','GC-BASE-0.3.36') or
                      firmware_since(args.get('_firmware'), 37) and channel == 2)
            ramp_s = rate/180000 + .25 if smooth else 0
            timeout = math.ceil(span/rate*1000 + ramp_s*1000 + 3000)
            if timeout > (420000 if firmware_since(args.get('_firmware'), 32) else 300000):
                raise ValueError('本次舵机指令超过执行期限，请提高指令速度或分段')
            tail = struct.pack('<I', rate)
        return 53, prefix + struct.pack('<IiIBB', action_id,
            target, timeout, channel,
            args.get('release_confirmed') is True) + tail
    if action == 'group':
        members = args.get('members')
        if not isinstance(members, list) or not 1 <= len(members) <= 5:
            raise ValueError('联调组需要1～5个不同执行器')
        names = ('lift', 'reach', 'tray', 'arm', 'gripper')
        mask, timeout, targets, speeds = 0, 1000, [0]*5, [0]*5
        accelerations=[1,1]
        for member in members:
            if not isinstance(member, dict) or member.get('axis') not in names:
                raise ValueError('联调执行器无效')
            i = names.index(member['axis'])
            if mask & (1 << i):
                raise ValueError('同一同时组不能重复设置同一执行器，请拆为不同顺序')
            mask |= 1 << i
            params = dict(member, _manual_tuning=True, _firmware=args.get('_firmware','GC-BASE-0.3.32'), release_confirmed=True)
            if i >= 2:
                params.setdefault('speed_rad_s', math.pi/4)
            _, wire = payload('move', params, state, token)
            targets[i], deadline = struct.unpack_from('<iI', wire, 12)
            timeout = max(timeout, deadline)
            speeds[i] = struct.unpack_from('<H' if i < 2 else '<I', wire, 20 if i < 2 else 22)[0]
            if i<2:accelerations[i]=acceleration(member.get('accel',1))
        return 63, prefix + struct.pack('<III5i5I', action_id, timeout, mask, *targets, *speeds) + (
            bytes(accelerations) if firmware_since(args.get('_firmware'), 42) else b'')
    if action == 'confirm':
        if args.get('confirmed') is not True or state.get('servo_state') not in (5, 9):
            raise ValueError('只能人工确认当前已完成命令；不能用计时或模型确认')
        return 54, prefix + struct.pack('<IIIiBI', state['servo_id'], token,
            state['uptime_ms'], state['target_mdeg'], state['channel'], 1)
    if action == 'next':
        if args.get('action_id') != state.get('pose_id') or args.get('revision') != state.get('revision') or args.get('step_index') != state.get('step_index'):
            raise ValueError('继续必须绑定当前实车姿态、版本和步骤')
        return 56, prefix + struct.pack('<IIB', args['action_id'], args['revision'], args['step_index'])
    if action == 'pose':
        if args.get('physical_taught') is not True or not args.get('revision'):
            raise ValueError('只允许显式确认的实车示教版本，禁止直接执行模型示教')
        return 55, prefix + pose_body(action_id,args['revision'],args.get('targets',{}),
                                      args.get('single_step',True),args.get('release_confirmed',False),
                                      gripper_max=grip_limit(args.get('_firmware')))
    raise ValueError('不支持的实车动作；本版不开放连续点动')


def integer(value, scale):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError('缺少有限数值目标')
    result = round(value * scale)
    if not -2**31 <= result < 2**31:
        raise ValueError('目标超出整数接口范围')
    return result


def pose_body(action_id, revision, targets, single_step=True, release_confirmed=False, *, gripper_max=98000):
    """正式55载荷的纯编码部分；无会话/零点权限，也不发送设备命令。"""
    if any(type(value) is not int or not 1 <= value < 0xFFFFFFF0 for value in (action_id,revision)):
        raise ValueError('姿态动作/版本编号无效')
    values = [integer(targets.get(key),1e6 if key in ('lift','reach') else 180000/math.pi)
              for key in ('lift','reach','arm','tray','gripper')]
    if not 0 <= values[4] <= gripper_max:
        raise ValueError('姿态夹爪目标超出当前固件安全上限')
    return struct.pack('<III5iBB',action_id,revision,120000,*values,single_step is True,release_confirmed is True)


class MechanismLink(Link):
    def __init__(self, commissioning=False):
        super().__init__(capabilities=5)
        self.commissioning = commissioning
        self.commands = deque(maxlen=1)
        self.results = deque(maxlen=40)
        self.actual = {}
        self.actual_at = self.base_at = -1.
        self.base = {}
        self.last_poll = -1.
        self.poll_kind = 58
        self.latched = False
        self.stop_wire = b''
        self.operation = None
        self.token = 0
        self.action_locked = False
        self.stop_queued = False
        self.stop_acknowledged = False
        self.stop_reason = ''
        self.manual_debug = False
        self.group = {}
        self.group_at = -1.
        self.poll_index = 0
        self.inspections = {}
        self.recovery_wait = False
        self.home = {}
        self.home_at = -1.
        self.park_failure = None
        self.last_park_poll = -1.
        self.failure = None
        self.last_failure_poll = -1.
        self.handoff = {}
        self.handoff_at = -1.
        self.start_event = {}
        self.execution = {}

    def reset(self, reason):
        self.recovery_wait = False
        # 失联后只请求停止，不自动重新握手/重发动作。
        self.manual_debug = False
        self.latched = True
        self.action_locked = True
        self.stop_queued = False
        self.stop_acknowledged = False
        self.stop_reason = reason
        self.commands.clear()
        self.operation = None
        self.message = reason
        self.stop_wire = p.Packet(p.STOP, self.boot or 0, self.session, self.request + 1, 250, b'').encode()
        self.bound = False
        self.pending = None

    def emergency(self, reason):
        """停止动作但保留同会话只读轮询；动作锁不自动解除。"""
        recovering = self.recovery_wait
        self.recovery_wait = False
        if self.latched or not self.bound or not self.boot:
            self.reset(reason)
            return
        if self.action_locked and not recovering and not (self.commands or self.operation or
                (self.pending and self.pending[0].kind == 61)):
            return
        self.action_locked = True
        self.manual_debug = False
        self.stop_reason = reason
        self.stop_acknowledged = False
        self.commands.clear()
        self.operation = None
        self.pending = None
        self.stop_queued = True

    def step(self, now):
        if self.recovery_wait:
            return b''
        if self.stop_wire:
            data, self.stop_wire = self.stop_wire, b''
            return data
        if self.latched:
            return b''
        if self.stop_queued:
            self.stop_queued = False
            self.request = (self.request + 1) & 0xFFFFFFFF
            packet = p.Packet(p.STOP, self.boot, self.session, self.request, 250, b'')
            self.pending = (packet, now)
            return packet.encode()
        return super().step(now)

    def extra_request(self, now):
        if firmware_since(self.firmware, 41):
            # 提前刷新关键状态，不能等250ms即将超时才被命令/诊断挤到后面。
            polls=[(35,self.base_at,.08),(58,self.actual_at,.08),(64,self.group_at,.10)]
            if self.home.get('state') in (1,2,3):polls.append((68,self.home_at,.10))
            due=[(now-at-limit,kind) for kind,at,limit in polls if now-at>=limit]
            if due:
                return max(due)[1],b''
        if self.commands:
            kind, data, request_id = self.commands.popleft()
            self.operation = request_id
            return kind, data
        if firmware_since(self.firmware, 45) and now-self.handoff_at >= .15:
            return 72, b''  # 交接显示不挤占已排队动作；关键35/58/64仍优先。
        if firmware_since(self.firmware, 39) and now-self.last_failure_poll >= 1:
            self.last_failure_poll = now
            return 70, b''
        if firmware_since(self.firmware, 35) and now-self.last_park_poll >= 1:
            self.last_park_poll = now
            return 69, b''  # 仅取冻结记录，不重启/清故障；关键状态仍按原节拍轮询。
        if now - self.last_poll >= (.03 if firmware_since(self.firmware, 34) else .04):
            self.last_poll = now
            if firmware_since(self.firmware, 32):
                kinds=(35,58,64,68) if firmware_since(self.firmware, 34) else (35,58,64)
                self.poll_kind = kinds[self.poll_index % len(kinds)]
                self.poll_index += 1
            else:
                self.poll_kind = 35 if self.poll_kind == 58 else 58
            return self.poll_kind, b''
        return None

    def custom_reply(self, packet, request, now, sent=None):
        if request.kind in (32,33,34,59,60,71,72):
            if packet.boot != self.boot or not packet.payload:
                self.reset('微调协议身份或应答无效')
                return True
            if packet.payload[0] != p.OK:
                self.results.appendleft(dict(request_id=self.operation,result=packet.payload[0],status='REJECTED'))
                self.operation = None
                return True
            data = packet.payload[1:]
            if request.kind == 72:
                if len(data)!=24 or struct.unpack_from('<I',data)[0]!=1:
                    self.reset('执行交接状态无效')
                else:
                    self.handoff=dict(zip(('schema','phase','epoch','id','zero_version','tick_ms'),struct.unpack('<6I',data)))
                    self.handoff_at=now
            elif request.kind in (59,60):
                if (len(data)!=108 or struct.unpack_from('<I',data)[0]!=3 or
                        struct.unpack_from('<I',data,4)[0]>5 or
                        struct.unpack_from('<I',data,16)[0]!=int(struct.unpack_from('<I',data,4)[0]==4)):
                    self.reset('PC0微调准备状态无效')
                else:
                    previous=self.start_event
                    self.start_event=dict(zip(('schema','phase','ticket','version','fired','beep'),struct.unpack_from('<6I',data)))
                    names=('reason','aborted_ms','guard_mask','guard_ms','pose_ms','yaw_ms','gyro_ms',
                           'position1_ms','position2_ms','position3_ms','position4_ms',
                           'speed1_ms','speed2_ms','speed3_ms','speed4_ms')
                    self.start_event.update(zip(names,struct.unpack_from('<15I',data,24)))
                    self.start_event.update(zip(('select_level','select_zone','select_valid','select_epoch',
                                                'latched_zone','latched_epoch'),struct.unpack_from('<6I',data,84)))
                    # 以实际59发送时刻计算保守倒计时，60查询不能续期。
                    if request.kind==59:
                        deadline=(now if sent is None else sent)+60.
                        if previous.get('ticket')==self.start_event['ticket'] and previous.get('version')==self.start_event['version']:
                            deadline=min(deadline,previous.get('deadline_s',deadline))
                        self.start_event['deadline_s']=deadline
                    elif previous.get('ticket')==self.start_event['ticket'] and previous.get('version')==self.start_event['version']:
                        if 'deadline_s' in previous:self.start_event['deadline_s']=previous['deadline_s']
                    self.start_event['at']=now
            else:
                valid = len(data)==4 if request.kind==71 else not data
                if not valid:
                    self.reset('微调命令应答长度无效')
                else:
                    result=struct.unpack('<I',data)[0] if data else 0
                    self.results.appendleft(dict(request_id=self.operation,result=result,status='ACCEPTED' if not result else 'REJECTED'))
            self.operation=None
            return True
        if request.kind in (38,65):
            if packet.boot != self.boot or not packet.payload or packet.payload[0] != p.OK:
                self.reset('恢复诊断或重启请求被拒绝；不重发')
            else:
                data = packet.payload[1:]
                if request.kind == 65 and len(data) != 4:
                    self.reset('重启应答格式无效')
                else:
                    self.inspections[self.operation] = dict(kind=request.kind,data=data.hex(),
                        result=struct.unpack('<I',data)[0] if request.kind==65 else None)
            self.operation = None
            return True
        if request.kind == p.STOP:
            if packet.boot != self.boot or packet.payload != bytes([p.OK]):
                self.reset('停止应答无效，动作保持锁定')
            else:
                self.stop_acknowledged = True
            return True
        if request.kind not in range(48, 59) and request.kind not in (35, 61, 62, 63, 64,66,67,68,69,70):
            return False
        if packet.boot != self.boot or not packet.payload:
            self.reset('实车启动身份/回包无效')
            return True
        if packet.payload[0] == p.BUSY and request.kind not in (58,35,64,66,67,68,69,70):
            # BUSY代表未进入命令；只重试同一不可变载荷，板端操作编号去重。
            self.commands.appendleft((request.kind, request.payload, self.operation))
            self.operation = None
            return True
        if packet.payload[0] != p.OK:
            self.reset('实车协议拒绝；动作不重放')
            return True
        data = packet.payload[1:]
        if request.kind == 70:
            if (not firmware_since(self.firmware, 39) or len(data)!=124 or
                    struct.unpack_from('<I',data)[0]!=1 or
                    struct.unpack_from('<I',data,4)[0] not in (0,1) or
                    struct.unpack_from('<I',data,16)[0]>2 or struct.unpack_from('<I',data,24)[0]&~3):
                self.reset('机构首次故障记录格式无效')
            else:
                names=('schema','captured','tick_ms','reason','axis','zero_version','flags')
                self.failure=dict(zip(names,struct.unpack_from('<7I',data)))
                for name,offset in (('actual_raw',28),('target_raw',44),('zero_actual_raw',60),('zero_target_raw',76),('position_um',92)):
                    self.failure[name]=list(struct.unpack_from('<2q',data,offset))
                self.failure['rpm']=list(struct.unpack_from('<2i',data,108))
                self.failure['age_ms']=list(struct.unpack_from('<2I',data,116))
        elif request.kind == 69:
            length,schema = (124,2) if firmware_since(self.firmware, 36) else (120,1)
            if (len(data) != length or struct.unpack_from('<I',data)[0] != schema or
                    struct.unpack_from('<I',data,4)[0] not in (0,1) or
                    struct.unpack_from('<I',data,12)[0] & ~63 or
                    struct.unpack_from('<I',data,16)[0] > 4 or
                    struct.unpack_from('<I',data,36)[0] > 3 or
                    (schema == 2 and struct.unpack_from('<I',data,120)[0] & ~3)):
                self.reset('底盘停车条件记录格式无效')
            else:
                names = ('schema','captured','tick_ms','mask','wheel','stopped','base_fault','can_errors','action_id','channel')
                self.park_failure = dict(zip(names,struct.unpack_from('<10I',data)))
                self.park_failure['wheels'] = [dict(zip(
                    ('rpm','status','speed_age_ms','position_age_ms','status_age_ms'),
                    struct.unpack_from('<i4I',data,40+i*20))) for i in range(4)]
                if schema == 2:
                    flags, = struct.unpack_from('<I',data,120)
                    self.park_failure['manual_speed_ignored'] = bool(flags & 1)
                    self.park_failure['wheel_speed_observed'] = bool(flags & 2)
        elif request.kind == 68:
            if len(data)!=32 or struct.unpack_from('<I',data)[0]!=1:
                self.reset('舵机初始化状态格式无效')
            else:
                self.home=dict(zip(('schema','uptime_ms','id','state','reason','zero_version','pwm_mask','issued_ms'),struct.unpack('<8I',data)))
                self.home_at=now
                self.actual['home_id']=self.home['id']
        elif request.kind == 64:
            if len(data) != 40 or struct.unpack_from('<I',data)[0] != 1:
                self.reset('联调组反馈格式无效')
            else:
                self.group = dict(zip(('schema','uptime_ms','id','state','reason','mask','axis_mask','tray_mdeg','arm_mdeg','gripper_mdeg'),struct.unpack('<10I',data)))
                self.group_at = now
                self.actual['group_id'] = self.group['id']
        elif request.kind == 58:
            if len(data) != 112:
                self.reset('机构状态长度无效')
            else:
                values = struct.unpack('<28I', data)
                if values[0] != 1:
                    self.reset('机构状态版本无效')
                else:
                    self.actual = dict(zip(FIELDS, values))
                    self.actual['group_id'] = self.group.get('id',0)
                    self.actual['home_id'] = self.home.get('id',0)
                    for key in ('lift_um', 'reach_um', 'target_mdeg'):
                        if self.actual[key] & 0x80000000:
                            self.actual[key] -= 2**32
                    self.actual_at = now
                    if self.actual.get('fault') or not self.actual.get('zero_valid'):
                        self.manual_debug = False
        elif request.kind == 35:
            if len(data) != STATUS_DATA.size:
                self.reset('底盘状态长度无效')
            else:
                values = STATUS_DATA.unpack(data)
                self.base = dict(mode=values[0], fault=values[1], stopped=bool(values[5]))
                self.execution = dict(action_id=values[2],action_state=values[3],fault=values[4],
                                      config_version=values[7],pose_valid=bool(values[10]),
                                      x_m=values[11],y_m=values[12],yaw_rad=values[13])
                self.base_at = now
        else:
            if len(data) != 4:
                self.reset('机构动作应答长度无效')
            else:
                result, = struct.unpack('<I', data)
                if request.kind == 62 and not result:
                    self.manual_debug = bool(request.payload[8])
                if request.kind == 61 and not result:
                    self.action_locked = False
                    self.stop_acknowledged = False
                    self.stop_reason = ''
                    self.manual_debug = False
                    self.actual_at = self.base_at = -1. # 必须读取准备后的新状态才能授权。
                self.results.appendleft(dict(request_id=self.operation, result=result,
                                             status='ACCEPTED' if not result else 'REJECTED'))
            self.operation = None
        return True

    def prepare(self, request_id, args, now):
        """同会话正常停止恢复；只恢复准备能力，不取得控制权或输出。"""
        m = self.actual
        if (not firmware_since(self.firmware, 26) or not self.commissioning or
                self.latched or not self.bound or not self.action_locked or
                not self.stop_acknowledged or now - self.actual_at >= .25 or
                now - self.base_at >= .25 or not self.base.get('stopped') or
                self.base.get('fault') or m.get('fault') != 6 or not m.get('attached') or
                m.get('config_mask') != 3 or not m.get('base_stopped') or
                m.get('zero_valid') or m.get('pwm_mask') or
                args.get('confirmed') is not True or
                args.get('boot_id') != str(self.boot) or args.get('session_id') != str(self.session) or
                args.get('zero_version') != m.get('zero_version')):
            raise ValueError('仅新固件、同会话正常停止且反馈新鲜/PWM关闭时可重新准备；CAN故障不能清除')
        if self.commands or self.operation or self.stop_queued:
            raise ValueError('上一请求尚未完成，禁止重复准备')
        self.token += 1
        self.commands.append((61, struct.pack('<IIB', self.token, m['zero_version'], 1), request_id))
        return dict(status='QUEUED', request_id=request_id, domain='physical', action_sent=False)

    def submit(self, request_id, action, args, now):
        args = dict(args)
        tuned = firmware_since(self.firmware, 31) and self.manual_debug
        args['_manual_tuning'] = tuned  # 不信任调用方提供的模式标志。
        args['_firmware'] = self.firmware
        if self.home.get('state') in (1,2,3) and action!='home_confirm':
            raise ValueError('舵机回零尚未确认；请核对三路实际0度或停止')
        if action in ('home','home_confirm'):
            if (not firmware_since(self.firmware, 34) or not self.manual_debug or
                    now-self.home_at>=.25 or self.home.get('zero_version')!=self.actual.get('zero_version')):
                raise ValueError('回零初始化要求0.3.34手动模式及有效零点、新鲜初始化状态')
            if action=='home_confirm' and (self.home.get('state')!=3 or args.get('home_id')!=self.home.get('id')):
                raise ValueError('只能确认本次已输出完成的回零动作')
        if self.group.get('state') == 1:
            raise ValueError('联调组运行中，禁止插入单项操作')
        if action == 'group' and (not firmware_since(self.firmware, 32) or not tuned or
                self.group.get('state') == 1 or now-self.group_at >= .25):
            raise ValueError('联调组需要0.3.32、手动模式及新鲜空闲组反馈')
        if action == 'move':
            if firmware_since(self.firmware, 31) and not self.manual_debug:
                raise ValueError('单项调试请先进入手动模式；自动姿态使用独立入口')
            if args.get('speed_rad_s') is not None and not tuned:
                raise ValueError('可调舵机速度需要0.3.31和本次手动模式')
            if tuned and args.get('axis') in ('tray','arm','gripper'):
                args.setdefault('speed_rad_s', math.pi/4)
            if tuned and args.get('axis') == 'gripper':
                args['release_confirmed'] = True  # 明确执行夹爪目标即代表本次开合意图。
        if action == 'mode' and (not firmware_since(self.firmware, 27) or not self.commissioning or not self.actual.get('zero_valid')):
            raise ValueError('手动调试模式要求新固件、显式机械许可及有效人工零点')
        if action == 'pose' and self.manual_debug:
            raise ValueError('手动调试模式不执行自动姿态；请先返回互锁模式')
        if self.action_locked:
            raise ValueError('动作已停止并锁定；只读反馈保留，需人工核对后重新准备')
        # 新板端目前只验收诊断附着，不能借版本兼容开放其它机械命令。
        commissioned = self.commissioning and firmware_since(self.firmware, 24)
        readonly_attach = not commissioned and firmware_since(self.firmware, 23) and action == 'attach'
        if (self.latched or not self.bound or (not readonly_attach and not commissioned and self.firmware not in ('GC-BASE-0.3.14', 'GC-BASE-0.3.15', 'GC-BASE-0.3.17', 'GC-BASE-0.3.18', 'GC-BASE-0.3.19'))
                or now - self.actual_at >= .25 or now - self.base_at >= .25):
            raise ValueError('实车固件/会话/反馈未就绪')
        if readonly_attach and (self.base.get('mode') != 0 or self.base.get('fault') or
                not self.base.get('stopped') or self.actual.get('schema') != 1 or
                any(self.actual.get(key) for key in ('attached', 'fault', 'zero_valid', 'pwm_mask',
                    'axis_state', 'servo_state', 'pose_state', 'motion_uncertain'))):
            raise ValueError('只读附着要求无故障、底盘已停车、机构未接入且无动作/PWM')
        if self.commands or self.operation or (self.pending and self.pending[0].kind in (*range(48, 58), 62, 63,66,67)):
            raise ValueError('上一条实车命令尚未完成应答')
        if action != 'attach' and (self.base.get('fault') or self.actual.get('fault') or not self.actual.get('base_stopped')):
            raise ValueError('实车故障或底盘未停稳；禁止动作')
        if action != 'attach' and args.get('zero_version', 0) != self.actual.get('zero_version'):
            raise ValueError('实车零点版本变化')
        self.token += 1
        kind, data = payload(action, args, self.actual, self.token)
        self.commands.append((kind, data, request_id))
        return dict(status='QUEUED', request_id=request_id, domain='physical', action_sent=False)

    def view(self, now):
        ages = {name: None if stamp < 0 or now < stamp else math.ceil((now-stamp)*1000)
                for name, stamp in (('actual', self.actual_at), ('base', self.base_at),
                                    ('group', self.group_at), ('home', self.home_at))}
        state_age = None if ages['actual'] is None or ages['base'] is None else max(ages['actual'], ages['base'])
        fresh = self.bound and not self.latched and state_age is not None and state_age < 250
        return dict(available=True, domain='physical', connected=fresh, real_connected=fresh,
                    axis_zero_policy=None if firmware_since(self.firmware, 39) else
                        dict(target_max_um=AXIS_ZERO_ZONE_UM,max_rpm=AXIS_ZERO_MAX_RPM),
                    manual_policy=dict(endpoint_tolerance_um=1000 if firmware_since(self.firmware, 43) else 500,tolerance_um=500,stable_ms=100,stable_samples=2,requested_speed=True)
                        if firmware_since(self.firmware, 39) else None,
                    firmware=self.firmware, boot_id=str(self.boot), session_id=str(self.session),
                    handoff=self.handoff if now-self.handoff_at<.25 else None,
                    execution=self.execution if now-self.base_at<.25 else None,
                    physical_start=dict({k:v for k,v in self.start_event.items() if k not in ('at','deadline_s')},
                        age_ms=max(0,round((now-self.start_event['at'])*1000)),
                        remaining_ms=max(0,int((self.start_event['deadline_s']-now)*1000))
                            if self.start_event.get('phase') in (1,2,3) and 'deadline_s' in self.start_event else None)
                        if self.start_event else None,
                    latched=self.latched, action_locked=self.action_locked, manual_debug=self.manual_debug,
                    stop_acknowledged=self.stop_acknowledged,
                    reason=self.stop_reason if self.action_locked else self.message,
                    mechanism=self.actual if fresh else None,
                    base=self.base if fresh else None, results=list(self.results),
                    group=self.group if fresh and now-self.group_at<.25 else None,
                    home=self.home if fresh and now-self.home_at<.25 else None,
                    park_failure=self.park_failure if fresh else None,
                    mechanism_failure=self.failure if fresh else None,
                    feedback_age_ms=ages, state_age_ms=state_age)
