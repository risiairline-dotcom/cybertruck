"""粗加工固定观察位的小位移交接；复用唯一MechanismLink和底盘执行器。"""

import hashlib
import json
import math
import struct

from .capabilities import firmware_since
from .execution import config_payload, SPEC_DATA


def contract(record, order, zero_version, epoch, transaction_id):
    from .mech_sequence import digest, groups
    if digest(record) != record.get('sha256'):
        raise ValueError('观察姿态版本SHA不匹配')
    targets = {}
    found = False
    for group in groups(record['rows']):
        for member in group['members']:
            key = member['axis']
            targets[key] = member['target_m'] if key in ('lift','reach') else member['target_rad']
        if group['order'] == order:
            found = True
            break
    keys = ('lift','reach','tray','arm','gripper')
    if not found or set(targets) != set(keys):
        raise ValueError('观察检查点必须明确包含两轴和三舵机，不补默认值')
    values = [round(targets[k]*(1e6 if k in keys[:2] else 180000/math.pi)) for k in keys]
    if any(not 0 <= value < 2**31 for value in values):
        raise ValueError('观察姿态目标无效')
    binding = dict(id=record['id'],revision=record['revision'],sha256=record['sha256'],order=order,
                   targets=values,scope='rough_empty_fixed_heading_micro')
    sha = hashlib.sha256(json.dumps(binding,sort_keys=True,separators=(',',':')).encode()).digest()
    body = struct.pack('<4I5i32sI',transaction_id,epoch,zero_version,1,*values,sha,0)
    return body, binding


def micro_goal(action_id, axis, millimeters):
    if axis not in ('x','y') or type(millimeters) not in (int,float) or not math.isfinite(millimeters):
        raise ValueError('微调轴或位移无效')
    if not 1 <= abs(millimeters) <= 5 or type(action_id) is not int or not 0 < action_id < 2**32:
        raise ValueError('本轮仅开放1～5毫米单轴小位移')
    x, y = (millimeters/1000,0) if axis=='x' else (0,millimeters/1000)
    spec = SPEC_DATA.pack(1,x,y,0, 0,0,0,0, 0,0,
                          .01,.05,.2,.1,.3,1, 0, 0,0,0,0,0)
    return struct.pack('<II',action_id,10000)+spec


# 只等待反馈、不发新请求的阶段期限（秒）；超时按失败停止，不自动重试。
STAGE_LIMIT_S = {
    'KEY_PREPARE': (5., '底盘位姿持续无效，未请求PC0准备'),
    'ARMING': (3., '底盘未在期限内进入执行模式'),
    'MOVING': (12., '小位移未在期限内完成，停止而不重放'),
}


class Arrival:
    def __init__(self):
        self.phase = 'IDLE'
        self.reason = ''
        self.serial = 0
        self.waiting = None
        self.body = None
        self.binding = None
        self.identity = None
        self.action_id = 0
        self.started = 0.
        self.steps = 0
        self.stage_clock = None

    @property
    def active(self):
        return self.phase not in ('IDLE','DONE','FAILED')

    def view(self):
        return dict(phase=self.phase,reason=self.reason,binding=self.binding,
                    action_id=self.action_id,steps=self.steps)

    def start(self, link, record, order, args, now):
        if self.active or not firmware_since(link.firmware, 45) or not link.commissioning:
            raise ValueError('微调要求已支持的固件、现场许可及空闲状态')
        if args.get('empty_path_confirmed') is not True or args.get('actual_pose_confirmed') is not True:
            raise ValueError('需按实物确认空爪、固定观察姿态及小位移空间')
        self.check(link,now)
        if not link.base.get('stopped') or not link.actual.get('zero_valid'):
            raise ValueError('底盘必须停车且人工零点有效')
        h=link.handoff
        if not h or now-link.handoff_at>=.25 or h['phase']:
            raise ValueError('交接状态尚未就绪')
        self.serial=max(self.serial,h['id'])+1
        self.body,self.binding=contract(record,order,link.actual['zero_version'],h['epoch'],self.serial)
        self.identity=(link.boot,link.session,link.actual['zero_version'])
        self.profile=dict(args['profile'],version=10,max_rpm=6)
        config_payload(self.profile)
        self.phase='PREPARE';self.reason='';self.started=now;self.steps=0;self.waiting=None
        self.stage_clock = None

    def check(self, link, now):
        if (link.latched or link.action_locked or not link.bound or
            not 0<=now-link.actual_at<.25 or not 0<=now-link.base_at<.25 or
            not 0<=now-link.group_at<.25 or link.actual.get('fault') or link.base.get('fault')):
            raise ValueError('微调会话或关键反馈失效')

    def queue(self, link, kind, data, now):
        if link.commands or link.operation:
            return False
        self.serial+=1
        request='arrival:'+str(self.serial)
        link.commands.append((kind,data,request))
        self.waiting=(request,now,kind)
        return True

    def move(self, link, axis, mm, now):
        if self.phase!='READY' or self.waiting:
            raise ValueError('尚未进入等待小位移状态')
        self.check(link,now)
        if self.steps>=20:
            raise ValueError('本次20步已用完，请返回机械控制并检查标定')
        self.action_id=max(self.action_id,link.execution.get('action_id',0))+1
        data=micro_goal(self.action_id,axis,mm)
        if not self.queue(link,34,data,now):
            raise ValueError('上一请求未完成')
        self.phase='GOAL';self.steps+=1

    def finish(self):
        if self.phase!='READY' or self.waiting:
            raise ValueError('须小位移完成后才返回机械控制')
        self.phase='RETURN'

    def cancel(self, reason):
        if self.active:
            self.phase='FAILED'
            self.reason=reason
        self.waiting=None

    def advance(self, link, now):
        if not self.active:
            return None
        try:
            if self.stage_clock is None or self.stage_clock[0]!=self.phase:
                self.stage_clock=(self.phase,now)
            limit,message=STAGE_LIMIT_S.get(self.phase,(None,''))
            if limit is not None and now-self.stage_clock[1]>limit:
                raise ValueError(message)
            self.check(link,now)
            if (link.boot,link.session,link.actual.get('zero_version'))!=self.identity:
                raise ValueError('微调启动身份或零点变化')
            if not link.actual.get('zero_valid'):
                raise ValueError('微调零点无效')
            if self.waiting:
                request,started,kind=self.waiting
                result=next((r for r in link.results if r['request_id']==request),None)
                if result:
                    self.waiting=None
                    if result['status']!='ACCEPTED':
                        if (self.phase in ('PREPARE','COMMIT','RETURN') and
                            result.get('result') == 3 and now-self.stage_clock[1]<3):
                            return None  # 明确未接纳的交接，可重试同一合同；运动目标不重放。
                        raise ValueError('交接或微调请求被拒绝，未重放动作')
                    transitions={'PREPARE':'COMMIT','COMMIT':'CONFIG','CONFIG':'KEY_PREPARE',
                                 'ARM':'ARMING','GOAL':'MOVING','RETURN':'DONE'}
                    self.phase=transitions.get(self.phase,self.phase)
                elif kind in (59,60) and link.start_event.get('ticket')==struct.unpack_from('<I',self.body)[0]:
                    self.waiting=None
                    self.phase='WAIT_PC0'
                elif now-started>2:
                    raise ValueError('微调请求未确认，停止而不重放')
                else:
                    return None
            if self.phase in ('PREPARE','COMMIT','RETURN'):
                op={'PREPARE':1,'COMMIT':2,'RETURN':4}[self.phase]
                self.queue(link,71,struct.pack('<I',op)+self.body,now)
            elif self.phase=='CONFIG':
                if link.execution.get('config_version')==10:
                    self.phase='KEY_PREPARE'
                else:
                    self.queue(link,32,config_payload(self.profile),now)
            elif self.phase=='KEY_PREPARE':
                if not link.execution.get('pose_valid') or now-self.started<.4:
                    return None
                self.queue(link,59,struct.pack('<II',struct.unpack_from('<I',self.body)[0],10),now)
                self.phase='KEY_REQUEST'
            elif self.phase=='WAIT_PC0':
                event=link.start_event
                if (event.get('ticket')!=struct.unpack_from('<I',self.body)[0] or
                    event.get('version')!=10 or not 0<=now-event.get('at',-1)<.25):
                    raise ValueError('PC0准备反馈身份不符或过期，未启动')
                if event.get('phase')==5:
                    reasons={1:'启动条件失效',2:'启动会话变化',3:'等待PC0满60秒超时，未启动',
                             4:'收到停止请求',5:'启停区选择变化'}
                    reason=reasons.get(event.get('reason'),'未知原因')
                    raise ValueError(f'PC0准备取消：{reason}；反馈检查位图{event.get("guard_mask",0)}，不自动重启')
                if now-self.started>65:
                    raise ValueError('等待PC0超时，未启动')
                if event.get('fired') and now-event.get('at',-1)<.25:
                    self.phase='ARM'
                else:
                    self.queue(link,60,b'',now)
            elif self.phase=='ARM':
                self.queue(link,33,b'',now)
            elif self.phase=='ARMING' and link.base.get('mode')==3:
                self.phase='READY'
            elif self.phase=='MOVING':
                e=link.execution
                if e.get('action_id')==self.action_id and e.get('action_state')==4 and link.base.get('mode')==3:
                    self.phase='READY'
        except ValueError as error:
            self.phase='FAILED';self.reason=str(error)
            return self.reason
        return None
