"""2号圆环对准画面中心；复用旧稳定观测和矩阵预览，执行仍交给唯一Arrival。"""
import copy
import math
from collections import deque

from .legacy.alignment_vision.core import AlignmentObserver, context, fingerprint
from .arrival import validate_calibration, preview


class FrameNotFresh(ValueError):
    pass


def camera_signature(value):
    size=value.get('image_size')
    device=value.get('device_id')
    profile=value.get('profile_sha256')
    if (not isinstance(size,list) or len(size)!=2 or
            any(type(v) is not int or not 1<=v<=4096 for v in size) or
            not isinstance(device,str) or not device or len(device)>512 or
            not isinstance(profile,str) or len(profile)!=64 or
            any(c not in '0123456789abcdef' for c in profile)):
        raise ValueError('相机身份、分辨率或视觉配置未就绪，需要新版相机元数据')
    return dict(device_id=device,image_size=size,profile_sha256=profile)


def load_calibration(value):
    if (not isinstance(value,dict) or value.get('goal')!='camera_image_center' or
            value.get('sha256')!=fingerprint({k:v for k,v in value.items() if k!='sha256'})):
        raise ValueError('自动对中标定文件缺少校验或目标不是画面中心')
    camera=value.get('camera_signature')
    if not isinstance(camera,dict):raise ValueError('标定文件缺少相机身份')
    camera_signature(dict(camera,image_size=camera.get('image_size')))
    data=validate_calibration(value,camera)
    binding=data.get('observation_binding_sha256')
    if not isinstance(binding,str) or len(binding)!=64 or any(c not in '0123456789abcdef' for c in binding):
        raise ValueError('标定缺少观察姿态摘要')
    if data.get('physical_directions_checked') is not True:
        raise ValueError('本车小位移方向尚未实物验收')
    return data


class Centering:
    def __init__(self):
        self.mode='manual'
        self.status='IDLE'
        self.reason=''
        self.calibration=None
        self.calibration_error='尚未导入本车已验收的方向/比例标定'
        self.observer=None
        self.camera=None
        self.stream=None
        self.binding=None
        self.key=None
        self.floor=0.
        self.barrier=0
        self.ready_at=0.
        self.started=0.
        self.last_sequence=-1
        self.last_observation=None
        self.pending_measure=None
        self.records=deque(maxlen=100)
        self.trace=deque(maxlen=24)
        self.trace_serial=0
        self.record_serial=0
        self.auto_steps=0
        self.total_mm=0.
        self.previous_radius=None
        self.not_improving=0
        self.completed=None
        self.reference_yaw=None

    def set_calibration(self,value):
        candidate=load_calibration(value)
        self.calibration=candidate
        self.calibration_error=''

    def begin(self, arrival, mode, now, yaw_correction_deg=0.):
        """yaw_correction_deg：粗停航向相对站位朝向的修正量（度，逆时针为正），由任务层或人工给出。"""
        if mode not in ('manual','auto'):raise ValueError('微调模式无效')
        if (type(yaw_correction_deg) not in (int,float) or not math.isfinite(yaw_correction_deg) or
                abs(yaw_correction_deg)>3):
            raise ValueError('航向修正须在±3度以内')
        binding=arrival.body[36:68].hex()
        if mode=='auto':
            if self.calibration is None:raise ValueError(self.calibration_error)
            if self.calibration['observation_binding_sha256']!=binding:
                raise ValueError('当前观察姿态与自动对中标定不一致')
        self.mode=mode;self.binding=binding;self.status='PREPARING';self.reason=''
        self.started=now;self.key=None;self.observer=None;self.camera=None;self.stream=None
        self.last_observation=None;self.pending_measure=None;self.last_sequence=-1
        self.auto_steps=0;self.total_mm=0.;self.previous_radius=None;self.not_improving=0
        self.completed=None;self.trace.clear()
        self.reference_yaw=None
        self.yaw_correction=math.radians(yaw_correction_deg)

    def cancel(self,reason):
        self.status='CANCELLED';self.reason=reason;self.mode='manual';self.pending_measure=None
        self.last_observation=None

    def pause(self,reason):
        self.status='PAUSED';self.reason=reason

    def view(self):
        return dict(status=self.status,mode=self.mode,reason=self.reason,
            goal='camera_image_center',target_position_id=2,observation=self.last_observation,
            automatic_enabled=self.mode=='auto' and self.status not in ('PAUSED','CANCELLED','CENTERED'),
            calibration_available=self.calibration is not None,calibration_error=self.calibration_error,
            calibration_id=self.calibration.get('calibration_id') if self.calibration else None,
            steps=self.auto_steps,total_mm=round(self.total_mm,3),record_count=len(self.records),records=list(self.records)[-12:],
            trace=list(self.trace),completed=self.completed)

    def note_move(self,link,axis,mm,now):
        o=self.last_observation
        if (o and o.get('stable') and 0<=now-o['received_s']<.25 and
                0<=now-o['latest_capture_s']<.25):
            self.pending_measure=dict(axis=axis,command_mm=mm,before=copy.deepcopy(o),
                before_odom=[link.execution.get('x_m'),link.execution.get('y_m')],
                action_id=link.execution.get('action_id',0))
        else:self.pending_measure=None

    def _observe(self,value,received,now):
        if (value.get('domain')!='camera_live' or value.get('camera')!='material' or
                value.get('recognition_mode')!='ring'):
            raise ValueError('相机帧来源不符')
        if (type(value.get('captured_s')) not in (int,float) or
                not 0<=now-value['captured_s']<.25 or not 0<=now-received<.25):
            raise FrameNotFresh('等待新鲜圆环图像')
        signature=camera_signature(value)
        stream=(value.get('stream_id'),value.get('capture_epoch'))
        if (not isinstance(stream[0],str) or not stream[0] or type(stream[1]) is not int or stream[1]<0):
            raise ValueError('相机采集身份不完整')
        if self.camera is not None and (self.camera!=signature or self.stream!=stream):
            raise ValueError('相机身份/配置或采集会话已改变，需要重新准备')
        if self.mode=='auto' and signature!=self.calibration['camera_signature']:
            raise ValueError('当前相机与已验收标定不一致')
        if self.camera is None:
            self.camera=signature;self.stream=stream
            self.observer=AlignmentObserver(signature,signature['device_id'])
            self.observer.reference=dict(context=context('rough','rough_2_image_center',2),
                centers_px={'2':[signature['image_size'][0]/2,signature['image_size'][1]/2]},id='image_center')
        if value['captured_s']<=self.floor:return None
        sequence=value.get('frame_id')
        if type(sequence) is not int or sequence<=self.last_sequence:return None
        self.last_sequence=sequence
        o=self.observer.process(value.get('items',[]),sequence,value['captured_s'],now,
            context('rough','rough_2_image_center',2),signature['image_size'])
        return dict(stable=o['offset_valid'],center_px=o['current_centers_px'].get('2'),
            image_center_px=self.observer.reference['centers_px']['2'],delta_px=o['delta_image_px'],
            reasons=o['reasons'],samples=o['track']['samples'],span_ms=o['track']['span_ms'],
            jitter_px=o['track']['jitter_px'],latest_capture_s=value['captured_s'],received_s=now,
            window_max_error_px=max((math.hypot(p[2][0]-signature['image_size'][0]/2,
                p[2][1]-signature['image_size'][1]/2) for _,p in self.observer.history),default=None),
            oldest_capture_s=self.observer.history[0][0] if self.observer.history else value['captured_s'],
            sequence=sequence,barrier=self.barrier,camera_signature=signature,stream_id=stream[0],
            capture_epoch=stream[1],observation_binding_sha256=self.binding)

    def _record(self,link,o):
        pending=self.pending_measure
        if not pending:return None
        self.pending_measure=None
        before=pending['before']
        self.record_serial+=1
        after=[link.execution.get('x_m'),link.execution.get('y_m')]
        odom_delta=None
        if all(type(x) in (int,float) and math.isfinite(x) for x in pending['before_odom']+after):
            odom_delta=[round((after[i]-pending['before_odom'][i])*1000,6) for i in (0,1)]
        record=dict(id=self.record_serial,axis=pending['axis'],command_mm=pending['command_mm'],
            odom_delta_mm=odom_delta,delta_px=[round(o['center_px'][i]-before['center_px'][i],6) for i in (0,1)],
            before=before,after=copy.deepcopy(o),observation_binding_sha256=self.binding,
            boot_id=str(link.boot),session_id=str(link.session),zero_version=link.actual['zero_version'],
            action_id=link.execution.get('action_id'),physical_reviewed=False)
        self.records.append(record)
        result=dict(schema_version=1,records=list(self.records),physical_reviewed=False)
        result['sha256']=fingerprint(result)
        return result

    def _step(self,error):
        data=self.calibration
        proposed=preview(error,data)
        if proposed['state']!='PRECISION_LIMIT':return proposed
        # 最小1mm处只接受模型预测会减小误差的单轴步，不将未对准谎报为完成。
        a,b=self.calibration['inverse_mm_per_px'][0];c,d=self.calibration['inverse_mm_per_px'][1]
        determinant=a*d-b*c
        jacobian=((d/determinant,-b/determinant),(-c/determinant,a/determinant))
        candidates=[]
        for axis in (0,1):
            for sign in (-1,1):
                mm=sign*data['min_step_mm']
                predicted=math.hypot(*(error[i]+jacobian[i][axis]*mm for i in (0,1)))
                candidates.append((predicted,axis,mm))
        predicted,axis,mm=min(candidates)
        if predicted>=math.hypot(*error)-.1:return proposed
        command=[0.,0.];command[axis]=mm
        return dict(state='STEP',command_mm=command)

    def _turn(self,arrival,link,degrees,now):
        if self.auto_steps>=12:
            self.pause('达到本轮12步预算，保持停车');return None
        degrees=max(-3.,min(3.,degrees))
        degrees=math.copysign(max(.1,abs(degrees)),degrees)
        self.pending_measure=None
        try:arrival.move(link,'yaw',degrees,now)
        except ValueError as exc:self.pause(str(exc));return None
        self.auto_steps+=1;self.previous_radius=None;self.not_improving=0
        self.status='MOVING';self.key=None
        return None

    def advance(self,arrival,link,snapshot,now):
        """只消费后台缓存，最多排一个原Arrival目标；返回待异步保存的数据。"""
        if self.status in ('IDLE','CANCELLED','CENTERED'):return None
        if not arrival.active:
            if arrival.phase=='DONE' and self.status=='RETURNING':
                self.status='CENTERED'
                self.completed=dict(observation=copy.deepcopy(self.last_observation),binding_sha256=self.binding,
                    boot_id=str(link.boot),session_id=str(link.session),zero_version=link.actual.get('zero_version'),
                    mechanical_actions_sent=False,physical_validated=False)
            elif arrival.phase=='FAILED':self.cancel(arrival.reason)
            return None
        if self.status=='PAUSED':return None
        if self.mode=='auto' and now-self.started>60:
            self.pause('自动对中超过本轮60秒预算，保持停车，未自动重启');return None
        if arrival.phase!='READY':
            self.key=None
            if self.status!='RETURNING':self.status='MOVING' if arrival.phase in ('GOAL','MOVING') else 'PREPARING'
            return None
        yaw=link.execution.get('yaw_rad')
        if self.mode=='auto':
            if (not link.execution.get('pose_valid') or type(yaw) not in (int,float) or not math.isfinite(yaw)):
                self.pause('底盘位姿反馈无效，保持停车');return None
            if self.reference_yaw is None:self.reference_yaw=yaw+self.yaw_correction
            difference=math.atan2(math.sin(yaw-self.reference_yaw),math.cos(yaw-self.reference_yaw))
            if abs(difference)>math.radians(3.2):
                self.pause('车头偏转超过本轮3度修正范围，保持停车');return None
            if abs(difference)>math.radians(.3):
                # 先修航向再看画面：夹爪中心与车体中心不重合，航向误差会变成夹爪处横向误差。
                return self._turn(arrival,link,-math.degrees(difference),now)
        key=(arrival.action_id,arrival.steps)
        if self.key!=key:
            self.key=key;self.barrier+=1;self.floor=now+.05;self.ready_at=now
            self.last_observation=None;self.status='OBSERVING'
            if self.observer:self.observer.history.clear()
        if not snapshot:
            if self.mode=='auto' and now-self.ready_at>3:self.pause('停车后未取得可用圆环新帧；保持停车')
            return None
        value,received=snapshot
        try:o=self._observe(value,received,now)
        except FrameNotFresh as exc:
            self.reason=str(exc)
            if self.mode=='auto' and now-self.ready_at>3:self.pause('停车后没有新鲜圆环图像；保持停车')
            return None
        except (ValueError,TypeError,KeyError) as exc:
            if self.mode=='auto':self.pause(str(exc))
            else:self.reason=str(exc)
            return None
        if o is None:return None
        self.last_observation=o
        o['yaw_rad']=yaw
        if not o['stable']:
            if self.mode=='auto' and now-self.ready_at>3:self.pause('2号圆环缺失、遮挡或不稳定；保持停车')
            return None
        saved=self._record(link,o)
        if self.mode!='auto':self.status='READY';self.reason='';return saved
        radius=math.hypot(*o['delta_px'])
        if (radius<=self.calibration['deadband_px'] and
                o['window_max_error_px']>self.calibration['deadband_px']):
            return saved # 必须整组停车后新帧均在容差内，不能只挑最后一帧。
        self.trace_serial+=1
        self.trace.append(dict(id=self.trace_serial,error_px=round(radius,3),action_id=arrival.action_id,
            barrier=self.barrier,center_px=o['center_px'],target_px=o['image_center_px']))
        if self.previous_radius is not None:
            if radius>self.previous_radius*1.6+2:
                self.pause('误差增大，检查方向映射；未继续追赶');return saved
            self.not_improving=self.not_improving+1 if radius>=self.previous_radius-.25 else 0
            if self.not_improving>=2:self.pause('连续两步未收敛，保持停车');return saved
        proposed=self._step(o['delta_px'])
        if proposed['state']=='DONE':
            self.status='RETURNING';self.reason='2号圆环已进入画面中心容差，正在交回机构'
            arrival.finish();return saved
        if proposed['state']!='STEP':
            self.pause('偏差超出已标定范围或小于可改善的最小步长；保持停车');return saved
        x,y=proposed['command_mm'];axis='x' if x else 'y';mm=x or y
        if self.auto_steps>=12 or self.total_mm+abs(mm)>40:
            self.pause('达到本轮12步/40毫米预算，保持停车');return saved
        self.note_move(link,axis,mm,now)
        try:arrival.move(link,axis,mm,now)
        except ValueError as exc:self.pause(str(exc));self.pending_measure=None;return saved
        self.previous_radius=radius;self.auto_steps+=1;self.total_mm+=abs(mm)
        self.status='MOVING';self.key=None
        return saved
