"""实车联调表：只调度正式联调组，不在网页计时或生成模拟完成。"""
import copy
import hashlib
import json

from .capabilities import firmware_since

NAMES = ('lift','reach','tray','arm','gripper')

def groups(rows, mode='ordered'):
    if mode not in ('ordered','together') or not isinstance(rows,list) or not 1 <= len(rows) <= 40:
        raise ValueError('需要1～40行动作，并选择顺序或全部同时')
    stages = {}
    for row in rows:
        if not isinstance(row,dict) or row.get('axis') not in NAMES or type(row.get('order')) is not int or not 1 <= row['order'] <= 999:
            raise ValueError('每行需填写1～999整数顺序及有效执行器')
        order = row['order'] if mode == 'ordered' else 1
        members = stages.setdefault(order,[])
        if any(m['axis']==row['axis'] for m in members):
            raise ValueError('同一同时组重复执行器；请拆为不同顺序号')
        members.append({k:copy.deepcopy(v) for k,v in row.items() if k in ('axis','target_m','target_rad','rpm','speed_rad_s','accel')})
    return [dict(order=n,members=stages[n]) for n in sorted(stages)]

def record(name, rows, revision):
    if not isinstance(name,str) or not name or len(name)>48 or not all(c.isalnum() or c=='_' for c in name):
        raise ValueError('联调表名称需1～48个字母、数字、汉字或下划线')
    groups(rows)
    value = dict(id=name,revision=revision,rows=copy.deepcopy(rows),domain='physical_draft',physical_validated=False)
    value['sha256'] = digest(value)
    return value

def digest(value):
    body = {k:v for k,v in value.items() if k!='sha256'}
    return hashlib.sha256(json.dumps(body,sort_keys=True,ensure_ascii=True,allow_nan=False).encode()).hexdigest()


class Sequence:
    def __init__(self):
        self.status='IDLE'
        self.reason=''
        self.index=0
        self.plan=[]
        self.request=''
        self.group_id=0
        self.identity=None
        self.run_id=0
        self.saved={}
        self.queued_at=0.
        self.deadline=0.
        self.wait_started=0.
        self.retry_after=-1.
        self.attempt=0

    def view(self):
        return dict(status=self.status,reason=self.reason,index=self.index,count=len(self.plan),
                    order=self.plan[self.index]['order'] if self.index<len(self.plan) else None,
                    id=self.saved.get('id'),revision=self.saved.get('revision'),group_id=self.group_id,
                    completion='servo_command_only')

    def start(self, value, mode, link, now):
        from .mechanism import payload
        if self.status=='RUNNING':
            raise ValueError('联调已运行；先停止，禁止重复执行')
        if digest(value)!=value.get('sha256'):
            raise ValueError('保存的联调版本校验不符')
        plan=groups(value['rows'],mode)
        # 所有步骤先做纯编码校验，避免执行第一步后才发现后面的非法目标。
        for stage in plan:
            payload('group',dict(members=stage['members'],zero_version=link.actual.get('zero_version'),
                                 _firmware=link.firmware),link.actual,1)
        if (not firmware_since(link.firmware, 32) or link.home.get('state') in (1,2,3) or not link.manual_debug or link.action_locked or link.latched or
                not link.commissioning or not link.bound or now-link.actual_at>=.25 or now-link.base_at>=.25 or
                now-link.group_at>=.25 or link.group.get('state')==1 or link.actual.get('fault') or
                not link.actual.get('zero_valid') or link.base.get('fault') or not link.base.get('stopped') or
                not link.actual.get('base_stopped') or not link.actual.get('attached') or link.actual.get('config_mask')!=3 or
                any(1<=link.actual.get(k,0)<=5 for k in ('axis_state','servo_state','pose_state'))):
            raise ValueError('联调要求0.3.32手动模式、有效零点及新鲜无故障停车反馈')
        if link.commands or link.operation:
            raise ValueError('上一请求尚未完成')
        self.saved=copy.deepcopy(value)
        self.plan=plan
        self.index=0
        self.request=''
        self.group_id=0
        self.run_id+=1
        self.status='RUNNING'
        self.reason=''
        self.identity=(link.boot,link.session,link.actual['zero_version'])
        self.wait_started=now
        self.retry_after=-1.
        self.attempt=0

    def cancel(self, reason):
        if self.status=='RUNNING':
            self.status='CANCELLED'
            self.reason=reason

    def advance(self, link, now):
        """由唯一设备核心周期调用；返回错误时调用方统一STOP，不续跑。"""
        if self.status!='RUNNING':
            return ''
        def fail(reason):
            self.status='FAILED'
            self.reason=reason
            return reason
        if (self.identity!=(link.boot,link.session,link.actual.get('zero_version')) or not link.manual_debug or
                link.latched or link.action_locked or link.actual.get('fault') or link.base.get('fault') or
                not link.actual.get('zero_valid') or not link.base.get('stopped') or
                now-link.actual_at>=.25 or now-link.base_at>=.25 or now-link.group_at>=.25):
            if link.actual.get('fault'):
                return fail(f'机构故障{link.actual["fault"]}，停止整组')
            if link.base.get('fault'):
                return fail(f'底盘故障{link.base["fault"]}，停止整组')
            reasons=[]
            for name,at in (('机构',link.actual_at),('底盘',link.base_at),('联调组',link.group_at)):
                if now-at>=.25:reasons.append(f'{name}状态过期{round((now-at)*1000)}ms')
            if self.identity!=(link.boot,link.session,link.actual.get('zero_version')):reasons.append('启动、会话或零点版本变化')
            if not link.manual_debug:reasons.append('手动模式已退出')
            if link.latched or link.action_locked:reasons.append('通信或动作已锁定')
            if not link.actual.get('zero_valid'):reasons.append('零点无效')
            if not link.base.get('stopped'):reasons.append('底盘停车未确认')
            return fail('联调条件变化：'+'；'.join(reasons)+'；停止整组')
        if self.request:
            result=next((r for r in link.results if r['request_id']==self.request),None)
            if result and result['result']:
                # 原因3表示尚未接纳；仅等待新反馈，绝不重发已接受/运行的组。
                if (result['result']==3 and now-self.wait_started<3. and
                        link.group.get('id')!=self.group_id and link.group.get('state') in (0,2)):
                    self.request=''
                    self.retry_after=now
                    self.reason='等待新反馈确认机构稳定'
                else:
                    return fail(f'联调组被板端拒绝：{result["result"]}')
            if not self.request:
                return ''
            if link.group.get('id')!=self.group_id:
                if now-self.queued_at>1.:
                    return fail('联调组身份反馈超时；停止且不重发')
                return ''
            if link.group.get('state') in (3,4):
                return fail(f'联调组停止：{link.group.get("reason")}')
            if not result or link.group.get('state')!=2:
                if now>self.deadline:
                    return fail('联调组完成超时；停止且不重发')
                return ''
            self.index+=1
            self.request=''
            self.wait_started=now
            self.retry_after=-1.
            self.reason=''
            if self.index==len(self.plan):
                self.status='DONE'
                return ''
        if now-self.wait_started>=3.:
            return fail('组间稳定等待超过3秒；停止且不重放动作')
        if (link.commands or link.operation or link.actual.get('stable_samples',0)<(2 if firmware_since(link.firmware, 39) else 5) or
                link.actual_at<=self.retry_after):
            return ''
        self.attempt+=1
        self.request=f'program_{self.run_id}_{self.index}_{self.attempt}'
        self.reason=''
        try:
            link.submit(self.request,'group',dict(members=self.plan[self.index]['members'],zero_version=self.identity[2]),now)
            import struct
            self.group_id=struct.unpack_from('<I',link.commands[-1][1],8)[0]
            self.queued_at=now
            self.deadline=now+struct.unpack_from('<I',link.commands[-1][1],12)[0]/1000+1.
        except ValueError as exc:
            return fail(str(exc))
        return ''
