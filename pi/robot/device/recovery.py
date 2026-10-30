"""人工故障恢复：先归档首次CAN现场，再显式重启；绝不重放动作。"""
import copy
import json
from pathlib import Path
import secrets
import struct

from .can_capture import CaptureReader, decode_page
from .capabilities import firmware_since


class Recovery:
    ACTIVE = ('CAPTURE', 'REQUEST', 'REBOOT', 'VERIFY', 'ATTACH')

    def __init__(self, runtime=None):
        self.runtime = Path(runtime)/'fault_recovery' if runtime else None
        self.status = 'IDLE'
        self.reason = ''
        self.record = {}
        self.path = None
        self.pending = None
        self.sent_at = self.started = self.ready_since = 0.
        self.reader = CaptureReader()

    @property
    def active(self):
        return self.status in self.ACTIVE

    def view(self):
        return dict(status=self.status, reason=self.reason, id=self.record.get('id'),
                    saved=bool(self.path), authorizes_motion=False)

    def save(self):
        if self.runtime is None:
            raise ValueError('故障恢复必须配置持久归档目录')
        self.runtime.mkdir(parents=True, exist_ok=True)
        if self.path is None:
            self.path = self.runtime/(self.record['id']+'.json')
        from ..workbench.storage import write_json
        write_json(self.path, dict(self.record, status=self.status, reason=self.reason))

    def start(self, link, args, now):
        m = link.actual
        if (self.active or not firmware_since(link.firmware, 33) or not link.commissioning or
                link.latched or not link.bound or not link.action_locked or not link.stop_acknowledged or
                now-link.actual_at >= .25 or now-link.base_at >= .25 or not link.base.get('stopped') or
                not m.get('attached') or m.get('config_mask') != 3 or
                args.get('confirmed') is not True or args.get('wiring_repaired') is not True or
                args.get('outputs_supported') is not True or args.get('boot_id') != str(link.boot) or
                args.get('session_id') != str(link.session) or args.get('zero_version') != m.get('zero_version') or
                link.commands or link.operation or link.stop_queued):
            raise ValueError('恢复要求0.3.33、已停止的同一新鲜会话，并确认修线及PWM断开后的机构支撑')
        self.record = dict(id=secrets.token_hex(12), boot=str(link.boot), session=str(link.session),
                           before=copy.deepcopy(link.view(now)), can_pages=[], reset_sent=False)
        self.status, self.reason = 'CAPTURE', '保存首次CAN错误和故障前收发历史'
        self.started = now
        self.ready_since = 0.
        self.reader = CaptureReader()
        self.pending = None
        try:
            self.save()  # 保存失败不排入任何设备命令。
        except OSError as exc:
            self.status = 'FAILED'
            raise ValueError('故障归档失败，未开始恢复') from exc

    def fail(self, reason):
        self.status, self.reason = 'FAILED', reason
        self.save()
        return reason

    def cancel(self, reason):
        if self.active:
            self.status, self.reason = 'CANCELLED', reason
            try:
                self.save()
            except OSError:
                self.reason += '；取消记录写入失败，停止仍执行'

    def advance(self, core, now):
        if not self.active:
            return None
        link = core.link
        if now-self.started > 35:
            return self.fail('恢复超时；不自动重启或重试')
        if self.status == 'REBOOT':
            if now-self.sent_at < .5:
                return None
            from .mechanism import MechanismLink
            core.link = MechanismLink(commissioning=True)
            self.status, self.reason = 'VERIFY', '核对新启动身份、停车及无故障状态'
            self.pending = None
            self.save()
            return None
        if self.status in ('VERIFY', 'ATTACH'):
            if link.latched:
                return self.fail('重连失败；保持停止，不自动再次重启')
            state = link.view(now)
            if not state['connected']:
                self.ready_since = 0.
                return None
            if str(link.boot) == self.record['boot']:
                return self.fail('未确认新的启动身份，恢复未完成')
            if not firmware_since(link.firmware, 33) or link.base.get('fault') or link.actual.get('fault'):
                return self.fail('重启后故障仍存在；检查线路，不自动重试')
            if not link.base.get('stopped'):
                return None
            if self.status == 'VERIFY':
                if link.operation or link.commands:
                    return None
                link.submit('recovery_attach_'+self.record['id'], 'attach', {}, now)
                self.status = 'ATTACH'
                return None
            m = link.actual
            if not m.get('attached') or m.get('config_mask') != 3:
                return None
            if m.get('pwm_mask') or m.get('zero_valid') or any(m.get(k) for k in ('axis_id','servo_id','pose_id')):
                return self.fail('恢复后输出/零点状态不符合初始准备条件')
            if not self.ready_since:
                self.ready_since = now
            if now-self.ready_since >= 1:
                self.status, self.reason = 'DONE', '恢复检查通过；请自行获取控制权并重新建零、记录参考'
                self.record['after'] = copy.deepcopy(state)
                self.save()
            return None
        if link.latched or str(link.boot) != self.record['boot'] or str(link.session) != self.record['session']:
            return self.fail('归档期间设备身份或连接变化，未继续恢复')
        if link.actual.get('zero_version') != self.record['before']['mechanism']['zero_version']:
            return self.fail('恢复期间零点版本变化，未继续重启')
        if self.pending:
            reply = link.inspections.pop(self.pending, None)
            if reply is None:
                if now-self.sent_at > 1:
                    return self.fail('恢复请求无明确结果；不重发')
                return None
            self.pending = None
            if self.status == 'CAPTURE':
                data = bytes.fromhex(reply['data'])
                try:
                    page = decode_page(data, self.reader.next_page)
                    self.record['can_pages'].append(page)
                    self.reader.feed(data, link.boot, link.session)
                except ValueError:
                    return self.fail('CAN故障分页校验失败，未发送重启')
                if not page['valid'] or self.reader.result:
                    self.record['can_capture'] = self.reader.result or self.reader.header
                    self.status, self.reason = 'REQUEST', '故障记录已保存，准备关闭PWM并重启控制器'
                self.save()
            else:
                if reply.get('result') != 0:
                    return self.fail('板端拒绝恢复；静止反馈或输出关闭条件未满足')
                self.status, self.reason = 'REBOOT', '板端已接受，等待新的启动身份'
                self.sent_at = now
                link.recovery_wait = True
                self.save()
                return None
        # 分页之间让正式35/58轮询推进，避免用归档请求挤掉实时状态。
        if (link.commands or link.operation or now-link.actual_at >= .12 or now-link.base_at >= .12):
            return None
        self.pending = 'recovery_'+self.record['id']+'_'+str(len(self.record['can_pages']))+'_'+self.status
        self.sent_at = now
        if self.status == 'CAPTURE':
            link.commands.append((38, bytes([self.reader.next_page]), self.pending))
        else:
            if not link.base.get('stopped') or not link.stop_acknowledged:
                return self.fail('归档后停车状态变化，未发送重启')
            self.record['reset_sent'] = True
            self.save()
            if link.token >= 0xfffffff0:
                return self.fail('恢复请求编号耗尽，未发送重启')
            link.token += 1
            link.commands.append((65, struct.pack('<4I', link.token,
                                  0x52454356, 7, link.actual['zero_version']), self.pending))
        return None
