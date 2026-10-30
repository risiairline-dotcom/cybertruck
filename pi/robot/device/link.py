"""控制核心推进会话与保活；网页不访问串口。借鉴旧Web/link.py时效检查。"""

import secrets
import struct
from collections import deque

from . import protocol as p


class Link:
    def __init__(self, capabilities=1):
        self.capabilities = capabilities
        self.decoder = p.Decoder()
        self.events = deque(maxlen=32)
        self.request = 0
        self.boot = self.generation = None
        self.session = secrets.randbits(64) or 1
        self.bound = False
        self.pending = None
        self.progress = 0
        self.last_heart = self.last_state = self.last_diag = -1.0
        self.state_at = None
        self.board = None
        self.firmware = ''
        self.diagnostics = {}
        self.message = '等待新协议握手'
        self.now = self.retry_at = 0.0

    def reset(self, reason):
        self.boot = self.generation = None
        self.session = secrets.randbits(64) or 1
        self.bound = False
        self.pending = None
        self.state_at = None
        self.board = None
        self.firmware = ''
        self.diagnostics = {}
        self.message = reason
        self.retry_at = self.now + .25
        if not self.events or self.events[-1] != reason:
            self.events.append(reason)

    def step(self, now):
        self.now = now
        if now < self.retry_at:
            return b''
        if self.pending:
            if now - self.pending[1] < .25:
                return b''
            self.reset('应答超时，重新核对会话')
            return b''
        if self.boot is None:
            kind, payload = p.HELLO, b''
        elif not self.bound:
            kind, payload = p.OPEN, struct.pack('<I', self.generation)
        elif now - self.last_heart >= .1:
            self.progress = (self.progress + 1) & 0xFFFFFFFF
            kind, payload = p.HEART, struct.pack('<I', self.progress)
            self.last_heart = now
        elif (extra := self.extra_request(now)) is not None:
            kind, payload = extra
        elif now - self.last_state >= .05:
            kind, payload = p.STATE, b''
            self.last_state = now
        elif now - self.last_diag >= .2:
            kind, payload = p.DIAG, b''
            self.last_diag = now
        else:
            return b''
        self.request = (self.request + 1) & 0xFFFFFFFF
        packet = p.Packet(kind, self.boot or 0, 0 if kind == p.HELLO else self.session,
                          self.request, 250, payload)
        self.pending = (packet, now)
        return packet.encode()

    def feed(self, data, now):
        self.now = now
        for packet in self.decoder.feed(data):
            if not self.pending:
                continue
            request, sent = self.pending
            if (packet.kind, packet.request, packet.session) != (
                    request.kind | 0x80, request.request, request.session):
                continue
            self.pending = None
            # now 是读出时刻，为到达时间的保守上界；超过期限的回复不能刷新状态。
            if now - sent >= .25:
                self.reset('应答超时，重新核对会话')
                continue
            if self.custom_reply(packet, request, now, sent):
                continue
            if not packet.payload or packet.payload[0] != p.OK:
                self.reset('设备拒绝请求，等待重新握手')
                continue
            payload = packet.payload[1:]
            if request.kind == p.HELLO:
                if len(payload) != 24 or not packet.boot:
                    self.reset('设备启动身份或HELLO长度无效')
                    continue
                generation, capabilities = struct.unpack_from('<II', payload)
                if capabilities != self.capabilities:
                    self.reset('设备能力不符合当前只读阶段')
                    continue
                self.boot, self.generation = packet.boot, generation
                self.firmware = payload[8:24].rstrip(b'\0').decode('ascii', errors='replace')
            elif packet.boot != self.boot:
                self.reset('设备启动身份改变，旧会话已失效')
            elif request.kind == p.OPEN:
                if payload:
                    self.reset('OPEN应答长度无效')
                    continue
                self.bound = True
                self.progress = 0
                self.last_heart = self.last_state = self.last_diag = -1.0
                self.message = '只读会话已建立，等待状态'
            elif request.kind == p.STATE:
                if len(payload) != p.STATE_DATA.size:
                    self.reset('状态长度无效')
                    continue
                values = p.STATE_DATA.unpack(payload)
                self.board = dict(zip(('uptime_ms', 'fault_bits', 'flags', 'heartbeats',
                                       'rx_good', 'rx_bad', 'rejected'), values))
                if self.board['flags'] & 3 != 3:
                    self.reset('设备不处于已绑定只读模式')
                    continue
                self.state_at = now
                self.message = '只读通信正常' if not self.board['fault_bits'] else '设备报告锁存故障'
            elif request.kind == p.DIAG:
                if len(payload) != 8:
                    self.reset('诊断长度无效')
                    continue
                dropped, generation = struct.unpack('<II', payload)
                self.diagnostics = dict(tx_replaced=dropped, generation=generation)
            elif request.kind == p.HEART and payload:
                self.reset('HEART应答长度无效')

    def extra_request(self, now):
        return None

    def custom_reply(self, packet, request, now, sent=None):
        return False

    def snapshot(self, now):
        age = None if self.state_at is None else max(0, int((now - self.state_at) * 1000))
        fresh = self.bound and age is not None and age < 250
        # 未接入的物理状态明确未知，不能把初始化0当成测量。
        return dict(connected=fresh, state_age_ms=age, readonly=True, ready=False,
                    message=self.message, firmware=self.firmware,
                    boot_id=None if self.boot is None else f'{self.boot:016x}',
                    session_id=f'{self.session:016x}' if self.bound else None,
                    board=self.board if fresh else None, diagnostics=self.diagnostics,
                    pose=None, pose_valid=False, zero_valid=False,
                    physical_stop_confirmed=False, task=None, grabs=None, places=None,
                    rx_good=self.decoder.good, rx_bad=self.decoder.bad, events=list(self.events))
