"""厂家格式的合成雷达报文夹具。"""

import binascii
import struct

from .protocol import cobs_encode

CASES = dict(detour='上侧道路有障碍', clear='完整空场', blocked='起点被占用',
             partial='覆盖不足', bad_crc='CRC损坏', missing='缺失点云块',
             low_quality='低置信度', unregistered='没有标定', wrong_zone='区号不匹配')


def fixture(case, start, roads):
    if case not in CASES or start not in ('start_1','start_2'):
        raise ValueError('离线雷达案例或起点无效')
    zone = 1 if start == 'start_1' else 2
    profile = dict(domain='simulation', synthetic=True, source='synthetic_vendor_v6', node=1,
        vendor_zone=zone, start=start, version=f'synthetic-zone-{zone}-v1',
        map_transform=[.1,.2,0.], diameter_m=.05, uncertainty_m=.01,
        coverage=list(roads), coverage_kind='synthetic_full_roads', empty_coverage_valid=True)
    if case == 'partial':
        profile.update(coverage=[], coverage_kind='unknown', empty_coverage_valid=False)
    if case == 'unregistered':
        profile['map_transform'] = None
    x, y = (.65 if zone == 1 else 1.75), 2.1
    if case == 'blocked':
        x, y = (.15 if zone == 1 else 2.25), 2.25
    objects = [] if case in ('clear','partial') else [(round((x-.1)*1000),round((y-.2)*1000))]
    packets = []
    def event(kind, payload):
        raw = struct.pack('<2sBBBHH',b'LR',6,1,kind,len(packets)+1,len(payload))+payload
        packets.append(cobs_encode(raw+struct.pack('<H',binascii.crc_hqx(raw,0xffff)))+b'\0')
    event(0x81,struct.pack('<BBBBhhHBH',1,37,6,3-zone if case == 'wrong_zone' else zone,0,0,0,0,120))
    event(0x82,b'\3\0')
    for index in range(14):
        seq = index+101
        points = [(x+dx,y+dy,100) for x,y in objects for dx,dy in ((-20,0),(0,20),(20,0),(0,-20))]
        event(0x84,struct.pack('<IIHHH',seq,1000+index*100,100,len(points),max(100,len(points))))
        if case != 'missing' or index != 4:
            event(0x85,struct.pack('<IHB',seq,0,len(points))+b''.join(struct.pack('<hhB',*p) for p in points))
        event(0x86,struct.pack('<IHBH',seq,len(points),0,200))
        event(0x8D,struct.pack('<IB',seq,len(objects))+bytes(range(len(objects))))
        event(0x87,struct.pack('<IBBBB',seq,14,0,0,len(objects))+b''.join(
            struct.pack('<hhBBH',x,y,14,30 if case == 'low_quality' else 95,24) for x,y in objects))
    event(0x82,b'\4\0')
    event(0x83,bytes([3,0,0,14,0]))
    if case == 'bad_crc':
        # 破坏封包体，保留分隔符；解码器必须拒绝，不能把丢失目标当空场。
        damaged = bytearray(packets[10]); damaged[-3] ^= 1; packets[10] = bytes(damaged)
    return b''.join(packets), profile
