"""基于本次AXF符号不停核读取机构/底盘/CAN；不打开串口、不写寄存器。"""
import argparse
import hashlib
import json
from pathlib import Path
import time
from elftools.elf.elffile import ELFFile

ROOT=Path(__file__).resolve().parents[1]


def layout_type(die):
    """返回真实类型大小和数组标志；数组缺byte_size时不能误用单元素大小。"""
    if die.tag == 'DW_TAG_array_type':
        item_size, _ = layout_type(die.get_DIE_from_attribute('DW_AT_type'))
        count = 1
        dimensions = 0
        for bound in die.iter_children():
            if bound.tag != 'DW_TAG_subrange_type':
                continue
            dimensions += 1
            values = bound.attributes
            if 'DW_AT_count' in values:
                count *= values['DW_AT_count'].value
            elif 'DW_AT_upper_bound' in values:
                lower = values['DW_AT_lower_bound'].value if 'DW_AT_lower_bound' in values else 0
                count *= values['DW_AT_upper_bound'].value - lower + 1
            else:
                raise ValueError('数组缺少长度，拒绝猜测布局')
        if not dimensions:
            raise ValueError('数组缺少维度，拒绝猜测布局')
        return item_size * count, True
    if 'DW_AT_type' in die.attributes and 'DW_AT_byte_size' not in die.attributes:
        return layout_type(die.get_DIE_from_attribute('DW_AT_type'))
    return die.attributes['DW_AT_byte_size'].value, False


def main():
    # pyocd 只在连接探针时需要；layout_type 供主机测试单独使用。
    from pyocd.core.helpers import ConnectHelper
    from pyocd.core.target import Target
    parser=argparse.ArgumentParser()
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=30.)
    parser.add_argument('--build-dir',type=Path,default=ROOT/'firmware/MDK-ARM/Objects',
                        help='对应已烧录HEX与AXF的归档目录，不允许用新候选符号读取旧板端')
    args=parser.parse_args()
    manifest=json.loads(args.manifest.read_text(encoding='utf-8'))
    if not manifest.get('verified') or not manifest.get('resumed_after_verify'):
        raise ValueError('必须先完成本次烧录和完整读回')
    if hashlib.sha256((args.build_dir/'Car.hex').read_bytes()).hexdigest()!=manifest['image_sha256']:
        raise ValueError('当前HEX与已读回固件不一致')
    if args.output.exists():
        raise ValueError('不覆盖旧证据')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    layouts={}
    names={'can_diag':'CanDiag','bus_diag':'BusDiag','executor':'Exec','guard':'MechGuard','axis':'Axis','servo':'Servo','pose':'Pose'}
    with (args.build_dir/'Car.axf').open('rb') as stream:
        elf=ELFFile(stream)
        table=elf.get_section_by_name('.symtab')
        if table.get_symbol_by_name('bus_timeout'):
            names['bus_timeout']='BusTimeout'
        symbols={name:dict(address=max(table.get_symbol_by_name(name),key=lambda s:s['st_size'])['st_value']) for name in names}
        for cu in elf.get_dwarf_info().iter_CUs():
            for die in cu.iter_DIEs():
                if die.tag!='DW_TAG_typedef' or 'DW_AT_name' not in die.attributes:continue
                name=die.attributes['DW_AT_name'].value.decode()
                if name not in set(names.values())|{'BusWheel','MotorReply'}:continue
                kind=die.get_DIE_from_attribute('DW_AT_type')
                fields={}
                for member in kind.iter_children():
                    if member.tag!='DW_TAG_member':continue
                    typ=member.get_DIE_from_attribute('DW_AT_type')
                    size, is_array = layout_type(typ)
                    fields[member.attributes['DW_AT_name'].value.decode()]=dict(offset=member.attributes['DW_AT_data_member_location'].value,size=size,array=is_array)
                layouts[name]=fields
        wheel_symbol=max(table.get_symbol_by_name('wheels'),key=lambda s:s['st_size'])
        wheel_address=wheel_symbol['st_value'];wheel_stride=wheel_symbol['st_size']//4
    options=dict(pack=manifest['pack'],target_override='stm32f407zg',frequency=100000,
                 connect_mode='attach',halt_on_connect=False,resume_on_disconnect=False,auto_unlock=False,config_file=None)
    session=ConnectHelper.session_with_chosen_probe(unique_id=manifest['probe'],blocking=False,options=options)
    record=dict(source='running SWD; no UART',writes=0,halted=False,reset=False,manifest=str(args.manifest),
                build_dir=str(args.build_dir),symbols=symbols,layouts=layouts,samples=[])
    with session:
        target=session.target
        if target.get_state()!=Target.State.RUNNING:raise RuntimeError('CPU未运行，不自动恢复')
        if bytes(target.read_memory_block8(0x1FFF7A10,12)).hex()!=manifest['uid']:raise RuntimeError('UID不符')
        def fields(address,layout):
            result={}
            for name,item in layout.items():
                if item['size'] in (1,2,4,8) and not item.get('array'):
                    result[name]=int.from_bytes(bytes(target.read_memory_block8(address+item['offset'],item['size'])),'little')
            return result
        start=time.monotonic()
        while time.monotonic()-start<args.seconds:
            row=dict(elapsed_s=time.monotonic()-start)
            for name,typ in names.items():row[name]=fields(symbols[name]['address'],layouts[typ])
            row['wheels']=[fields(wheel_address+i*wheel_stride+layouts['BusWheel']['speed']['offset'],layouts['MotorReply']) for i in range(4)]
            row['pwm_ccer']=target.read32(0x40010020)
            row['pwm_cr1']=target.read32(0x40010000)
            record['samples'].append(row)
            time.sleep(.2)
        record['cpu_running_after']=target.get_state()==Target.State.RUNNING
        record['duration_s']=time.monotonic()-start
    rows=record['samples']
    record['summary']=dict(samples=len(rows),duration_s=record['duration_s'],
        can_errors=[rows[0]['can_diag']['errors'],rows[-1]['can_diag']['errors']],
        rx_increase=rows[-1]['can_diag']['rx_sequence']-rows[0]['can_diag']['rx_sequence'],
        faults=list(sorted(set(r['executor']['fault'] for r in rows))),
        stopped_all=all(r['executor']['stop_confirmed']==1 for r in rows),
        enabled_all_zero=all(not r['executor']['enabled_wheels'] for r in rows),
        pwm_off=all(not(r['pwm_ccer']&0x5555) for r in rows),
        mechanism_zero_invalid=all(not r['guard']['zero_valid'] for r in rows),
        wheel_rpm_zero=all(all(w['valid'] and w['value']==0 for w in r['wheels']) for r in rows))
    args.output.write_text(json.dumps(record,indent=2),encoding='utf-8')
    print(json.dumps(record['summary'],indent=2))


if __name__=='__main__':main()
