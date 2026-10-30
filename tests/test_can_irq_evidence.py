"""用库内原版HAL中断函数回放RAM标志；只构建主机夹具，不改固件。"""
import re
from pathlib import Path
import subprocess
import tempfile
import unittest

from test_execution import ROOT, compile_c


def function(source, name):
    start=source.index('void '+name+'(')
    brace=source.index('{',start); depth=1; end=brace+1
    while depth:
        depth += (source[end]=='{')-(source[end]=='}'); end+=1
    return source[start:end]


def constants(source, headers):
    definitions={}
    for path in headers:
        for name,value in re.findall(r'^#define\s+([A-Z][A-Za-z0-9_]+)\s+([^\r\n]+)',path.read_text(encoding='utf-8'),re.M):
            definitions[name]=re.sub(r'/\*.*','',value).strip()
    names=set(re.findall(r'\b(?:HAL_CAN_[A-Za-z0-9_]+|CAN_[A-Za-z0-9_]+)\b',source))
    selected={}
    while names:
        name=names.pop()
        if name in selected or name not in definitions: continue
        selected[name]=definitions[name]
        names.update(re.findall(r'\b(?:HAL_CAN_[A-Za-z0-9_]+|CAN_[A-Za-z0-9_]+)\b',definitions[name]))
    return '\n'.join(f'#ifndef {name}\n#define {name} {value}\n#endif' for name,value in selected.items())


class CanIrqEvidenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.folder=tempfile.TemporaryDirectory(dir=ROOT/'output/tests')
        cls.output=Path(cls.folder.name)
        cls.hal=ROOT/'firmware/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c'
        cls.body=function(cls.hal.read_text(encoding='utf-8'),'HAL_CAN_IRQHandler')
        header=constants(cls.body,[ROOT/'firmware/Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h',
                                   ROOT/'firmware/Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_can.h'])
        (cls.output/'can_irq_constants.h').write_text(header,encoding='utf-8')
        cls.irq=cls.output/'hal_irq.c'
        cls.irq.write_text('#include "can_irq_stub.h"\n'+cls.body,encoding='utf-8')
        cls.binary=cls.output/'irq.exe'
        cls.build(cls.binary,ROOT/'firmware/User/Device/can_tx.c')

    @classmethod
    def build(cls,binary,transport):
        compile_c(binary,[transport,cls.irq,ROOT/'tests/can_irq_evidence.c'],
                  extra=['-I',str(ROOT/'tests'),'-I',str(cls.output),'-include',str(ROOT/'tests/can_irq_stub.h')])

    @classmethod
    def tearDownClass(cls):
        cls.folder.cleanup()

    def run_case(self,index):
        result=subprocess.run([str(self.binary),str(index)],capture_output=True)
        self.assertEqual(result.returncode,0,result.stderr.decode('utf-8',errors='replace'))

    def test_observed_terr0_lec4_no_motion_replay(self): self.run_case(0)
    def test_optional_lecie_changes_hal_detail_not_failure(self): self.run_case(1)
    def test_pure_arbitration_remains_bounded_retry(self): self.run_case(2)
    def test_lec_without_transmit_error_does_not_fake_terr(self): self.run_case(3)
    def test_success_with_old_lec_does_not_fake_error(self): self.run_case(4)

if __name__=='__main__': unittest.main()
