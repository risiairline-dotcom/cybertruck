"""只读核对硬件安全规则与 Keil 工程引用；不编译、不启动设备、不写入工程。"""

import json
import re
import sys
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
FW = ROOT / "firmware"
ERRORS = []
CHECKS = 0


def check(ok, message):
    """累计检查结果，一次列出全部问题。"""
    global CHECKS
    CHECKS += 1
    if not ok:
        ERRORS.append(message)


def read(path):
    try:
        return path.read_text(encoding="utf-8-sig")
    except (OSError, UnicodeError) as exc:
        check(False, f"无法读取 {path.relative_to(ROOT)}：{exc}")
        return ""


def compact(value):
    return re.sub(r"\s+", "", value)


def clean_c(source):
    """删除注释，保留字符串，避免把注释里的示例当成执行代码。"""
    pattern = r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|/\*.*?\*/|//[^\n]*'
    return re.sub(pattern, lambda m: " " if m[0].startswith(("/*", "//")) else m[0],
                  source, flags=re.S)


def body(source, name):
    """按配对花括号提取 C 函数体。"""
    match = re.search(r"\b" + re.escape(name) + r"\s*\([^;{}]*\)\s*\{", source)
    if not match:
        check(False, f"缺少函数定义：{name}")
        return ""
    depth = 1
    for pos in range(match.end(), len(source)):
        if source[pos] == "{":
            depth += 1
        elif source[pos] == "}":
            depth -= 1
            if depth == 0:
                return source[match.end():pos]
    check(False, f"函数花括号不完整：{name}")
    return ""


def load_ioc():
    data = {}
    for line in read(FW / "Car.ioc").splitlines():
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        data[key.replace(r"\ ", " ").replace(r"\:", ":")] = value.replace(r"\:", ":")
    return data


def verify_ioc(data):
    """板上晶振、CAN 波特率、PWM 默认占空比及安全相关引脚。"""
    expected = {
        "Mcu.CPN": "STM32F407ZGT6", "RCC.HSE_VALUE": "8000000",
        "RCC.PLLSourceVirtual": "RCC_PLLSOURCE_HSE", "CAN1.NART": "DISABLE",
        "PC0.Signal": "GPIO_Input", "PC0.GPIO_PuPd": "GPIO_PULLUP",
        "PC1.Signal": "GPIO_Input", "PC1.GPIO_PuPd": "GPIO_PULLUP",
        "PG11.Signal": "GPIO_Output", "PG11.PinState": "GPIO_PIN_SET",
        "PD0.Signal": "CAN1_RX", "PD1.Signal": "CAN1_TX",
        "PE9.Signal": "S_TIM1_CH1", "PE11.Signal": "S_TIM1_CH2",
        "PE13.Signal": "S_TIM1_CH3", "PE14.Signal": "S_TIM1_CH4",
        "ProjectManager.KeepUserCode": "true",
    }
    for channel in (1, 2, 3, 4):
        expected[f"TIM1.Pulse-PWM Generation{channel} CH{channel}"] = "0"
    for key, value in expected.items():
        check(data.get(key) == value, f"IOC {key} 应为 {value}，实际 {data.get(key, '缺失')}")
    try:
        rate = int(data["RCC.APB1Freq_Value"]) / int(data["CAN1.Prescaler"]) / 15
        check(rate == 500000, f"IOC CAN 波特率应为 500k，实际 {rate}")
    except (KeyError, ValueError, ZeroDivisionError):
        check(False, "IOC 无法计算 CAN 波特率")


def verify_sources():
    """生成代码中 CubeMX 不会替我们保证的项。"""
    hal_config = clean_c(read(FW / "Core/Inc/stm32f4xx_hal_conf.h"))
    check(bool(re.search(r"#define\s+HSE_VALUE\s+8000000U\b", hal_config)),
          "HAL HSE_VALUE 必须为板上 8MHz 晶振")
    main = read(FW / "Core/Src/main.c")
    for section, pattern in (("2", r"\bApp_Init\s*\(\s*\)\s*;"), ("3", r"\bApp_Run\s*\(\s*\)\s*;")):
        match = re.search(r"/\*\s*USER CODE BEGIN " + section + r"\s*\*/(.*?)/\*\s*USER CODE END "
                          + section + r"\s*\*/", main, re.S)
        check(bool(match) and bool(re.search(pattern, clean_c(match[1]))),
              f"main USER CODE {section} 缺少 App 钩子")


def verify_owners():
    """CAN、PWM、串口各只有一个所有者；启动不使能、不下发动作。"""
    forbidden = (r"\bHAL_CAN_(?:Start|AddTxMessage|ActivateNotification)\s*\(",
                 r"\bHAL_TIM(?:Ex)?_\w*Start\w*\s*\(",
                 r"\bLL_(?:CAN|TIM|USART)_\w+\s*\(")
    can_owner = FW / 'User/Device/can_tx.c'
    can_init = FW / 'User/Board/base_port.c'
    pwm_owner = FW / 'User/Board/mech_port.c'
    for folder in (FW / "Core", FW / "User"):
        for path in folder.rglob("*"):
            if path.suffix.lower() not in (".c", ".h"):
                continue
            source = clean_c(read(path))
            for pattern in forbidden:
                for match in re.finditer(pattern, source):
                    call = match[0]
                    allowed = ((path == pwm_owner and call.startswith('HAL_TIM_PWM_Start')) or
                               (path == can_owner and 'AddTxMessage' in call) or
                               (path == can_init and call.startswith('HAL_CAN_') and 'AddTxMessage' not in call))
                    check(allowed, f"{call} 只能由唯一 CAN/PWM 所有者调用：{path.relative_to(ROOT)}")
            if re.search(r'\bHAL_UART(?:Ex)?_(?:Receive\w*|Transmit\w*)\s*\(', source):
                check(path == FW / 'User/Board/port.c', f'串口只能由 Board/port.c 管理：{path.relative_to(ROOT)}')
    mechanism_port = clean_c(read(pwm_owner))
    check(not re.search(r'\b(?:HAL_\w+|Bus_Send|Servo_Begin|Pose_Begin|Axis_Begin)\s*\(',
                        body(mechanism_port, 'MechPort_Init')), '机构初始化禁止发送、使能或启动 PWM')
    check('TIM_CHANNEL_4' not in mechanism_port, 'PE14 仅配置预留，不并入三个舵机控制')
    base_init = body(clean_c(read(can_init)), 'Base_Init')
    check('Bus_Stop();' in base_init and 'Bus_MechWatch();' in base_init, '启动必须先停止底盘及机构驱动目标')
    check(not re.search(r'\b(?:Bus_(?:Start|Speed|Send)|Exec_(?:Arm|Goal))\s*\(', base_init),
          'Base_Init 禁止使能、下发动作或自动开始查询')
    addresses = json.loads(read(ROOT / 'config/base_trials.json'))['addresses']
    check('BASE_SAFE_ADDRESSES{' + ','.join(map(str, addresses)) + '}' in compact(read(FW / 'User/Config/addresses.h')),
          '启动停止地址必须与 config/base_trials.json 一致')


def verify_keil():
    project = FW / "MDK-ARM/Car.uvprojx"
    try:
        xml = ET.fromstring(read(project))
    except ET.ParseError as exc:
        check(False, f"Keil XML 无法解析：{exc}")
        return
    targets = xml.findall("./Targets/Target")
    check(len(targets) == 1, "Keil 必须只有一个 Target")
    if not targets:
        return
    target = targets[0]
    check(target.findtext("TargetName") == "Car", "Keil Target 必须为 Car")
    check(target.findtext(".//uAC6") == "1", "Keil 必须选择 ARM Compiler 6")
    # 编译不得顺带下载或进入调试。
    for tag in ("BeforeCompile", "BeforeMake", "AfterMake"):
        for index in (1, 2):
            check(target.findtext(f".//{tag}/RunUserProg{index}") in (None, "0"), f"Keil {tag}.RunUserProg{index} 必须关闭")
    for tag in ("RunDebugAfterBuild", "UpdateFlashBeforeDebugging"):
        check(target.findtext(f".//{tag}") == "0", f"Keil {tag} 必须关闭")
    refs = []
    for node in target.findall(".//FilePath"):
        value = (node.text or "").strip().replace("\\", "/")
        path = (project.parent / value).resolve()
        check(bool(value) and path.is_file(), f"Keil 文件引用不存在：{value}")
        refs.append(path)
    check(len(refs) == len(set(refs)), "Keil 存在重复文件引用")
    sources = {path.resolve() for path in (FW / "User").rglob("*.c")}
    missing = sorted(str(path.relative_to(FW.resolve())) for path in sources - set(refs))
    check(not missing, f"Keil 未引用 User 源文件：{missing}")


def main():
    data = load_ioc()
    verify_ioc(data)
    verify_sources()
    verify_owners()
    verify_keil()
    print(json.dumps({"result": "FAIL" if ERRORS else "PASS", "checks": CHECKS, "errors": ERRORS},
                     ensure_ascii=False, indent=2))
    return 1 if ERRORS else 0


if __name__ == "__main__":
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    raise SystemExit(main())
