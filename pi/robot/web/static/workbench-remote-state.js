import {firmwareSince} from './workbench-firmware.js?v=3';
import {canAct, targetError} from './workbench-mech-guide.js?v=3';
import {faultText} from './workbench-mech-status.js?v=15';

/* 实车机械页纯显示规则；不保存令牌、零点或模型状态。 */
export const actuators = [
    {id:'lift', name:'升降', pin:'CAN · ID5', unit:'mm', min:0, max:49, initial:0},
    {id:'reach', name:'伸缩', pin:'CAN · ID6', unit:'mm', min:0, max:50, initial:0},
    {id:'tray', name:'料盘', pin:'PE9', unit:'°', min:0, max:270, initial:0},
    {id:'arm', name:'转臂', pin:'PE11', unit:'°', min:83, max:180, initial:180},
    {id:'gripper', name:'夹爪', pin:'PE13', unit:'°', min:0, max:98, initial:98}
];
export const manualTuning = state => firmwareSince(state.firmware, 31) && state.manual_debug === true;
export const armMaximum = state => firmwareSince(state.firmware, 32) && state.manual_debug ? 359 : manualTuning(state) ? 270 : 180;
export const gripHold = 98;
export const gripMaximum = state => firmwareSince(state.firmware, 38) ? 100 : 98;
export function freshDevice(state, receivedAt, now, transportMs=0) {
    const age = state.state_age_ms;
    return state.connected === true && state.domain === 'physical' &&
        Number.isFinite(age) && age >= 0 && Number.isFinite(transportMs) && transportMs >= 0 &&
        now >= receivedAt && age + transportMs + now - receivedAt < 250;
}
export function blockedReasons(state, owned, pending) {
    const reasons = [], m = state.mechanism;
    if (!state.connected || !m) reasons.push(state.reason || '设备反馈未就绪');
    if (state.base?.fault || m?.fault) reasons.push(`故障锁存：底盘 ${state.base?.fault || 0} / 机构 ${m?.fault?faultText(m.fault):0}`);
    if (state.action_locked) reasons.push(state.reason || '动作已锁定，只读反馈保留');
    if (state.site_ready === false) reasons.push('设备核心保持只读，现场动作许可未开放');
    if (!owned) reasons.push('尚未取得实车控制权');
    if (pending) reasons.push('上一请求结果待核对，请勿重复发送');
    if (m && !m.attached) reasons.push('机构反馈尚未接入');
    if (m?.attached && m.config_mask !== 3) reasons.push('两轴驱动配置尚未核对完整');
    if (m?.attached && !m.zero_valid) reasons.push('零点无效：需在最高 / 最近端人工建零');
    if ([1,2,3].includes(state.home?.state)) reasons.push('舵机回零初始化中；实际到0度后确认，或停止取消');
    if (firmwareSince(state.firmware, 31) && m?.zero_valid && !state.manual_debug)
        reasons.push('先进入手动调试，再执行单轴或舵机；完成后可继续下一角度');
    return reasons;
}
export function allowed(state, owned, pending, action) {
    if(state.arrival && !['IDLE','DONE','FAILED'].includes(state.arrival.phase))return false;
    if (action === 'stop') return true;
    if (['CAPTURE','REQUEST','REBOOT','VERIFY','ATTACH'].includes(state.recovery?.status)) return false;
    if ([1,2,3].includes(state.home?.state) && action!=='home_confirm') return false;
    if (action === 'prepare') {
        const m=state.mechanism;
        return !owned && !pending && state.connected === true && state.site_ready === true &&
            firmwareSince(state.firmware, 26) && state.action_locked === true &&
            !state.latched && state.stop_acknowledged === true && state.base?.stopped === true &&
            state.base?.fault === 0 && m?.fault === 6 && m.attached === 1 &&
            m.config_mask === 3 && m.base_stopped === 1 && !m.zero_valid && m.pwm_mask === 0;
    }
    if (action === 'claim') return state.connected === true && !pending && !state.action_locked;
    if (action === 'load') return true;
    if (state.action_locked) return false;
    if (action==='home' || action==='home_confirm') return firmwareSince(state.firmware, 34) &&
        state.manual_debug===true && !!state.home &&
        (action!=='home_confirm'||state.home.state===3) && canAct(state,owned,pending,'move');
    if (state.sequence?.status==='RUNNING' || state.group?.state===1) return false;
    if (action === 'program_save') return owned && state.connected===true && !pending;
    if (action === 'program_run') return firmwareSince(state.firmware, 32) && state.manual_debug===true &&
        !!state.group && canAct(state,owned,pending,'move');
    if (action === 'attach' || action === 'teach') return owned && state.connected === true && !pending &&
        (action !== 'attach' || (!state.base?.fault && !state.mechanism?.fault && state.base?.stopped && !state.mechanism?.attached));
    if (action === 'mode' && !firmwareSince(state.firmware, 27)) return false;
    if (action === 'move' && firmwareSince(state.firmware, 31) && !state.manual_debug) return false;
    return canAct(state, owned, pending, action);
}
export function canRecover(state, owned, pending) {
    const m=state.mechanism, busy=['CAPTURE','REQUEST','REBOOT','VERIFY','ATTACH'].includes(state.recovery?.status);
    return state.domain==='physical' && firmwareSince(state.firmware, 33) && state.connected===true && state.site_ready===true &&
        !owned && !pending && !busy && !state.latched && state.action_locked===true &&
        state.stop_acknowledged===true && state.base?.stopped===true && m?.attached===1 && m.config_mask===3;
}
export function moveArgs(id, raw, rpm=5, release=false, options={}) {
    if (raw === null || raw === undefined || String(raw).trim() === '') throw Error('请填写明确目标');
    const value = Number(raw), linear = id === 'lift' || id === 'reach';
    const reason = options.manualTuning && linear
        ? (!Number.isFinite(value)||value<0||Math.round(value*1000)>2147483647?'目标超出非负整数协议范围':'')
        : options.manualTuning && id==='arm'
        ? (!Number.isFinite(value)||value<0||value>(options.armMax??270)?`转臂手动命令角必须为0～${options.armMax??270}度`:'')
        : id==='gripper'
        ? (!Number.isFinite(value)||value<0||value>(options.gripMax===100?100:98)?`夹爪命令角必须为0～${options.gripMax===100?100:98}度`:'')
        : targetError(id, value);
    if (reason) throw Error(reason);
    const limit=options.manualTuning?3000:30;
    const accel=options.accel??1;
    if(linear && (!Number.isInteger(accel)||accel<0||accel>255))throw Error('轴加速度档位必须为0～255整数');
    if (linear && (!Number.isInteger(rpm) || rpm < 1 || rpm > limit)) throw Error(`轴速度必须为1～${limit}RPM的整数`);
    const speed = options.speedDegS ?? 45;
    if (!linear && options.manualTuning && (!Number.isFinite(speed)||speed<1||speed>360))
        throw Error('舵机指令速度必须为1～360度/秒');
    return {axis:id, [linear ? 'target_m' : 'target_rad']:value * (linear ? .001 : Math.PI/180),
        ...(linear ? {rpm,...(accel===1?{}:{accel})} : options.manualTuning ? {speed_rad_s:speed*Math.PI/180} : {}),
        release_confirmed:release === true || (id==='gripper' && value<gripHold)};
}
