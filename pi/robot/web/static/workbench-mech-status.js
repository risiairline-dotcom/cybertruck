import {firmwareSince} from './workbench-firmware.js?v=1';
/* 只解释真实反馈；不解锁故障、不推定实物到位，也不发送请求。 */
const faults = {
    1:'人工零点未建立', 2:'机构反馈超过250毫秒', 3:'机构尚未稳定',
    4:'驱动器状态异常', 5:'驱动器配置或使能条件不符', 6:'停止请求已执行，需重新准备',
    7:'启动或会话已变化', 8:'位置超出当前模式范围', 9:'底盘未确认停车',
    10:'姿态碰撞互锁', 11:'零点版本变化', 12:'反馈或命令格式异常',
    13:'零点已建立', 14:'底层故障锁存',
    100:'动作编号冲突', 101:'上一轴动作尚未结束', 102:'轴命令发送失败',
    103:'轴命令确认异常或超时', 104:'轴未在期限内到位',
    105:'另一轴出现非预期速度或位移', 106:'运动方向、速度或位移不符',
    107:'驱动器目标与本次位置账本不符', 108:'轴动作已取消', 109:'动作期间零点变化'
};

export function faultText(code) {
    return `${faults[code] || '未识别的机构故障'}（${code}）`;
}

export function parkingText(record) {
    if (!record?.captured) return '';
    const reasons = [[1,'反馈无效'],[2,'反馈超250ms'],[4,'轮速超过1RPM'],
        [8,'驱动器异常'],[16,'停车未确认'],[32,'底层故障']];
    const wheel = record.wheels?.[record.wheel-1];
    return `首次停车条件失败 #${record.action_id}：${reasons.filter(([bit])=>record.mask&bit).map(([,name])=>name).join('、')}`+
        (wheel?`；ID${record.wheel}轮速${wheel.rpm}RPM，速度/位置/状态帧龄${wheel.speed_age_ms}/${wheel.position_age_ms}/${wheel.status_age_ms}ms`:'')+'。';
}

export function mechanismNotice(state, waiting='') {
    const m=state.mechanism;
    if (!state.connected || !m) return {kind:'muted',text:'等待新鲜反馈，当前不能发送动作。'};
    if (!state.base?.fault && m.fault===6) {
        const event=state.safety_events?.at(-1);
        const names={'lease-expired':'控制权续约超时','page-hidden':'页面进入后台','page-exit':'页面关闭',
            'page-leave':'离开机械页','browser-offline':'浏览器断网','panel-stop-button':'点击停止','program-failed':'联调中止'};
        return {kind:'muted',text:`停止请求已执行（6）。${event?`来源：${names[event.source]||event.source}；${event.reason}`:'停止来源尚未取得，不能只凭6判断原因。'} 按恢复说明重新准备，不自动续跑。`};
    }
    if (state.base?.fault || m.fault) {
        const reason=state.base?.fault?`底盘故障（${state.base.fault}）`:faultText(m.fault);
        const record=state.mechanism_failure;
        const frozen=record?.captured?` 首次触发${record.tick_ms}ms，${record.axis?`ID${record.axis+4}`:'公共条件'}；`+
            `升降/伸缩${record.position_um.map(n=>Math.abs(n)>1e12?'未知':(n/1000).toFixed(3)).join('/')}mm，`+
            `速度${record.rpm.join('/')}RPM，位置帧龄${record.age_ms.join('/')}ms。`:'';
        const detail=(m.fault===9?parkingText(state.park_failure):m.fault===105?'这是板端另一轴监测触发；仅凭目测不能区分线缆牵扯与反馈变化。':'')+frozen;
        return {kind:'fault',text:`${reason}。${detail}先停止并排除干涉；确认机构静止且有支撑后，按恢复说明人工操作。不能直接继续下一动作。`};
    }
    if (state.action_locked) return {kind:'muted',text:'本轮已停止；按“正常停止后重新准备”或恢复说明操作，不会自动续跑。'};
    if (waiting) return {kind:'busy',text:`${waiting}：等待板端应答，不要重复点击。`};
    if (state.sequence?.status==='RUNNING' || state.group?.state===1)
        return {kind:'busy',text:'联调组执行中，等待全部成员完成后交接；可随时停止。'};
    if ([1,2,3].includes(state.home?.state))
        return {kind:'busy',text:state.home.state===3?'回零命令已输出，等待你确认三路实物到0°。':'舵机回零准备中，请等待反馈。'};
    if (m.axis_state>=1 && m.axis_state<=5)
        return {kind:'busy',text:`轴动作 #${m.axis_id}：${m.axis_state===5?'正在确认停车':'执行中，等待反馈到位'}。完成后按钮自动恢复；应答接受不代表已到位。`};
    if (m.servo_state>=1 && m.servo_state<=5)
        return {kind:'busy',text:`舵机动作 #${m.servo_id}：${m.servo_state===5?'等待实物姿态确认':'执行中'}。`};
    if (m.pose_state>=1 && m.pose_state<=5)
        return {kind:'busy',text:`姿态 #${m.pose_id} 执行中，按当前步骤确认或放行。`};
    if (!m.zero_valid) return {kind:'muted',text:'尚未建零；接入反馈后，核对升降最高、伸缩最近端再建零。'};
    if (!state.manual_debug) return {kind:'muted',text:'准备已就绪，请进入手动调试，再执行单轴或舵机。'};
    return {kind:'ready',text:'上一操作已结束，可输入并执行下一目标；舵机命令完成不代表实物角度已测得。'+
        (firmwareSince(state.firmware, 36)?' 手动机械调试已取消车轮1RPM限制，通信和驱动器保护继续生效。':'')};
}
