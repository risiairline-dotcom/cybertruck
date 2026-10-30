// 界面只经网关调用独立核心；输入草稿、离线执行、实车状态分别表达。
import {drawMap, drawSpeed} from './workbench-map.js?v=3';
import {initLidar, renderLidar} from './workbench-lidar.js?v=6';
import {modelView} from './workbench-execution.js?v=1';
import {initScene, sceneValues, applyScene, overlayDraft} from './workbench-scene.js?v=3';
import {motionDirection, matchesRelease, stopShortcut} from './workbench-input.js?v=2';
import {initMechanism, renderMechanism, cancelMechHeld, stopMechHeld, stopMechanism} from './workbench-mechanism.js?v=48.3';

const $ = id => document.getElementById(id);
const text = (id, value) => { $(id).textContent = value; };
const uid = () => typeof crypto.randomUUID === 'function' ? crypto.randomUUID()
    : [...crypto.getRandomValues(new Uint8Array(16))].map(value => value.toString(16).padStart(2, '0')).join('');
const client = uid(); // 每个标签页独立身份，刷新不自动续跑。
const names = {start_1:'启停区1', start_2:'启停区2', qr:'扫码', raw:'原料', rough:'粗加工', staging:'暂存', north:'north预停点'};
const modes = {IDLE:'待命', MANUAL:'按住点动', RUNNING:'闭环执行中', STOPPING:'正在停车', STOPPED:'已停止', COMPLETE:'动作完成', FAILED:'模型故障'};
const presets = {batch:['qr','raw','rough','staging','raw','rough','staging','start_1'],
    tour:['qr','raw','rough','staging','start_1'], raw:['qr','raw']};
let token = '', tokenSince = 0, sequence = 0, generation = 0, queue = Promise.resolve();
let current = null, received = 0, planId = '', dirty = true, lastJob = '', lastEvents = '';
let held = null, heldButton = null, manualBusy = false;
let targets = ['qr', 'raw', 'rough', 'staging', 'start_1'];
let hydrated = false, localEdited = false, autoTimer = null, autoPending = false;
let submitted = '', versionsKey = '';
let preferredVersion = '';

function moving() { return current?.program?.busy || current?.execution_check?.status === 'RUNNING' || ['RUNNING','MANUAL','STOPPING'].includes(current?.simulation?.mode); }
function canonical(value) {
    if (Array.isArray(value)) return value.map(canonical);
    if (value && typeof value === 'object') return Object.fromEntries(Object.keys(value).sort().map(key => [key, canonical(value[key])]));
    return value;
}
const fingerprint = value => JSON.stringify(canonical(value));

function readSettings() {
    return {source:$('source').value, start:$('start').value,
        targets:$('source').value === 'custom' ? [...targets] : presets[$('source').value].map(name => name === 'start_1' ? $('start').value : name),
        blocked:[...document.querySelectorAll('#roads input:checked')].map(input => input.value),
        speed_m_s:number('route-speed', .03, 1.), ...sceneValues()};
}

function applySettings(request) {
    $('source').value = request.source;
    $('start').value = request.start;
    $('route-speed').value = request.speed_m_s;
    $('speed-range').value = request.speed_m_s;
    targets = [...request.targets];
    for (const input of document.querySelectorAll('#roads input')) input.checked = request.blocked.includes(input.value);
    applyScene(request);
    sequenceView();
    hydrated = true;
    localEdited = false;
    submitted = fingerprint(request);
}

async function submitPlan() {
    if (!token || !online() || moving()) { autoPending = true; return; }
    try {
        const args = readSettings();
        submitted = fingerprint(args);
        localEdited = false;
        autoPending = false;
        const result = await command('plan', args);
        if (result.draft) applySettings(result.draft);
        dirty = true;
        notify('按当前障碍物、停靠点和限速后台规划；不会自动启动模型。');
        updateButtons();
    } catch (error) { autoPending = false; notify(error.message, true); }
}

function notify(message, error = false) {
    text('notice', message);
    $('notice').classList.toggle('error', error);
}
function online() { return current?.available && performance.now() - received < 700; }
async function post(op, args = {}, lease = token) {
    const response = await fetch('/api/workbench/command', {
        method:'POST', headers:{'Content-Type':'application/json', 'X-Workbench':'1'},
        body:JSON.stringify({op, args, client, token:lease, sequence:++sequence, request_id:uid()}),
        signal:AbortSignal.timeout(1200), keepalive:op === 'stop'
    });
    const result = await response.json();
    if (!response.ok) throw new Error(typeof result.detail === 'string' ? result.detail : '参数无效，请检查输入范围');
    return result;
}
function command(op, args = {}) {
    const epoch = generation;
    const next = queue.then(() => {
        if (epoch !== generation) throw new Error('停止前的待发命令已撤销');
        return post(op, args);
    });
    queue = next.catch(() => {});
    return next;
}
initLidar({command, settings:readSettings, notify,
    acceptDraft:result => { applySettings(result.draft); dirty = true; autoPending = false; }});
function clearHeld() {
    held = null;
    heldButton?.classList.remove('pressed');
    heldButton = null;
}
async function stop(reason = '已请求停止模拟') {
    generation++;
    clearHeld();
    cancelMechHeld();
    token = '';
    updateButtons();
    try { await post('stop', {}, ''); notify(reason + '；控制权已释放。'); }
    catch (error) { notify('网关未确认停止；离线核心点动租约会独立过期。' + error.message, true); }
}
function stopVisible(source) {
    if (document.body.dataset.page !== 'drive' && document.body.dataset.page !== undefined) return stopMechanism(source);
    return stop();
}
async function stopHeld(reason) {
    generation++;
    clearHeld();
    if (!token) return;
    try { await post('manual_stop'); notify(`${reason}；等待模型停稳，可继续点动。`); }
    catch (error) { notify(`点动停车未获确认，核心350ms续约保护仍生效。${error.message}`, true); }
}
function number(id, low, high) {
    const value = $(id).valueAsNumber;
    if (!Number.isFinite(value) || value < low || value > high) throw new Error(`${$(id).labels?.[0]?.textContent.trim() || id} 超出范围`);
    return value;
}
async function pulse() {
    if (!held || manualBusy) return;
    if (!token || !online()) { clearHeld(); return; }
    manualBusy = true;
    try {
        const speed = number('jog-speed', .01, .15);
        const yaw = number('jog-yaw', 1, 28) * Math.PI / 180;
        const [x, y, w] = held;
        const norm = Math.max(1, Math.hypot(x, y));
        await command('manual', {vx_m_s:x * speed / norm, vy_m_s:y * speed / norm, wz_rad_s:w * yaw});
    } catch (error) {
        clearHeld(); await stop('点动请求异常，停止模拟'); notify(error.message, true);
    } finally { manualBusy = false; }
}
function hold(button) {
    if (!token || !online() || button.disabled || held) return;
    held = button.dataset.dir.split(',').map(Number);
    heldButton = button;
    button.classList.add('pressed');
    pulse();
}
for (const button of document.querySelectorAll('.jog')) {
    button.addEventListener('pointerdown', event => {
        if (event.button !== 0) return;
        event.preventDefault(); button.setPointerCapture(event.pointerId); hold(button);
    });
    for (const name of ['pointerup', 'pointercancel', 'lostpointercapture']) button.addEventListener(name, () => {
        if (heldButton === button) stopHeld('方向控制已松开');
    });
    button.addEventListener('contextmenu', event => event.preventDefault());
}
window.addEventListener('keydown', event => {
    if (stopShortcut(event)) {
        event.preventDefault(); stopVisible(`keyboard-${event.code}`); return;
    }
    const direction = motionDirection(event);
    if (!direction || $('page-drive').hidden) return;
    event.preventDefault();
    hold(document.querySelector(`[data-dir="${direction}"]`));
});
window.addEventListener('keyup', event => {
    if (matchesRelease(event, heldButton?.dataset.dir)) stopHeld('键盘已松开');
});
window.addEventListener('blur', () => { if (held) stopHeld('窗口失焦'); });
document.addEventListener('visibilitychange', () => { if (document.hidden && held) stopHeld('页面已隐藏'); });
window.addEventListener('pagehide', () => { if (held) stop('页面关闭'); });
window.addEventListener('offline', () => { if (held) stop('浏览器断网'); });
$('stop').onclick = () => stopVisible('top-stop-button');
$('global-real-stop').onclick = () => stopMechanism('global-stop-button');
$('events-filter').oninput = () => {
    const query = $('events-filter').value.trim().toLowerCase();
    for (const row of $('events').children) row.hidden = !row.textContent.toLowerCase().includes(query);
};
$('events-export').onclick = () => {
    const value = {exported_at:new Date().toISOString(), domain:'simulation_history', events:current?.events || []};
    const url = URL.createObjectURL(new Blob([JSON.stringify(value,null,2)],{type:'application/json'}));
    const link = document.createElement('a');link.href=url;link.download=`gc-model-events-${Date.now()}.json`;
    link.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
};
$('pad-stop').onclick = () => stop();
$('claim').onclick = async () => {
    const epoch = generation;
    try {
        const result = await command('claim');
        if (epoch !== generation) return;
        token = result.token;
        tokenSince = performance.now();
        notify('已取得离线控制权。按住方向键运动，松开即请求停车。');
        updateButtons();
        if (autoPending) submitPlan();
    } catch (error) { notify(error.message, true); }
};
$('release').onclick = async () => {
    if (held) { await stop(); return; }
    try { await command('release'); token = ''; updateButtons(); }
    catch (error) { notify(error.message, true); }
};
function updateButtons() {
    const enabled = !!token && online();
    const mode = current?.simulation?.mode;
    const checkBusy = current?.execution_check?.status === 'RUNNING';
    const mechBusy = checkBusy || !!current?.program?.busy || !!current?.mechanism?.busy || ['RUNNING','WAIT_OBSERVATION'].includes(current?.transfer?.mode);
    const mechReady = !mechBusy && (!current?.mechanism?.zero_version || current?.mechanism?.transport_ready);
    const idle = current?.simulation?.stopped && mode !== 'FAILED' && mechReady;
    for (const button of document.querySelectorAll('.controlled')) button.disabled = !enabled;
    for (const button of document.querySelectorAll('.jog')) button.disabled = !enabled || (!idle && mode !== 'MANUAL');
    $('move').disabled = !enabled || !idle;
    $('reset').disabled = !enabled || !current?.simulation?.stopped || mechBusy;
    $('plan').disabled = !enabled || moving() || mechBusy;
    $('run').disabled = !enabled || !idle || dirty || !planId;
    $('save').disabled = !enabled || dirty || !planId || mechBusy;
    $('verify-execution').disabled = !enabled || !idle || dirty || !planId || !current?.execution_check?.available;
    $('cancel-execution').disabled = !enabled || !checkBusy;
    $('claim').disabled = !online() || (!!current?.owner?.client && current.owner.client !== client);
    $('claim').hidden = !!token;
    $('release').hidden = !token;
    $('defaults').disabled = !enabled || !current?.simulation?.stopped || mechBusy;
    $('load-version').disabled = !enabled || !current?.simulation?.stopped || mechBusy || !$('saved-version').value;
    for (const item of document.querySelectorAll('.scene-input, #start, #source, #route-speed, #speed-range, #add-target')) item.disabled = moving();
    renderLidar(current || {}, enabled, moving() || mechBusy);
}
function draftChanged() {
    dirty = true;
    localEdited = true;
    autoPending = true;
    clearTimeout(autoTimer);
    if ($('auto-plan').checked) autoTimer = setTimeout(submitPlan, 650);
    text('draft-state', token ? '草稿已修改，等待自动重规划；不会自动执行。' : '草稿已修改；获取控制权后可自动规划。');
    updateButtons();
}
function sequenceView() {
    const source = $('source').value;
    $('custom-editor').hidden = source !== 'custom';
    const stops = [$('start').value, ...(source === 'custom' ? targets : presets[source].map(name => name === 'start_1' ? $('start').value : name))];
    $('route-sequence').replaceChildren();
    stops.forEach((name, index) => {
        if (index) { const arrow = document.createElement('b'); arrow.textContent = '→'; $('route-sequence').append(arrow); }
        const item = document.createElement('span'); item.textContent = names[name] || name; $('route-sequence').append(item);
    });
    $('targets').replaceChildren();
    targets.forEach((name, index) => {
        const row = document.createElement('li'); row.textContent = names[name] || name;
        for (const [label, fn] of [['↑', () => { if (index) [targets[index - 1], targets[index]] = [targets[index], targets[index - 1]]; }], ['×', () => targets.splice(index, 1)]]) {
            const button = document.createElement('button'); button.textContent = label;
            button.setAttribute('aria-label', `${label === '↑' ? '上移' : '删除'}${names[name] || name}`);
            button.onclick = () => { fn(); sequenceView(); draftChanged(); }; row.append(button);
        }
        $('targets').append(row);
    });
}
for (const [id, label] of Object.entries(names)) {
    const option = document.createElement('option'); option.value = id; option.textContent = label; $('target').append(option);
}
$('add-target').onclick = () => {
    if (targets.length >= 9) { notify('每次最多选择9个站点', true); return; }
    targets.push($('target').value); sequenceView(); draftChanged();
};
for (const id of ['source', 'start']) $(id).addEventListener('change', () => {
    if (id === 'start') {
        targets = targets.map(name => name === 'start_1' || name === 'start_2' ? $('start').value : name);
    }
    sequenceView(); draftChanged();
});
for (const [from, to] of [['route-speed', 'speed-range'], ['speed-range', 'route-speed']]) {
    $(from).addEventListener('input', () => { $(to).value = $(from).value; draftChanged(); });
}
$('plan').onclick = submitPlan;
$('verify-execution').onclick = async () => {
    try { await command('verify', {plan_id:planId}); notify('开始正式执行链离线联调，PC0由合成夹具产生，不连接实车。'); }
    catch (error) { notify(error.message, true); }
};
$('cancel-execution').onclick = async () => {
    try { await command('verify_cancel'); notify('已请求取消，等待模型STOP结果。'); }
    catch (error) { notify(error.message, true); }
};
$('auto-plan').onchange = () => { if ($('auto-plan').checked && dirty) submitPlan(); };
$('saved-version').onchange = updateButtons;
for (const [id, op] of [['defaults','defaults'], ['load-version','load']]) $(id).onclick = async () => {
    clearTimeout(autoTimer);
    autoPending = false;
    try {
        const result = await command(op, op === 'load' ? {plan_id:$('saved-version').value} : {});
        applySettings(result.draft);
        lastJob = '';
        dirty = op === 'defaults';
        if (op === 'defaults') {
            planId = '';
            $('auto-plan').checked = true;
            $('jog-speed').value = .05;
            $('jog-yaw').value = 10;
            $('move-x').value = 100;
            $('move-y').value = 0;
            $('move-yaw').value = 0;
        }
        notify(op === 'defaults' ? '已恢复初始设置并清空障碍；历史保存版本保留，未运行。' : '已读取保存的路径和参数，仅恢复模型待命。');
    } catch (error) { notify(error.message, true); }
};
for (const op of ['run', 'save']) $(op).onclick = async () => {
    try {
        await command(op, {plan_id:planId});
        if (op === 'save') {
            preferredVersion = planId;
            versionsKey = '';
        }
        notify(op === 'run' ? '离线闭环已开始，速度和路线版本已冻结。' : '离线版本已保存到 output/workbench/plans。');
    }
    catch (error) { notify(error.message, true); }
};
$('move').onclick = async () => {
    try {
        await command('move', {x_m:number('move-x', -500, 500) / 1000,
            y_m:number('move-y', -500, 500) / 1000,
            yaw_rad:number('move-yaw', -180, 180) * Math.PI / 180,
            speed_m_s:number('jog-speed', .01, .15)});
        notify('已开始定量离线动作；完成以闭环到位和稳定停车为准。');
    } catch (error) { notify(error.message, true); }
};
$('reset').onclick = async () => {
    try { await command('reset', {start:$('start').value}); notify('离线场景已重置；这不代表实车配准或软件建零。'); }
    catch (error) { notify(error.message, true); }
};
for (const button of document.querySelectorAll('[data-page]')) button.onclick = () => {
    if (held) stopHeld('切换页面');
    if (document.body.dataset.page === 'mechanism' && button.dataset.page !== 'mechanism') stopMechHeld();
    document.querySelectorAll('.page').forEach(page => { page.hidden = page.id !== `page-${button.dataset.page}`; });
    document.querySelectorAll('[data-page]').forEach(item => item.classList.toggle('active', item === button));
    document.querySelector('h1').textContent = {drive:'底盘与导航', mechanism:'机械远程控制', vision:'相机与标定', logs:'运行记录'}[button.dataset.page];
    document.body.dataset.page = button.dataset.page;
    $('global-real-stop').hidden = !['vision','logs'].includes(button.dataset.page);
    $('stop').innerHTML = button.dataset.page === 'mechanism' ? '■ 停止实车 <kbd>Space</kbd>' : '■ 停止模拟 <kbd>Space</kbd>';
    history.replaceState(null, '', `#${button.dataset.page}`);
};
function renderPlan(job) {
    const key = `${job.id}:${job.status}:${job.progress?.completed}`;
    if (key === lastJob) return;
    lastJob = key;
    text('plan-status', {IDLE:'尚未规划', PLANNING:'后台计算中', READY:'离线版本已就绪', FAILED:'规划失败'}[job.status] || job.status);
    if (job.status === 'PLANNING' && job.progress) text('plan-status', `${job.progress.completed}/${job.progress.total} 工位 · ${job.progress.current}`);
    if (job.status === 'FAILED') { notify(job.reason, true); planId = ''; dirty = true; text('limit-reason', job.reason); }
    if (job.status === 'IDLE' || job.status === 'PLANNING') {
        text('nav-budget', '导航预算待本次规划完成后计算。');
        planId = '';
        dirty = true;
        text('plan-time', '— s'); text('plan-count', '—'); text('plan-hash', '未生成');
        text('map-status', job.status === 'PLANNING' ? '按场景重规划中' : '等待规划');
        if (job.status === 'IDLE') {
            text('draft-state', '初始设置已恢复；规划后才能运行。');
            text('limit-reason', '请求上限可设到1m/s，实际轨迹同时受轮速、弯道、距离和加减速限制。');
        }
    }
    if (job.status !== 'READY') return;
    const plan = job.plan;
    planId = plan.id;
    // 其他浏览器生成的路线也能查看；草稿仅在与本页输入一致时标记已应用。
    const params = plan.parameters;
    let same = false;
    try { same = fingerprint(params) === fingerprint(readSettings()); } catch { /* 无效输入仍是草稿。 */ }
    dirty = !same;
    text('plan-time', `${plan.seconds.toFixed(2)} s`); text('plan-count', plan.legs.length); text('plan-hash', plan.sha256.slice(0, 10));
    text('map-status', `${plan.legs.length} 段 · ${params.speed_m_s.toFixed(2)} m/s`);
    text('draft-state', same ? '当前草稿已生成独立离线版本；实车参数保持原配置。' : '显示核心最新版本；与当前草稿不同，运行前请重新规划。');
    if (same && $('notice').textContent.includes('后台规划')) notify('当前场景规划已完成；点击运行才启动离线模型。');
    const limits = plan.limits;
    const referenceSeconds = plan.legs.reduce((total, leg) => total + leg.seconds, 0);
    const reserveSeconds = plan.seconds - referenceSeconds;
    text('nav-budget', `两批导航目标60秒；当前${plan.seconds.toFixed(2)}秒估计 = 参考运动${referenceSeconds.toFixed(2)}秒 + 分段停稳预留${reserveSeconds.toFixed(2)}秒。共${plan.legs.length}段；预留不是实测停车时间。${params.source === 'batch' ? `距预算还差${Math.max(0, plan.seconds-60).toFixed(2)}秒。` : '本路线仅为局部/巡回对照，不代表两批预算验收。'}`);
    text('limit-reason', `请求上限 ${limits.requested_m_s.toFixed(2)} m/s；参考采样峰值 ${limits.sampled_peak_m_s.toFixed(3)} m/s；保守轮速界 ${limits.wheel_bound_rpm.toFixed(1)} / ${limits.max_rpm.toFixed(0)} RPM。${limits.reasons.join('；')}。按预计时间在道路朝向候选中择优。`);
    $('leg-list').replaceChildren();
    for (const [index, leg] of plan.legs.entries()) {
        const row = document.createElement('div');
        const name = document.createElement('span'); name.textContent = `${index + 1}. ${leg.name}`;
        const duration = document.createElement('span'); duration.textContent = `${leg.seconds.toFixed(2)} s`;
        row.append(name, duration); $('leg-list').append(row);
    }
    drawSpeed(plan);
}
function render(state) {
    const previousMode = current?.simulation?.mode;
    current = state; received = performance.now();
    text('connection', state.available ? '离线核心在线' : '离线核心未连接');
    $('connection').className = `badge ${state.available ? 'good' : 'muted'}`;
    if (!state.available) { invalidate(); return; }
    if ($('notice').textContent.trim().startsWith('正在连接离线核心')) {
        notify('离线核心已连接；原控制区只作用于模型。实车状态与命令见独立实车区。');
    }
    if (token && state.owner.client !== client && performance.now() - tokenSince > 300) { token = ''; clearHeld(); }
    const sim = modelView(state);
    const check = state.execution_check;
    text('execution-check-state', check?.visible ? `${check.status} · ${check.completed ?? check.segment ?? 0}/${check.total ?? '—'}段 · ${(check.elapsed_s || 0).toFixed(2)}s · ${check.stop_confirmed ? '模型STOP已确认' : '尚无最终STOP确认'}${check.reason ? ' · '+check.reason : ''}` : '当前路线尚未联调；历史完成记录不代表本路线已执行');
    text('mode', modes[sim.mode] || sim.mode);
    text('stop-state', sim.stopped ? '模型轮速已稳定' : '模型运动 / 等待停稳');
    text('pose', `${sim.pose[0].toFixed(3)} / ${sim.pose[1].toFixed(3)}`);
    text('yaw', `航向 ${(Math.atan2(Math.sin(sim.pose[2]), Math.cos(sim.pose[2])) * 180 / Math.PI).toFixed(1)}° · 模拟`);
    text('progress', `${sim.completed} / ${sim.total || '—'}`);
    text('elapsed', `模型运行 ${sim.elapsed_s.toFixed(2)} s`);
    text('rpm', sim.rpm.length ? Math.max(...sim.rpm.map(Math.abs)).toFixed(1) : '未知');
    text('session', sim.session.slice(0, 8)); text('error-xy', Number.isFinite(sim.position_error_m) ? `${(sim.position_error_m * 1000).toFixed(2)} mm` : '未知');
    text('error-yaw', Number.isFinite(sim.yaw_error_rad) ? `${(sim.yaw_error_rad * 180 / Math.PI).toFixed(2)}°` : '未知');
    text('fault', sim.mode === 'FAILED' ? sim.reason : sim.fault ? `模型故障 ${sim.fault}` : '0 / 无'); text('odom', `${sim.odom_origin_ms} ms · 连续`);
    text('owner', state.owner.client === client ? '本页持有控制权' : state.owner.client ? '其他浏览器正在控制' : '当前无人控制');
    if (!$('roads').children.length) {
        for (const [road, a, b] of state.field.edges) {
            const label = document.createElement('label'); const input = document.createElement('input');
            input.type = 'checkbox'; input.value = road; input.onchange = draftChanged;
            label.append(input, document.createTextNode(`${names[a] || a} ↔ ${names[b] || b}`)); $('roads').append(label);
        }
    }
    if (!hydrated) applySettings(state.draft);
    else if (!token && !localEdited && fingerprint(state.draft) !== submitted) applySettings(state.draft);
    const key = JSON.stringify(state.saved_versions);
    if (key !== versionsKey) {
        versionsKey = key;
        const old = $('saved-version').value;
        $('saved-version').replaceChildren();
        for (const saved of state.saved_versions) {
            const option = document.createElement('option');
            option.value = saved.id;
            option.disabled = saved.schema !== 2;
            option.textContent = `${saved.schema === 2 ? '' : '旧格式 / '}${{batch:'两批',tour:'巡回',raw:'原料',custom:'自选'}[saved.source]} · ${saved.speed_m_s.toFixed(2)}m/s · ${saved.id.slice(0,8)}`;
            $('saved-version').append(option);
        }
        if (!state.saved_versions.some(item => item.schema === 2)) {
            const option = document.createElement('option'); option.value = ''; option.textContent = '尚无当前格式保存版本'; $('saved-version').append(option); $('saved-version').value = '';
        } else if (preferredVersion && state.saved_versions.some(item => item.id === preferredVersion && item.schema === 2)) {
            $('saved-version').value = preferredVersion;
            preferredVersion = '';
        } else if (state.saved_versions.some(item => item.id === old && item.schema === 2)) $('saved-version').value = old;
    }
    renderPlan(state.planning); drawMap(state); overlayDraft(); updateButtons();
    renderMechanism(state, !!token && online() && check?.status !== 'RUNNING');
    const visiblePlan = sim.mode === 'RUNNING' && state.active_plan ? state.active_plan : state.planning.plan;
    if (visiblePlan) {
        const executed = visiblePlan.id === sim.active_plan_id;
        text('map-status', `${visiblePlan.legs.length} 段 · ${visiblePlan.parameters.speed_m_s.toFixed(2)} m/s · ${executed ? sim.mode === 'COMPLETE' ? '模型已完成' : '执行版本' : '待运行预览'}`);
    }
    if (previousMode === 'RUNNING' && sim.mode === 'COMPLETE') notify(`离线模型${sim.completed}段已完成，用时${sim.elapsed_s.toFixed(2)}s，已确认模型停稳。`);
    if (previousMode !== 'FAILED' && sim.mode === 'FAILED') notify(sim.reason, true);
    const eventKey = JSON.stringify(state.events);
    if (eventKey !== lastEvents) {
        lastEvents = eventKey; $('events').replaceChildren();
        for (const event of state.events) {
            const row = document.createElement('div'); const time = document.createElement('time');
            const name = document.createElement('b'); const detail = document.createElement('p');
            time.textContent = new Date(event.at * 1000).toLocaleString('zh-CN', {hour12:false});
            name.textContent = event.name; detail.textContent = event.detail;
            row.append(time, name, detail); $('events').append(row);
        }
        $('events-filter').oninput();
    }
}
function invalidate() {
    if (held) stop('状态流中断');
    token = ''; current = null;
    for (const [id, value] of Object.entries({mode:'状态已过期', pose:'— / —', yaw:'航向未知', rpm:'—', 'stop-state':'无法确认模型停车', 'error-xy':'—', 'error-yaw':'—', fault:'未知', odom:'—'})) text(id, value);
    drawMap({available:false}); updateButtons();
    renderMechanism(null, false);
}
function connect() {
    const socket = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/api/workbench/stream`);
    socket.onmessage = event => { try { render(JSON.parse(event.data)); } catch { notify('状态数据异常，等待新帧', true); invalidate(); } };
    socket.onclose = () => { invalidate(); text('connection', '连接中断'); setTimeout(connect, 1500); };
    socket.onerror = () => socket.close();
}
setInterval(pulse, 100);
setInterval(() => {
    if (received && performance.now() - received > 700) invalidate();
    if (token && online() && !held && !document.hidden) command('renew').catch(error => { token = ''; updateButtons(); notify(error.message, true); });
}, 500);
initScene({changed:draftChanged, notify, isRunning:moving});
initMechanism({command, notify});
const initialPage = location.hash.slice(1);
if (['drive','mechanism','vision','logs'].includes(initialPage)) document.querySelector(`[data-page="${initialPage}"]`).click();
sequenceView(); updateButtons(); connect();
