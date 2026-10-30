import {setText} from './workbench-dom.js?v=1';

/* 转臂+伸缩对中：底盘保持停车，先执行观察联调表；探测结果需人工核对窗口无碰撞后才导入。 */
export function initArmCentering(root, current, owned, busy, command) {
    const panel=document.createElement('article');panel.className='card';
    panel.innerHTML=`<h2>转臂+伸缩对中（2号工位）</h2>
      <p>底盘停车不动；先执行观察联调表并等待完成，再由伸缩和PE11转臂把2号圆环移到画面中心。升降不参与。</p>
      <label><input data-field="confirm" type="checkbox">刚执行完观察联调表，空爪，探测/修正范围内无干涉</label>
      <label>探测伸缩 / mm <input data-field="probe-reach" type="number" min="0.5" max="3" step="0.5" value="2"></label>
      <label>探测转臂 / 度 <input data-field="probe-arm" type="number" min="0.5" max="1.5" step="0.5" value="1"></label>
      <button data-field="probe">开始探测（8次小幅正反动作）</button>
      <button data-field="auto">开始自动对中</button>
      <p data-field="status"></p><p data-field="probe-result"></p>
      <fieldset><legend>导入探测结果</legend>
        <label>窗口伸缩 ±mm <input data-field="win-reach" type="number" min="1" max="10" step="0.5" value="6"></label>
        <label>窗口转臂 ±度 <input data-field="win-arm" type="number" min="0.5" max="5" step="0.5" value="3"></label>
        <label>单步伸缩 mm <input data-field="step-reach" type="number" min="0.5" max="3" step="0.5" value="3"></label>
        <label>单步转臂 度 <input data-field="step-arm" type="number" min="0.5" max="1.5" step="0.5" value="1.5"></label>
        <label>中心容差 px <input data-field="deadband" type="number" min="1" max="30" step="1" value="4"></label>
        <label><input data-field="window-checked" type="checkbox">已在实物上把伸缩和转臂摆到窗口两端检查，无碰撞</label>
        <button data-field="accept">保存为正式标定</button>
      </fieldset>
      <p data-field="calibration"></p>`;
    root.append(panel);const el=name=>panel.querySelector(`[data-field="${name}"]`);
    const number=name=>Number(el(name).value);
    const valid=(...names)=>names.every(n=>el(n).value.trim()&&el(n).checkValidity());
    el('probe').onclick=()=>{
        if(!el('confirm').checked||!valid('probe-reach','probe-arm'))return;
        command('arm_centering_start',{mode:'probe',confirmed:true,probe_reach_mm:number('probe-reach'),probe_arm_deg:number('probe-arm')});
        el('confirm').checked=false;
    };
    el('auto').onclick=()=>{
        if(!el('confirm').checked)return;
        command('arm_centering_start',{mode:'auto',confirmed:true});el('confirm').checked=false;
    };
    el('accept').onclick=()=>{
        if(!el('window-checked').checked||!valid('win-reach','win-arm','step-reach','step-arm','deadband'))return;
        command('arm_centering_accept',{confirmed:true,physical_window_checked:true,
            window:{reach_mm:number('win-reach'),arm_deg:number('win-arm')},
            max_step:{reach_mm:number('step-reach'),arm_deg:number('step-arm')},deadband_px:number('deadband')});
        el('window-checked').checked=false;
    };
    function draw(){
        const s=current(),a=s.arm_centering||{},status=a.status||'IDLE';
        const idle=!['MOVING','OBSERVING'].includes(status);
        const ready=s.connected&&owned()&&!busy()&&s.site_ready&&s.manual_debug&&!s.action_locked&&
            !s.mechanism?.fault&&!s.base?.fault&&s.base?.stopped&&s.sequence?.status==='DONE'&&idle&&
            !['PREPARE','COMMIT','CONFIG','KEY_PREPARE','KEY_REQUEST','WAIT_PC0','ARM','ARMING','READY','GOAL','MOVING','RETURN'].includes(s.arrival?.phase);
        el('probe').disabled=!ready;
        el('auto').disabled=!ready||!a.calibration_available;
        el('accept').disabled=!(s.connected&&owned()&&!busy()&&status==='DONE'&&a.probe);
        const o=a.offset||{};
        setText(el('status'),`${status} ${a.reason||''}；当前偏移 伸缩${o.reach_mm??0}mm / 转臂${o.arm_deg??0}°，${a.steps||0}步`+
            (s.sequence?.status!=='DONE'?'；需先执行观察联调表':''));
        const p=a.probe;
        setText(el('probe-result'),p?`响应矩阵 ${JSON.stringify(p.jacobian_px)} px/(mm,度)；残差 ${p.residual_px}px，回差 ${p.hysteresis_px}px，条件数 ${p.condition}。回差或残差大于容差时不要导入。`:'尚无探测结果');
        setText(el('calibration'),a.calibration_available?'已有正式标定；只适用于绑定的观察姿态及相机配置':(a.calibration_error||''));
    }
    panel.addEventListener('input',draw);return{draw};
}
