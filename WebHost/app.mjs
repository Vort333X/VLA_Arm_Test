import { AXES, CMD, MSG, VERSION, checkAck, decodeFeedback, targetsPayload } from './protocol.mjs?v=axes-1';
import { ArmSerial } from './serial.mjs?v=axes-1';
import { ArmView, forward, readModel } from './arm-model.mjs?v=axes-1';
import { createPlanarIk, PITCH_JOINTS } from './planar-ik.mjs?v=axes-1';
import { PlanarView, TopView } from './planar-view.mjs?v=axes-1';

const POLL_MS = 200;
const STALE_MS = 1000;
const HISTORY_MS = 30_000;
const SERVO_CENTER = 135;
const ALL_PWM_CHANNELS = (1 << AXES.length) - 1;
const PITCH_AXES = PITCH_JOINTS.map(port => AXES.indexOf(port));
const YAW_AXIS = AXES.indexOf('J1');
const ROLL_AXIS = AXES.indexOf('J5');
const $ = id => document.getElementById(id);
$('protocol-label').textContent = `USART1 · 协议 V${VERSION}`;
const axisTable = $('axis-table');
const angleInput = $('target');
const modelAngle = $('model-angle');
const modelFollow = $('model-follow');
const reverseButton = $('model-reverse');
const targetInputs = ['x', 'y', 'z'].map(axis => $(`target-${axis}`));
const targetRoll = $('target-roll');
const planarPitch = $('planar-pitch');
const planarSend = $('planar-send');
const edits = AXES.map(() => '');
const rows = AXES.map((port, index) => {
    $('joint').append(new Option(port, String(index)));
    const row = axisTable.insertRow();
    const button = document.createElement('button');
    button.type = 'button';
    button.className = 'axis-button';
    button.textContent = port;
    button.addEventListener('click', () => selectJoint(index));
    row.insertCell().append(button);
    const values = Array.from({ length: 4 }, () => row.insertCell());
    return { row, button, values };
});

let selected = 0;
let link = null;
let phase = 'idle';
let operation = null;
let queued = null;
let latest = null;
let latestAt = 0;
let lastReplyTime = '';
let logFault = false;
let history = [];
let model = null;
let modelSpan = 0;
const modelTargets = AXES.map(() => SERVO_CENTER);
const modelDirections = AXES.map(() => 1);
let modelDraft = null;
let planarIk = null;
let planarSelection = null;
let sliceYaw = 0;
let targetPreviewKey;
const armView = new ArmView($('arm-view'), port => selectJoint(AXES.indexOf(port)));
const planarView = new PlanarView($('planar-view'), pickPlanarPoint);
const targetArmView = new ArmView($('target-arm-view'));
const targetTopView = new TopView($('target-top-view'), (x, y) => setSpatialPoint([x, y, targetInputs[2].valueAsNumber]));
const formatAngle = value => value.toFixed(2);
const clockText = () => new Date().toLocaleTimeString('zh-CN', { hour12: false });
const feedbackFresh = () => phase === 'ready' && !!latest && performance.now() - latestAt <= STALE_MS;

function log(message, level = 'info') {
    const list = $('log');
    list.querySelector('.hint')?.remove();
    const item = document.createElement('li');
    item.dataset.level = level;
    const time = document.createElement('time');
    time.textContent = clockText();
    item.append(time, document.createTextNode(message));
    list.prepend(item);
    if (list.children.length > 40) list.lastElementChild.remove();
}

function result(message, level = 'info') {
    $('control-result').textContent = message;
    $('control-result').dataset.level = level;
}

function updateControls() {
    const connect = $('connect');
    connect.disabled = !navigator.serial || phase === 'opening' || phase === 'closing';
    connect.textContent = phase === 'ready' ? '断开' : '连接串口';
    const busy = phase !== 'ready' || operation === 'command' || !!queued;
    for (const id of ['send', 'disable', 'disable-all']) $(id).disabled = busy;
    for (const id of ['enable', 'enable-all']) $(id).disabled = busy || !feedbackFresh();
    const modelAxis = model?.joints.some(joint => joint.port === AXES[selected]);
    const modelBusy = modelFollow.checked && (busy || !feedbackFresh());
    modelAngle.disabled = !modelAxis || modelBusy;
    reverseButton.disabled = modelAngle.disabled || operation === 'command' || !!queued;
    $('model-home').disabled = !model || modelBusy;
    modelFollow.disabled = !model || operation === 'command' || !!queued || (!modelFollow.checked && !feedbackFresh());
    renderPlanar(busy);
}

function setPhase(next, text) {
    phase = next;
    $('link-state').dataset.state = next;
    $('link-state').textContent = text;
    updateControls();
}

function selectJoint(index) {
    if (index < 0) return;
    modelDraft = null;
    selected = index;
    $('joint').value = String(index);
    angleInput.value = edits[index];
    $('trace-title').textContent = `${AXES[index]} 响应曲线`;
    renderFeedback();
}

function renderFeedback() {
    const fresh = feedbackFresh();
    axisTable.classList.toggle('stale', !!latest && !fresh);
    rows.forEach(({ row, button, values }, index) => {
        const active = index === selected;
        row.dataset.selected = active;
        button.setAttribute('aria-pressed', String(active));
        const axis = latest?.[index];
        values[0].textContent = axis ? formatAngle(axis.targetDeg) : '—';
        values[1].textContent = axis?.sampled ? formatAngle(axis.feedbackDeg) : '—';
        values[2].textContent = axis?.sampled ? String(axis.adc) : '—';
        values[3].textContent = axis ? (axis.enabled ? '已使能' : '关闭') : '—';
        values[3].dataset.enabled = Boolean(fresh && axis?.enabled);
    });
    $('snapshot-status').textContent = latest ? (fresh ? `回包 ${lastReplyTime}` : `反馈已过期 · ${lastReplyTime}`) : '等待回包';
    updateControls();
    renderModel();
    drawTrace();
}

function modelOffset(axis, target = modelTargets[axis]) {
    return (target - SERVO_CENTER) * modelDirections[axis];
}

function renderModel() {
    if (!model) return;
    const ports = model.joints.map(joint => AXES.indexOf(joint.port));
    const desired = AXES.map((_, axis) => {
        const target = axis === selected && modelDraft !== null ? modelDraft : modelTargets[axis];
        return modelOffset(axis, target);
    });
    const angles = ports.map(axis => desired[axis]);
    const angle = ports.includes(selected) ? desired[selected] : undefined;
    modelAngle.value = angle ?? 0;
    $('model-axis').textContent = `${AXES[selected]} 偏角`;
    $('model-angle-value').textContent = angle === undefined ? '—' : `${formatAngle(angle)}°`;
    reverseButton.setAttribute('aria-pressed', String(modelDirections[selected] === -1));
    const pose = forward(model, angles);
    let status = modelFollow.checked ? '三维指令 · 实机跟随' : '三维指令 · 预览';
    if (modelFollow.checked && angle !== undefined && feedbackFresh() && !latest[selected].enabled) {
        status += ` · ${AXES[selected]} PWM 关闭`;
    }
    $('model-status').textContent = status;
    $('tcp').textContent = pose.tcp.map((value, index) => `${'XYZ'[index]} ${value.toFixed(1)}`).join('　');
    armView.setPose(pose, AXES[selected], modelSpan);
}

function planarTargets({ angles, yaw, roll }) {
    const target = (axis, offset) => {
        const degrees = SERVO_CENTER + offset * modelDirections[axis];
        return [axis, Math.round(degrees * 100) / 100];
    };
    const targets = angles.map((angle, i) => target(PITCH_AXES[i], angle));
    if (yaw !== null) targets.push(target(YAW_AXIS, yaw));
    if (roll !== null) targets.push(target(ROLL_AXIS, roll));
    return targets;
}

function planarMotionIssue(ik, targets) {
    if (ik !== planarIk) return { reason: '模型已更换，请重新选点' };
    if (!feedbackFresh()) return { reason: '等待串口反馈后可发送' };
    const disabled = targets.filter(([axis]) => !latest[axis].enabled).map(([axis]) => AXES[axis]);
    if (disabled.length) return { reason: `发送前请使能 ${disabled.join('、')} 的 PWM` };
    const start = PITCH_AXES.map(axis => modelOffset(axis, latest[axis].targetDeg));
    const end = targets.slice(0, PITCH_AXES.length).map(([axis, degrees]) => modelOffset(axis, degrees));
    const reason = ik.pathCollision(start, end);
    return reason ? { reason, pathBlocked: true } : null;
}

const validPitch = pitch => Number.isFinite(pitch) && Math.abs(pitch) <= 180;

function renderSpatialPreview(selection) {
    if (!planarIk) return;
    const board = feedbackFresh() ? model.joints.map(({ port }) => {
        const axis = AXES.indexOf(port);
        return modelOffset(axis, latest[axis].targetDeg);
    }) : null;
    const angles = selection?.angles && !selection.issue
        ? [selection.yaw ?? board?.[0] ?? sliceYaw, ...selection.angles, selection.roll ?? board?.[4] ?? 0]
        : null;
    const key = JSON.stringify([board, angles, selection?.world]);
    if (key === targetPreviewKey) return;
    targetPreviewKey = key;
    const reference = board ? forward(model, board) : null;
    const pose = angles ? forward(model, angles) : null;
    targetArmView.setPose(pose, null, modelSpan, reference, selection?.world);
}

function renderPlanar(busy) {
    const pitch = planarPitch.valueAsNumber;
    const selection = planarSelection;
    const yaw = selection?.yaw;
    const r = selection?.point ? selection.point[0] + planarIk.shoulderR : null;
    const yawText = yaw === null ? '保持' : (yaw === undefined ? '—' : `${yaw.toFixed(1)}°`);
    $('planar-point').textContent = `J1 ${yawText}　R ${r === null ? '—' : `${r.toFixed(1)} mm`}`;

    let issue;
    let pathBlocked = false;
    if (!planarIk) issue = '当前模型不支持空间选点';
    else if (!validPitch(pitch)) issue = '夹爪俯仰偏角须在 −180° 到 180° 之间';
    else if (selection?.issue) issue = selection.issue;
    else if (!planarIk.collision) issue = '仅几何预览 · 缺少平面防撞尺寸';
    else if (!selection) issue = '在视图中选点，或填全 XYZ 预览';
    else {
        const motion = planarMotionIssue(planarIk, planarTargets(selection));
        issue = motion?.reason;
        pathBlocked = !!motion?.pathBlocked;
    }

    planarView.setPose(validPitch(pitch) ? planarIk : null, pitch, selection?.point ? selection : null, pathBlocked);
    targetTopView.setTarget(planarIk, selection?.world, !!selection?.issue || pathBlocked);
    renderSpatialPreview(selection);
    if (!issue && busy) issue = '等待当前指令完成';
    planarSend.disabled = !!issue;
    $('planar-status').textContent = issue ?? (selection?.sent ? '板端已接收末端目标' : '已预览，可发送末端目标');
}

function pickPlanarPoint(x, z) {
    if (planarIk) setSpatialPoint(planarIk.worldPoint(sliceYaw, x, z));
}

function setSpatialPoint(point) {
    point.forEach((value, i) => { targetInputs[i].value = Number.isFinite(value) ? String(Number(value.toFixed(2))) : ''; });
    updateSpatialTarget();
}

function updateSpatialTarget() {
    const pitch = planarPitch.valueAsNumber;
    const world = targetInputs.map(input => input.valueAsNumber);
    const roll = targetRoll.value === '' && !targetRoll.validity.badInput ? null : targetRoll.valueAsNumber;
    planarSelection = null;
    if (planarIk && validPitch(pitch) && world.every(Number.isFinite)) {
        const pose = planarIk.solveSpatial(world, pitch, sliceYaw);
        const geometric = pose?.angles ? null : planarIk.solveSpatial(world, pitch, sliceYaw, false);
        let issue = pose?.angles ? null : '当前夹爪俯仰下不可达';
        if (geometric?.angles) issue = planarIk.collisionAt(geometric.angles);
        if (roll !== null && (!Number.isFinite(roll) || Math.abs(roll) > SERVO_CENTER)) issue = '工具滚转须在 −135° 到 135° 之间，或留空';
        const displayPose = geometric?.angles ? { ...geometric, angles: null } : pose;
        planarSelection = { ...displayPose, world, roll, issue };
        if (displayPose) sliceYaw = displayPose.sliceYaw;
    }
    updateControls();
}

function drawTrace() {
    const canvas = $('trace');
    const w = canvas.clientWidth, h = canvas.clientHeight;
    const dpr = window.devicePixelRatio || 1;
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(h * dpr);
    const ctx = canvas.getContext('2d');
    ctx.scale(dpr, dpr);
    const isAdc = $('trace-mode').value === 'adc';
    $('trace-legend').textContent = isAdc ? 'ADC 原始值 · 最近 30 秒' : '目标（绿） / 板端反馈（蓝） · 最近 30 秒';
    const now = performance.now();
    const windowed = history.filter(sample => now - sample.t <= HISTORY_MS);
    const traces = isAdc ? [{ field: 'adc', color: '#f5cd81' }] :
        [{ field: 'targetDeg', color: '#9df7d1' }, { field: 'feedbackDeg', color: '#88b9fb' }];
    const points = traces.map(trace => windowed.map(sample => {
        const axis = sample.axes[selected];
        return { t: sample.t, value: axis.sampled || trace.field === 'targetDeg' ? axis[trace.field] : null };
    }));
    const values = points.flatMap(series => series.map(point => point.value).filter(Number.isFinite));
    $('trace-empty').hidden = values.length > 0;
    if (!values.length || w < 96) return;
    const min = Math.min(...values), max = Math.max(...values);
    const pad = Math.max((max - min) * 0.12, isAdc ? 4 : 1);
    const lo = min - pad, hi = max + pad;
    const left = 55, top = 16, width = w - left - 17, height = h - 42;
    ctx.font = '11px Consolas, monospace';
    ctx.fillStyle = '#9eafb8';
    ctx.strokeStyle = '#293944';
    for (let i = 0; i <= 4; i++) {
        const y = top + height * i / 4;
        const value = hi - (hi - lo) * i / 4;
        ctx.beginPath();
        ctx.moveTo(left, y);
        ctx.lineTo(left + width, y);
        ctx.stroke();
        ctx.fillText(value.toFixed(isAdc ? 0 : 1), 10, y + 3);
    }
    ctx.fillText('−30 s', left, h - 8);
    ctx.fillText('现在', w - 43, h - 8);
    for (let i = 0; i < traces.length; i++) {
        const color = traces[i].color;
        ctx.beginPath();
        let previous;
        let last;
        for (const point of points[i]) {
            if (point.value === null) { previous = null; continue; }
            const x = left + width * (1 - (now - point.t) / HISTORY_MS);
            const y = top + height * (hi - point.value) / (hi - lo);
            if (!previous || point.t - previous.t > STALE_MS) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
            previous = point;
            last = [x, y];
        }
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        ctx.stroke();
        if (last) {
            ctx.beginPath();
            ctx.arc(...last, 3, 0, 2 * Math.PI);
            ctx.fillStyle = color;
            ctx.fill();
        }
    }
}

async function updateDevice() {
    if (phase !== 'ready' || operation) return;
    const current = link;
    const job = queued;
    queued = null;
    operation = job ? 'command' : 'poll';
    updateControls();
    try {
        if (job) {
            const issue = job.validate?.();
            if (issue) throw new Error(issue);
            const ack = await current.request(job.type, MSG.ack, job.payload);
            if (link !== current) return;
            checkAck(ack, job.type);
            job.complete?.(true);
            const message = `${job.label} · 板端已接收`;
            result(message);
            log(message);
        } else {
            const feedback = decodeFeedback(await current.request(CMD.feedback, MSG.feedback));
            if (link !== current) return;
            latest = feedback;
            latestAt = performance.now();
            lastReplyTime = clockText();
            history.push({ t: latestAt, axes: latest });
            history = history.filter(sample => latestAt - sample.t <= HISTORY_MS);
            if (logFault) log('反馈已恢复');
            logFault = false;
            renderFeedback();
        }
    } catch (error) {
        if (link !== current) return;
        if (job) {
            job.complete?.(false);
            result(`${job.label} · ${error.message}`, 'error');
        }
        if (job || !logFault) log(error.message, 'error');
        if (!job) logFault = true;
    } finally {
        if (link === current) {
            operation = null;
            updateControls();
            if (queued || job) void updateDevice();
        }
    }
}

function enqueue(type, payload, label, complete, validate) {
    if (phase !== 'ready' || operation === 'command' || queued) return false;
    queued = { type, payload, label, complete, validate };
    result(`${label} · 等待应答`);
    updateControls();
    void updateDevice();
    return true;
}

function finishTargets(targets, accepted) {
    modelDraft = null;
    if (accepted) {
        for (const [axis, degrees] of targets) modelTargets[axis] = degrees;
    }
    updateControls();
    renderModel();
}

function sendTargets(targets, label, complete = accepted => finishTargets(targets, accepted), validate) {
    return enqueue(CMD.targets, targetsPayload(targets), label, accepted => {
        if (accepted && latest) {
            latest = latest.slice();
            for (const [axis, targetDeg] of targets) latest[axis] = { ...latest[axis], targetDeg };
        }
        complete(accepted);
    }, validate);
}

function setModelTargets(targets, label) {
    if (!modelFollow.checked) finishTargets(targets, true);
    else if (!feedbackFresh() || !sendTargets(targets, label)) finishTargets(targets, false);
}

async function disconnect(message, failed = false) {
    const current = link;
    link = null;
    queued = null;
    operation = null;
    modelFollow.checked = false;
    modelDraft = null;
    if (planarSelection) planarSelection.sent = false;
    setPhase('closing', '正在关闭');
    await current?.close();
    setPhase(failed ? 'error' : 'idle', failed ? '连接中断' : '未连接');
    log(message, failed ? 'error' : 'info');
    result(failed ? message : '串口已关闭，板端保持最后的 PWM 状态', failed ? 'error' : 'info');
    renderFeedback();
}

$('connect').addEventListener('click', async () => {
    if (phase === 'ready') {
        await disconnect('串口已断开');
        return;
    }
    setPhase('opening', '选择串口');
    try {
        const port = await navigator.serial.requestPort();
        setPhase('opening', '协议握手');
        const session = new ArmSerial(error => {
            if (link === session && phase === 'ready') void disconnect(error.message, true);
        });
        link = session;
        await session.open(port);
        latest = null;
        history = [];
        logFault = false;
        setPhase('ready', '已连接');
        log(`USART1 V${VERSION} 握手通过`);
        result('可发送单轴指令');
        renderFeedback();
        void updateDevice();
    } catch (error) {
        if (error.name === 'NotFoundError') setPhase('idle', '未连接');
        else await disconnect(error.message, true);
    }
});

$('joint').addEventListener('change', event => selectJoint(Number(event.target.value)));
angleInput.addEventListener('input', () => { edits[selected] = angleInput.value; });
$('target-form').addEventListener('submit', event => {
    event.preventDefault();
    const degrees = angleInput.valueAsNumber;
    try { sendTargets([[selected, degrees]], `${AXES[selected]} 目标 ${formatAngle(degrees)}°`); }
    catch (error) { result(error.message, 'error'); }
});
for (const [id, enabled, all] of [
    ['enable', 1, false], ['disable', 0, false],
    ['enable-all', 1, true], ['disable-all', 0, true],
]) {
    $(id).addEventListener('click', () => {
        if ($(id).disabled) return;
        const mask = all ? ALL_PWM_CHANNELS : 1 << selected;
        const scope = all ? '全部' : AXES[selected];
        enqueue(CMD.enable, new Uint8Array([mask, enabled]), `${scope} ${enabled ? '使能' : '失能'} PWM`);
    });
}
$('clear-log').addEventListener('click', () => $('log').replaceChildren());
$('trace-mode').addEventListener('change', drawTrace);
new ResizeObserver(drawTrace).observe($('trace'));

modelFollow.addEventListener('change', () => {
    modelDraft = null;
    if (modelFollow.checked) {
        modelFollow.checked = feedbackFresh();
        if (modelFollow.checked) latest.forEach((axis, i) => { modelTargets[i] = axis.targetDeg; });
    }
    updateControls();
    renderModel();
});

modelAngle.addEventListener('input', () => {
    modelDraft = SERVO_CENTER + modelAngle.valueAsNumber * modelDirections[selected];
    updateControls();
    renderModel();
});

modelAngle.addEventListener('change', () => {
    if (modelDraft === null) return;
    setModelTargets([[selected, modelDraft]], `${AXES[selected]} 目标 ${formatAngle(modelDraft)}°`);
});

modelAngle.addEventListener('pointercancel', () => {
    modelDraft = null;
    renderModel();
});

reverseButton.addEventListener('click', () => {
    modelDirections[selected] *= -1;
    modelDraft = null;
    if (planarSelection) planarSelection.sent = false;
    updateControls();
    renderModel();
});

$('model-home').addEventListener('click', () => {
    const centers = model.joints.map(joint => [AXES.indexOf(joint.port), SERVO_CENTER]);
    setModelTargets(centers, '整臂回中');
});

for (const input of [...targetInputs, planarPitch, targetRoll]) input.addEventListener('input', updateSpatialTarget);
for (const [i, axis] of ['x', 'y', 'z'].entries()) {
    for (const [label, sign] of [['minus', -1], ['plus', 1]]) {
        $(`jog-${axis}-${label}`).addEventListener('click', () => {
            const point = targetInputs.map(input => input.valueAsNumber);
            if (!point.every(Number.isFinite)) return;
            point[i] += sign * Number($('target-step').value);
            setSpatialPoint(point);
        });
    }
}

planarSend.addEventListener('click', () => {
    if (planarSend.disabled) return;
    const ik = planarIk;
    const selection = planarSelection;
    const targets = planarTargets(selection);
    sendTargets(targets, '末端空间目标', accepted => {
        if (selection === planarSelection) selection.sent = accepted;
    }, () => planarMotionIssue(ik, targets)?.reason);
});

function loadModel(data) {
    const measured = readModel(data);
    if (!measured) return;
    const ik = createPlanarIk(measured, SERVO_CENTER);
    model = measured;
    planarIk = ik;
    planarSelection = null;
    sliceYaw = 0;
    targetPreviewKey = undefined;
    planarPitch.value = '0';
    targetRoll.value = '';
    const defaults = ik ? [...ik.baseXY, ik.shoulderZ + ik.reach * 0.87] : [NaN, NaN, NaN];
    defaults.forEach((value, i) => { targetInputs[i].value = Number.isFinite(value) ? String(Number(value.toFixed(2))) : ''; });
    $('planar-panel').hidden = !planarIk;
    modelFollow.checked = false;
    modelDraft = null;
    modelTargets.fill(SERVO_CENTER);
    modelDirections.fill(1);
    modelSpan = Math.max(...[...model.joints.map(joint => joint.point), model.tcp].map(point => Math.hypot(...point)), 1);
    $('model-empty').hidden = true;
    $('pose-control').hidden = false;
    renderFeedback();
}

$('model-file').addEventListener('change', async event => {
    const file = event.target.files[0];
    if (!file) return;
    try { loadModel(JSON.parse(await file.text())); }
    catch (error) { log(`模型导入失败 · ${error.message}`, 'error'); }
    event.target.value = '';
});

try {
    const response = await fetch('./model.json', { cache: 'no-store' });
    if (response.ok && !model) loadModel(await response.json());
} catch (error) {
    log(`模型配置读取失败 · ${error.message}`, 'error');
}

if (navigator.serial) {
    setPhase('idle', '未连接');
} else {
    setPhase('error', 'Web Serial 不可用');
    result('请在桌面 Chrome 中通过 localhost 打开', 'error');
}
selectJoint(0);
setInterval(() => {
    if (!latest || performance.now() - latestAt > STALE_MS) renderFeedback();
    void updateDevice();
}, POLL_MS);
