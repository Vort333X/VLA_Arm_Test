import { AXES, CMD, MSG, checkAck, decodeFeedback, targetPayload } from './protocol.mjs';
import { ArmSerial } from './serial.mjs';
import { ArmView, feedbackAngles, forward, readModel } from './arm-model.mjs';

const POLL_MS = 200;
const STALE_MS = 1000;
const HISTORY_MS = 30_000;
const $ = id => document.getElementById(id);
const axisTable = $('axis-table');
const angleInput = $('target');
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
const armView = new ArmView($('arm-view'), port => selectJoint(AXES.indexOf(port)));
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
    connect.textContent = phase === 'ready' ? '断开' : '连接 USB';
    const busy = phase !== 'ready' || operation === 'command' || !!queued;
    for (const id of ['send', 'disable']) $(id).disabled = busy;
    $('enable').disabled = busy || !feedbackFresh();
}

function setPhase(next, text) {
    phase = next;
    $('link-state').dataset.state = next;
    $('link-state').textContent = text;
    updateControls();
}

function selectJoint(index) {
    if (index < 0) return;
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
    if (model) {
        const angles = latest && feedbackAngles(model, latest);
        const pose = forward(model, angles || model.joints.map(() => 0));
        $('model-status').textContent = angles ? (fresh ? '反馈推算姿态' : '反馈姿态已过期') : '参考姿态 · 等待 ADC 标定与反馈';
        $('tcp').textContent = pose.tcp.map((value, index) => `${'XYZ'[index]} ${value.toFixed(1)}`).join('　');
        armView.setPose(pose, AXES[selected], modelSpan);
    }
    drawTrace();
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
            const ack = await current.request(job.type, MSG.ack, job.payload);
            if (link !== current) return;
            checkAck(ack, job.type);
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
        if (job) result(`${job.label} · ${error.message}`, 'error');
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

function enqueue(type, payload, label) {
    if (phase !== 'ready' || operation === 'command' || queued) return;
    queued = { type, payload, label };
    result(`${label} · 等待应答`);
    updateControls();
    void updateDevice();
}

async function disconnect(message, failed = false) {
    const current = link;
    link = null;
    queued = null;
    operation = null;
    setPhase('closing', '正在关闭');
    await current?.close();
    setPhase(failed ? 'error' : 'idle', failed ? '连接中断' : '未连接');
    log(message, failed ? 'error' : 'info');
    result(failed ? message : '串口已关闭，板端保持最后的 PWM 状态', failed ? 'error' : 'info');
    renderFeedback();
}

$('connect').addEventListener('click', async () => {
    if (phase === 'ready') {
        await disconnect('USB 已断开');
        return;
    }
    setPhase('opening', '选择串口');
    try {
        const port = await navigator.serial.requestPort({ filters: [{ usbVendorId: 0x0483, usbProductId: 0x5740 }] });
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
        log('USB CDC V1 握手通过');
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
    try { enqueue(CMD.targets, targetPayload(selected, degrees), `${AXES[selected]} 目标 ${formatAngle(degrees)}°`); }
    catch (error) { result(error.message, 'error'); }
});
for (const [id, enabled] of [['enable', 1], ['disable', 0]]) {
    $(id).addEventListener('click', () => enqueue(CMD.enable,
        new Uint8Array([1 << selected, enabled]), `${AXES[selected]} ${enabled ? '使能' : '关闭'} PWM`));
}
$('clear-log').addEventListener('click', () => $('log').replaceChildren());
$('trace-mode').addEventListener('change', drawTrace);
new ResizeObserver(drawTrace).observe($('trace'));

function loadModel(data) {
    const measured = readModel(data);
    if (!measured) return;
    model = measured;
    modelSpan = Math.max(...[...model.joints.map(joint => joint.point), model.tcp].map(point => Math.hypot(...point)), 1);
    $('model-empty').hidden = true;
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

if (!navigator.serial) {
    setPhase('error', 'Web Serial 不可用');
    result('请在桌面 Chrome 中通过 localhost 打开', 'error');
}
selectJoint(0);
setInterval(() => {
    if (!latest || performance.now() - latestAt > STALE_MS) renderFeedback();
    void updateDevice();
}, POLL_MS);
