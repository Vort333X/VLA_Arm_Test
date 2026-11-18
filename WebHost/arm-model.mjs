import { AXES } from './protocol.mjs?v=axes-1';

const legacyPorts = new Map([
    ['H3', 'J1'], ['H7', 'J2'], ['H5', 'J3'], ['H10', 'J4'], ['H8', 'J5'], ['H6', 'J6'],
]);

const vector = value => Array.isArray(value) && value.length === 3 && value.every(Number.isFinite);
const add = (a, b) => a.map((value, i) => value + b[i]);
const sub = (a, b) => a.map((value, i) => value - b[i]);
const scale = (a, factor) => a.map(value => value * factor);
const dot = (a, b) => a.reduce((sum, value, i) => sum + value * b[i], 0);
const cross = ([x, y, z], [u, v, w]) => [y * w - z * v, z * u - x * w, x * v - y * u];

export function readModel(data) {
    if (Array.isArray(data?.joints) && !data.joints.length && data.tcp === null) return null;
    if (!Array.isArray(data?.joints) || !data.joints.length || data.joints.length > AXES.length || !vector(data.tcp)) {
        throw new Error('请填写关节列表与参考姿态的 TCP 坐标');
    }
    const ports = new Set();
    const joints = data.joints.map((joint, i) => {
        const { point, axis, adc } = joint;
        const port = legacyPorts.get(joint.port) ?? joint.port;
        if (!AXES.includes(port) || ports.has(port)) throw new Error(`第 ${i + 1} 项的轴号无效或重复`);
        const name = port;
        ports.add(port);
        if (!vector(point) || !vector(axis)) throw new Error(`${name} 的轴心和轴方向应为三个数`);
        const length = Math.hypot(...axis);
        if (length < 1e-6) throw new Error(`${name} 的轴方向不能为零`);
        if (adc !== undefined && (!adc || !Number.isInteger(adc.zero) || !Number.isInteger(adc.atSample) ||
            adc.zero < 0 || adc.zero > 4095 || adc.atSample < 0 || adc.atSample > 4095 ||
            !Number.isFinite(adc.angleDeg) || adc.atSample === adc.zero || adc.angleDeg === 0)) {
            throw new Error(`${name} 的 ADC 标定需要两个不同的测量点`);
        }
        return { name, port, point, axis: scale(axis, 1 / length), adc };
    });
    const commandLimits = data.commandLimits === undefined ? {} : data.commandLimits;
    if (!commandLimits || typeof commandLimits !== 'object' || Array.isArray(commandLimits)) {
        throw new Error('commandLimits 应按轴号填写命令角范围');
    }
    for (const [port, range] of Object.entries(commandLimits)) {
        if (!AXES.includes(port) || !Array.isArray(range) || range.length !== 2 || !range.every(Number.isFinite) ||
            range[0] < 0 || range[0] > range[1] || range[1] > 270) {
            throw new Error(`${port} 的命令角限位无效`);
        }
    }
    return { joints, tcp: data.tcp, collision: data.collision, commandLimits };
}

function rotate(point, joint, angle) {
    const relative = sub(point, joint.point);
    const c = Math.cos(angle);
    const s = Math.sin(angle);
    const rotated = add(add(scale(relative, c), scale(cross(joint.axis, relative), s)),
        scale(joint.axis, dot(joint.axis, relative) * (1 - c)));
    return add(joint.point, rotated);
}

// 轴线和 TCP 坐标均以参考姿态的底座坐标系为基准
export function forward(model, degrees) {
    const radians = degrees.map(value => value * Math.PI / 180);
    function transform(point, upstream) {
        for (let i = upstream - 1; i >= 0; i--) point = rotate(point, model.joints[i], radians[i]);
        return point;
    }
    const joints = model.joints.map((joint, i) => {
        const point = transform(joint.point, i);
        return { ...joint, point, axis: sub(transform(add(joint.point, joint.axis), i), point) };
    });
    // 在尖端画一根垂直于末轴的小线，便于观察 roll
    const lastAxis = model.joints.at(-1).axis;
    const across = cross(lastAxis, Math.abs(lastAxis[1]) < 0.9 ? [0, 1, 0] : [0, 0, 1]);
    const mark = add(model.tcp, scale(across, 16 / Math.hypot(...across)));
    return { joints, tcp: transform(model.tcp, joints.length), tipMark: transform(mark, joints.length) };
}

export function feedbackAngles(model, axes) {
    const angles = [];
    for (const joint of model.joints) {
        const value = axes.find(axis => axis.port === joint.port);
        if (!value?.sampled || !joint.adc) return null;
        const { zero, atSample, angleDeg } = joint.adc;
        angles.push((value.adc - zero) * angleDeg / (atSample - zero));
    }
    return angles;
}

export class ArmView {
    yaw = -0.85;
    pitch = 0.53;
    pose = null;

    constructor(canvas, onJoint) {
        this.canvas = canvas;
        this.onJoint = onJoint;
        this.nodes = [];
        let drag;
        canvas.addEventListener('pointerdown', event => {
            if (!this.span) return;
            drag = { x: event.offsetX, y: event.offsetY, yaw: this.yaw, pitch: this.pitch, moved: false };
            canvas.setPointerCapture(event.pointerId);
        });
        canvas.addEventListener('pointermove', event => {
            if (!drag) return;
            const dx = event.offsetX - drag.x;
            const dy = event.offsetY - drag.y;
            if (Math.hypot(dx, dy) > 4) drag.moved = true;
            if (!drag.moved) return;
            this.yaw = drag.yaw - dx * 0.009;
            this.pitch = Math.max(-0.1, Math.min(1.5, drag.pitch + dy * 0.009));
            this.draw();
        });
        canvas.addEventListener('pointerup', event => {
            if (drag && !drag.moved) {
                const hit = this.nodes.find(node => Math.hypot(node.x - event.offsetX, node.y - event.offsetY) < 14);
                if (hit) this.onJoint?.(hit.port);
            }
            drag = null;
        });
        canvas.addEventListener('lostpointercapture', () => { drag = null; });
        new ResizeObserver(() => this.draw()).observe(canvas);
    }

    setPose(pose, selected, span, reference = null, target = null) {
        this.pose = pose;
        this.selected = selected;
        this.span = span;
        this.reference = reference;
        this.target = target;
        this.draw();
    }

    draw() {
        const canvas = this.canvas;
        const w = canvas.clientWidth;
        const h = canvas.clientHeight;
        const dpr = window.devicePixelRatio || 1;
        canvas.width = Math.round(w * dpr);
        canvas.height = Math.round(h * dpr);
        const ctx = canvas.getContext('2d');
        ctx.scale(dpr, dpr);
        this.nodes = [];
        if (!this.span || !w || !h) return;

        const factor = Math.min(w * 0.40, h * 0.62) / (this.span || 1);
        const cy = Math.cos(this.yaw), sy = Math.sin(this.yaw);
        const cp = Math.cos(this.pitch), sp = Math.sin(this.pitch);
        const project = ([x, y, z]) => ({ x: w * 0.50 + (cy * x - sy * y) * factor,
            y: h * 0.72 - (cp * z + sp * (sy * x + cy * y)) * factor });
        function line(a, b, color, width) {
            ctx.beginPath();
            ctx.moveTo(a.x, a.y);
            ctx.lineTo(b.x, b.y);
            ctx.strokeStyle = color;
            ctx.lineWidth = width;
            ctx.stroke();
        }
        const grid = this.span / 8;
        const edge = 7 * grid;
        for (let i = -7; i <= 7; i++) {
            line(project([i * grid, -edge, 0]), project([i * grid, edge, 0]), '#23313c', 1);
            line(project([-edge, i * grid, 0]), project([edge, i * grid, 0]), '#23313c', 1);
        }
        const origin = project([0, 0, 0]);
        ['X', 'Y', 'Z'].forEach((label, index) => {
            const end = [0, 0, 0];
            end[index] = grid * 2;
            const tip = project(end);
            const color = ['#f3937f', '#8dd4a8', '#88b9fb'][index];
            line(origin, tip, color, 1.5);
            ctx.fillStyle = color;
            ctx.font = '12px monospace';
            ctx.fillText(label, tip.x + 4, tip.y - 4);
        });
        if (this.reference) {
            const points = [...this.reference.joints.map(joint => joint.point), this.reference.tcp].map(project);
            for (let i = 1; i < points.length; i++) line(points[i - 1], points[i], '#647786', 3);
        }
        if (this.pose) this.drawPose(ctx, project, line, origin, grid);
        if (this.target) {
            const tip = project(this.target);
            line({ x: tip.x - 7, y: tip.y }, { x: tip.x + 7, y: tip.y }, '#f5cd81', 2);
            line({ x: tip.x, y: tip.y - 7 }, { x: tip.x, y: tip.y + 7 }, '#f5cd81', 2);
            ctx.fillStyle = '#f5cd81';
            ctx.fillText('目标', tip.x + 12, tip.y + 13);
        }
    }

    drawPose(ctx, project, line, origin, grid) {
        this.nodes = this.pose.joints.map(joint => ({ ...project(joint.point), port: joint.port }));
        const tcp = project(this.pose.tcp);
        const points = [origin, ...this.nodes, tcp];
        ctx.lineCap = 'round';
        for (let i = 1; i < points.length; i++) {
            line(points[i - 1], points[i], '#172a36', 12);
            line(points[i - 1], points[i], '#5db9c3', 5);
        }
        this.pose.joints.forEach((joint, i) => {
            const node = this.nodes[i];
            const active = node.port === this.selected;
            line(project(sub(joint.point, scale(joint.axis, grid * 0.46))),
                project(add(joint.point, scale(joint.axis, grid * 0.46))), '#88b9fb', 1.5);
            ctx.beginPath();
            ctx.arc(node.x, node.y, active ? 8 : 6, 0, 2 * Math.PI);
            ctx.fillStyle = active ? '#9df7d1' : '#183645';
            ctx.fill();
            ctx.strokeStyle = '#a4e3ea';
            ctx.lineWidth = 1.5;
            ctx.stroke();
            ctx.fillStyle = '#d4e9eb';
            ctx.font = '12px monospace';
            ctx.fillText(joint.name, node.x + 11, node.y - 8);
        });
        line(tcp, project(this.pose.tipMark), '#f5cd81', 2);
        ctx.fillStyle = '#f5cd81';
        ctx.fillRect(tcp.x - 4, tcp.y - 4, 8, 8);
        ctx.fillText('TCP', tcp.x + 9, tcp.y - 8);
    }
}
