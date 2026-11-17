export class PlanarView {
    constructor(canvas, onPick) {
        this.canvas = canvas;
        canvas.addEventListener('click', event => {
            if (!this.layout) return;
            const rect = canvas.getBoundingClientRect();
            const { left, right, top, bottom, originX, scale, zMin } = this.layout;
            const px = event.clientX - rect.left;
            const py = event.clientY - rect.top;
            if (px >= left && px <= right && py >= top && py <= bottom) {
                onPick((px - originX) / scale, zMin + (bottom - py) / scale);
            }
        });
        new ResizeObserver(() => this.draw()).observe(canvas);
    }

    setPose(ik, pitch, selected, pathBlocked) {
        if (ik === this.ik && selected === this.selected && Object.is(pitch, this.pitch) && pathBlocked === this.pathBlocked) return;
        this.ik = ik;
        this.pitch = pitch;
        this.selected = selected;
        this.pathBlocked = pathBlocked;
        this.draw();
    }

    draw() {
        const { canvas, ik, pitch, selected } = this;
        this.layout = null;
        const width = canvas.clientWidth;
        const height = canvas.clientHeight;
        if (!width || !height) return;
        const dpr = window.devicePixelRatio || 1;
        canvas.width = Math.round(width * dpr);
        canvas.height = Math.round(height * dpr);
        const ctx = canvas.getContext('2d');
        ctx.scale(dpr, dpr);
        if (!ik) return;

        const extent = Math.ceil(ik.reach / 50) * 50;
        const zMin = Math.min(0, ik.collision?.floorZ ?? 0);
        const zMax = Math.max(zMin + 50, Math.ceil((ik.shoulderZ + ik.reach) / 50) * 50 + 10);
        const originX = width / 2;
        const bottom = height - 29;
        const scale = Math.min((width - 82) / (2 * extent), (bottom - 18) / (zMax - zMin));
        if (scale <= 0) return;
        const left = originX - extent * scale;
        const right = originX + extent * scale;
        const top = bottom - (zMax - zMin) * scale;
        this.layout = { left, right, top, bottom, originX, scale, zMin };
        const toPixel = ([x, z]) => [originX + x * scale, bottom - (z - zMin) * scale];
        function line(a, b, color, size = 1) {
            ctx.beginPath();
            ctx.moveTo(...toPixel(a));
            ctx.lineTo(...toPixel(b));
            ctx.strokeStyle = color;
            ctx.lineWidth = size;
            ctx.stroke();
        }

        ctx.fillStyle = '#203b3c';
        const step = Math.max(12, extent / 40, (zMax - zMin) / 40);
        for (let x = -extent; x < extent; x += step) {
            for (let z = zMin; z < zMax; z += step) {
                if (!ik.solve(x + step / 2, z + step / 2, pitch)) continue;
                const [px, py] = toPixel([x, z + step]);
                ctx.fillRect(px, py, step * scale + 0.3, step * scale + 0.3);
            }
        }

        ctx.fillStyle = '#9eafb8';
        ctx.font = '11px monospace';
        ctx.textAlign = 'center';
        const xStep = width < 470 ? 100 : 50;
        for (let x = -Math.floor(extent / xStep) * xStep; x <= extent; x += xStep) {
            line([x, zMin], [x, zMax], x === 0 ? '#45616c' : '#283945');
            ctx.fillText(String(Math.round(x + ik.shoulderR)), toPixel([x, zMin])[0], bottom + 15);
        }
        ctx.textAlign = 'right';
        for (let z = Math.ceil(zMin / 50) * 50; z <= zMax; z += 50) {
            line([-extent, z], [extent, z], z === 0 ? '#45616c' : '#283945');
            ctx.fillText(String(z), left - 6, toPixel([0, z])[1] + 3);
        }
        ctx.textAlign = 'left';
        ctx.fillText('高度 mm', left + 4, top - 5);
        ctx.textAlign = 'right';
        ctx.fillText('R mm', right, bottom - 6);

        if (ik.collision) {
            const { floorZ, baseTopZ, baseRadius, baseX } = ik.collision;
            const [x, y] = toPixel([baseX - baseRadius, baseTopZ]);
            ctx.fillStyle = '#683d3a';
            ctx.fillRect(x, y, 2 * baseRadius * scale, (baseTopZ - floorZ) * scale);
            line([-extent, floorZ], [extent, floorZ], '#c78375', 2);
        }
        if (!selected) return;
        if (selected.angles) {
            const pose = ik.project(selected.angles);
            for (let i = 1; i < pose.length; i++) line(pose[i - 1], pose[i], '#79c7cd', 3);
        }
        const [x, z] = selected.point;
        const marker = selected.issue || this.pathBlocked ? '#f3937f' : '#f5cd81';
        const size = 6 / scale;
        line([x - size, z], [x + size, z], marker, 2);
        line([x, z - size], [x, z + size], marker, 2);
    }
}

export class TopView {
    constructor(canvas, onPick) {
        this.canvas = canvas;
        canvas.addEventListener('click', event => {
            if (!this.layout) return;
            const rect = canvas.getBoundingClientRect();
            const { cx, cy, scale, extent, base } = this.layout;
            const x = (event.clientX - rect.left - cx) / scale;
            const y = (cy - event.clientY + rect.top) / scale;
            if (Math.abs(x) <= extent && Math.abs(y) <= extent) onPick(base[0] + x, base[1] + y);
        });
        new ResizeObserver(() => this.draw()).observe(canvas);
    }

    setTarget(ik, target, blocked) {
        if (this.ik === ik && this.target === target && this.blocked === blocked) return;
        this.ik = ik;
        this.target = target;
        this.blocked = blocked;
        this.draw();
    }

    draw() {
        const { canvas, ik, target, blocked } = this;
        const cx = canvas.clientWidth / 2, cy = canvas.clientHeight / 2;
        const dpr = window.devicePixelRatio || 1;
        canvas.width = Math.round(cx * 2 * dpr);
        canvas.height = Math.round(cy * 2 * dpr);
        this.layout = null;
        if (!ik || Math.min(cx, cy) <= 24) return;
        const extent = Math.ceil((ik.reach + Math.abs(ik.shoulderR)) / 50) * 50;
        const scale = (Math.min(cx, cy) - 24) / extent;
        const base = ik.baseXY;
        this.layout = { cx, cy, scale, extent, base };
        const ctx = canvas.getContext('2d');
        ctx.scale(dpr, dpr);
        const line = (x1, y1, x2, y2, color) => {
            ctx.beginPath();
            ctx.moveTo(cx + x1 * scale, cy - y1 * scale);
            ctx.lineTo(cx + x2 * scale, cy - y2 * scale);
            ctx.strokeStyle = color;
            ctx.stroke();
        };
        for (let i = -4; i <= 4; i++) {
            const p = i * extent / 4;
            const color = i ? '#283945' : '#54717d';
            line(-extent, p, extent, p, color);
            line(p, -extent, p, extent, color);
        }
        ctx.font = '11px monospace';
        ctx.fillStyle = '#9eafb8';
        ctx.fillText('X', cx + extent * scale + 5, cy - 4);
        ctx.fillText('Y', cx + 5, cy - extent * scale - 5);
        ctx.fillText('J1', cx + 5, cy + 13);
        if (!target) return;
        const x = target[0] - base[0], y = target[1] - base[1];
        if (Math.abs(x) > extent || Math.abs(y) > extent) return;
        const color = blocked ? '#f3937f' : '#f5cd81';
        line(0, 0, x, y, color);
        ctx.fillStyle = color;
        ctx.beginPath();
        ctx.arc(cx + x * scale, cy - y * scale, 5, 0, 2 * Math.PI);
        ctx.fill();
    }
}
