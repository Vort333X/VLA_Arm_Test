const mix = (a, b, t) => a.map((v, i) => v + (b[i] - v) * t);
const cross = (a, b, c) => (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);

function pointToSegment(p, a, b) {
    const dx = b[0] - a[0], dz = b[1] - a[1];
    const length2 = dx * dx + dz * dz;
    const t = length2 ? Math.max(0, Math.min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dz) / length2)) : 0;
    return Math.hypot(p[0] - a[0] - dx * t, p[1] - a[1] - dz * t);
}

function segmentDistance(a, b, c, d) {
    const boxesOverlap = [0, 1].every(i =>
        Math.max(a[i], b[i]) >= Math.min(c[i], d[i]) && Math.max(c[i], d[i]) >= Math.min(a[i], b[i]));
    if (boxesOverlap && cross(a, b, c) * cross(a, b, d) <= 0 && cross(c, d, a) * cross(c, d, b) <= 0) return 0;
    return Math.min(pointToSegment(a, c, d), pointToSegment(b, c, d), pointToSegment(c, a, b), pointToSegment(d, a, b));
}

function intersectsBox(a, b, min, max) {
    let enter = 0, exit = 1;
    for (let i = 0; i < 2; i++) {
        const delta = b[i] - a[i];
        if (!delta) {
            if (a[i] < min[i] || a[i] > max[i]) return false;
            continue;
        }
        const t1 = (min[i] - a[i]) / delta, t2 = (max[i] - a[i]) / delta;
        enter = Math.max(enter, Math.min(t1, t2));
        exit = Math.min(exit, Math.max(t1, t2));
        if (enter > exit) return false;
    }
    return true;
}

export function createPlanarCollision(config, baseX, lengths) {
    if (!config) return null;
    const { floorZ, baseTopZ, baseRadius, linkRadii, margin, shoulderMount, jointRelief } = config;
    if (![floorZ, baseTopZ, baseRadius, margin, shoulderMount, jointRelief].every(Number.isFinite) ||
        baseTopZ <= floorZ || baseRadius <= 0 || margin < 0 || shoulderMount <= 0 || shoulderMount >= lengths[0] ||
        jointRelief <= 0 || jointRelief >= Math.min(...lengths) || !Array.isArray(linkRadii) || linkRadii.length !== 3 ||
        linkRadii.some(r => !Number.isFinite(r) || r <= 0)) throw new Error('平面防撞尺寸无效');

    function check(points) {
        if (linkRadii.some((r, i) => Math.min(points[i][1], points[i + 1][1]) <= floorZ + r + margin)) {
            return '连杆或夹爪低于桌面净空';
        }
        for (let i = 0; i < 3; i++) {
            const [a, b] = [points[i], points[i + 1]];
            const pad = linkRadii[i] + margin;
            // 肩部最靠近底座的一小段属于正常安装区域
            const start = i ? a : mix(a, b, shoulderMount / lengths[0]);
            if (intersectsBox(start, b, [baseX - baseRadius - pad, floorZ - pad],
                [baseX + baseRadius + pad, baseTopZ + pad])) return '连杆或夹爪进入底座净空';
        }

        for (let i = 0; i < 3; i++) {
            for (let j = i + 1; j < 3; j++) {
                const [a, d] = [points[i], points[j + 1]];
                let [b, c] = [points[i + 1], points[j]];
                // 相邻连杆在关节处相连，检查从安装区向外伸出的部分
                if (j === i + 1) {
                    b = mix(b, a, jointRelief / lengths[i]);
                    c = mix(c, d, jointRelief / lengths[j]);
                }
                if (segmentDistance(a, b, c, d) <= linkRadii[i] + linkRadii[j] + margin) return '连杆或夹爪之间净空不足';
            }
        }
        return null;
    }

    return { floorZ, baseTopZ, baseRadius, baseX, check };
}
