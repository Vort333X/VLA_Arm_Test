import { createPlanarCollision } from './planar-collision.mjs';

export const PITCH_PORTS = ['H7', 'H5', 'H10'];

const DEG = Math.PI / 180;
const UP = [0, 0, 1];
const dot = (a, b) => a.reduce((sum, n, i) => sum + n * b[i], 0);
const sub = (a, b) => a.map((n, i) => n - b[i]);
const cross = ([x, y, z], [u, v, w]) => [y * w - z * v, z * u - x * w, x * v - y * u];
const nearZero = v => Math.hypot(...v) < 0.05;
const wrap = angle => Math.atan2(Math.sin(angle), Math.cos(angle));

export function createPlanarIk(model, limitDeg) {
    const ports = ['H3', ...PITCH_PORTS, 'H8'];
    if (model.joints.length !== ports.length || model.joints.some((joint, i) => joint.port !== ports[i])) return null;

    const [yaw, shoulder, elbow, wrist, roll] = model.joints;
    const normal = shoulder.axis;
    if (Math.abs(normal[2]) > 1e-6 || Math.abs(yaw.axis[2]) < 1 - 1e-6) return null;
    const pitches = [shoulder, elbow, wrist];
    if (pitches.some(joint => Math.abs(dot(normal, joint.axis)) < 1 - 1e-6)) return null;
    const sign = pitches.map(joint => Math.sign(dot(normal, joint.axis)));
    const front = cross(normal, UP);
    const shoulderFromYaw = sub(shoulder.point, yaw.point);
    if (Math.abs(dot(shoulderFromYaw, normal)) > 0.05) return null;
    const shoulderR = dot(shoulderFromYaw, front);
    const points = [shoulder.point, elbow.point, wrist.point, model.tcp];
    const links = points.slice(1).map((p, i) => sub(p, points[i]));
    if (links.some(link => Math.abs(dot(link, normal)) > 0.05)) return null;
    if (!nearZero(cross(roll.axis, links[2])) || !nearZero(cross(roll.axis, sub(model.tcp, roll.point)))) return null;

    const lengths = links.map(link => Math.hypot(dot(link, front), link[2]));
    if (lengths.some(length => length < 0.05)) return null;
    const reference = links.map(link => Math.atan2(dot(link, front), link[2]));
    const [upper, lower, tool] = lengths;
    const shoulderZ = shoulder.point[2];
    const reach = upper + lower + tool;
    const collision = createPlanarCollision(model.collision, -shoulderR, lengths);

    function linkAngles(angles) {
        let rotation = 0;
        return reference.map((angle, i) => {
            rotation += angles[i] * sign[i] * DEG;
            return angle + rotation;
        });
    }

    function project(angles) {
        const joints = [[0, shoulderZ]];
        linkAngles(angles).forEach((a, i) => {
            const [x, z] = joints.at(-1);
            joints.push([x + lengths[i] * Math.sin(a), z + lengths[i] * Math.cos(a)]);
        });
        return joints;
    }

    const collisionAt = angles => collision?.check(project(angles)) ?? null;

    function pathCollision(from, to) {
        const steps = Math.max(1, Math.ceil(Math.max(...to.map((angle, i) => Math.abs(angle - from[i]))) / 2));
        for (let step = 0; step <= steps; step++) {
            const angles = to.map((angle, i) => from[i] + (angle - from[i]) * step / steps);
            const issue = collisionAt(angles);
            if (issue) return step === 0 ? `起始姿态：${issue}` : `途中姿态：${issue}`;
        }
        return null;
    }

    function solve(x, z, pitch, checkCollision = true) {
        // 按设定的夹爪俯仰，把目标换算成腕轴位置
        const toolAngle = reference[2] + pitch * DEG;
        const wx = x - tool * Math.sin(toolAngle);
        const wz = z - shoulderZ - tool * Math.cos(toolAngle);
        const cosine = (wx * wx + wz * wz - upper * upper - lower * lower) / (2 * upper * lower);
        if (!Number.isFinite(cosine) || Math.abs(cosine) > 1 + 1e-8) return null;
        const bend = Math.acos(Math.max(-1, Math.min(1, cosine)));
        let best = null;
        let bestCost = Infinity;
        for (const elbowAngle of [bend, -bend]) {
            const upperAngle = Math.atan2(wx, wz) - Math.atan2(lower * Math.sin(elbowAngle), upper + lower * Math.cos(elbowAngle));
            const turnA = upperAngle - reference[0];
            const turnB = upperAngle + elbowAngle - reference[1];
            const rotation = [turnA, turnB - turnA, toolAngle - reference[2] - turnB];
            const angles = rotation.map((turn, i) => sign[i] * wrap(turn) / DEG);
            if (angles.some(angle => Math.abs(angle) > limitDeg + 1e-7)) continue;
            if (checkCollision && collisionAt(angles)) continue;
            const cost = angles.reduce((total, angle) => total + angle ** 2, 0);
            if (cost < bestCost) {
                best = angles;
                bestCost = cost;
            }
        }
        return best;
    }

    function worldPoint(sliceYaw, r, z) {
        const angle = sliceYaw * yaw.axis[2] * DEG;
        const radius = shoulderR + r;
        const x = front[0] * Math.cos(angle) - front[1] * Math.sin(angle);
        const y = front[0] * Math.sin(angle) + front[1] * Math.cos(angle);
        return [yaw.point[0] + x * radius, yaw.point[1] + y * radius, z];
    }

    function solveSpatial(target, pitch, preferredYaw = 0, checkCollision = true) {
        const [x, y, z] = sub(target, [yaw.point[0], yaw.point[1], 0]);
        const radius = Math.hypot(x, y);
        const heading = Math.atan2(front[0] * y - front[1] * x, front[0] * x + front[1] * y);
        const choices = radius < 1e-6 ? [[preferredYaw, 0]] : [
            [heading / (yaw.axis[2] * DEG), radius],
            [wrap(heading + Math.PI) / (yaw.axis[2] * DEG), -radius],
        ];
        choices.sort(([a], [b]) => Math.abs(a - preferredYaw) - Math.abs(b - preferredYaw));
        let nearest = null;
        for (const [sliceYaw, distance] of choices) {
            if (Math.abs(sliceYaw) > limitDeg + 1e-7) continue;
            const point = [distance - shoulderR, z];
            const yawTarget = radius < 1e-6 ? null : sliceYaw;
            const angles = solve(...point, pitch, checkCollision);
            const pose = { sliceYaw, yaw: yawTarget, point, angles };
            if (angles) return pose;
            nearest ??= pose;
        }
        return nearest;
    }

    return { reach, shoulderZ, collision, project, solve, collisionAt, pathCollision,
        shoulderR, baseXY: yaw.point.slice(0, 2), worldPoint, solveSpatial };
}
