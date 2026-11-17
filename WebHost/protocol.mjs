export const AXES = Object.freeze(['J1', 'J2', 'J3', 'J4', 'J5', 'J6']);
export const VERSION = 2;
export const CMD = Object.freeze({ targets: 1, enable: 2, feedback: 3, ping: 4 });
export const MSG = Object.freeze({ ack: 0x80, feedback: 0x81, pong: 0x84 });
const MAX_PAYLOAD = 512;
const HEADER_SIZE = 8;
const FRAME_OVERHEAD = 10;

export function crc16(bytes) {
    let crc = 0xffff;
    for (const byte of bytes) {
        crc ^= byte << 8;
        for (let bit = 0; bit < 8; bit++) {
            crc = ((crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0)) & 0xffff;
        }
    }
    return crc;
}

export function encodeFrame(type, seq, payload = new Uint8Array()) {
    if (payload.length > MAX_PAYLOAD) throw new Error('载荷超过 512 字节');
    const bytes = new Uint8Array(FRAME_OVERHEAD + payload.length);
    const view = new DataView(bytes.buffer);
    bytes.set([0xaa, 0x55, VERSION, type]);
    view.setUint16(4, seq, true);
    view.setUint16(6, payload.length, true);
    bytes.set(payload, HEADER_SIZE);
    const end = bytes.length - 2;
    view.setUint16(end, crc16(bytes.subarray(2, end)), true);
    return bytes;
}

// 从字节流找同步字，坏帧从下一个字节继续找帧头
export class FrameDecoder {
    buffer = new Uint8Array();

    reset() { this.buffer = new Uint8Array(); }

    push(chunk) {
        const bytes = new Uint8Array(this.buffer.length + chunk.length);
        bytes.set(this.buffer);
        bytes.set(chunk, this.buffer.length);
        const view = new DataView(bytes.buffer);
        const frames = [];
        let offset = 0;
        while (offset < bytes.length) {
            if (bytes[offset] !== 0xaa) { offset++; continue; }
            if (offset + 1 === bytes.length) break;
            if (bytes[offset + 1] !== 0x55) { offset++; continue; }
            if (bytes.length - offset < HEADER_SIZE) break;
            const length = view.getUint16(offset + 6, true);
            if (bytes[offset + 2] !== VERSION || length > MAX_PAYLOAD) { offset++; continue; }
            const end = offset + HEADER_SIZE + length;
            if (end + 2 > bytes.length) break;
            if (view.getUint16(end, true) !== crc16(bytes.subarray(offset + 2, end))) {
                offset++;
                continue;
            }
            frames.push({ type: bytes[offset + 3], seq: view.getUint16(offset + 4, true),
                payload: bytes.slice(offset + HEADER_SIZE, end) });
            offset = end + 2;
        }
        this.buffer = bytes.slice(offset);
        return frames;
    }
}

export function targetPayload(axis, degrees) {
    return targetsPayload([[axis, degrees]]);
}

export function targetsPayload(targets) {
    const payload = new Uint8Array(1 + 2 * AXES.length);
    const view = new DataView(payload.buffer);
    for (const [axis, degrees] of targets) {
        if (!Number.isInteger(axis) || axis < 0 || axis >= AXES.length) throw new Error('无效的关节');
        if (!Number.isFinite(degrees) || degrees < 0 || degrees > 270) throw new Error('角度须在 0～270° 内');
        payload[0] |= 1 << axis;
        view.setUint16(1 + 2 * axis, Math.round(degrees * 100), true);
    }
    return payload;
}

export function decodeFeedback(payload) {
    if (payload.length !== 2 + 6 * AXES.length) throw new Error('反馈长度不符');
    const view = new DataView(payload.buffer, payload.byteOffset, payload.byteLength);
    return AXES.map((port, index) => {
        const offset = 2 + index * 6;
        const bit = 1 << index;
        return { port, enabled: Boolean(payload[0] & bit), sampled: Boolean(payload[1] & bit),
            targetDeg: view.getUint16(offset, true) / 100,
            feedbackDeg: view.getUint16(offset + 2, true) / 100,
            adc: view.getUint16(offset + 4, true) };
    });
}

export function checkAck(payload, command) {
    if (payload.length !== 2 || payload[0] !== command) throw new Error('ACK 格式不符');
    const status = payload[1];
    if (status) throw new Error(['', '载荷长度错误', '参数越界', '未知命令'][status] ?? `状态码 ${status}`);
}
