import { CMD, FrameDecoder, MSG, VERSION, encodeFrame } from './protocol.mjs?v=axes-1';

const TIMEOUT_MS = 300;

export class ArmSerial {
    #port = null;
    #reader = null;
    #writer = null;
    #reading = null;
    #writing = null;
    #pending = null;
    #seq = 0;
    #decoder = new FrameDecoder();
    #closed = false;

    constructor(onLinkLost) { this.onLinkLost = onLinkLost; }

    async open(port) {
        await port.open({ baudRate: 115200, dataBits: 8, stopBits: 1, parity: 'none', flowControl: 'none', bufferSize: 4096 });
        this.#port = port;
        this.#closed = false;
        try {
            this.#reader = port.readable.getReader();
            this.#writer = port.writable.getWriter();
            this.#reading = this.#read();
            const probe = new Uint8Array([0x56, 0x4c, 0x41, VERSION]);
            const answer = await this.request(CMD.ping, MSG.pong, probe);
            if (answer.length !== probe.length || !answer.every((byte, i) => byte === probe[i])) {
                throw new Error('握手回显不一致');
            }
        } catch (error) {
            await this.close();
            throw error;
        }
    }

    async request(type, expected, payload) {
        if (this.#closed || !this.#writer) throw new Error('串口未连接');
        if (this.#pending || this.#writing) throw new Error('上一条请求尚未完成');
        this.#seq = (this.#seq + 1) & 0xffff;
        const seq = this.#seq;
        const bytes = encodeFrame(type, seq, payload);
        const reply = new Promise((resolve, reject) => {
            const timer = setTimeout(() => {
                this.#decoder.reset();
                this.#finish(new Error('等待板端应答超时'));
            }, TIMEOUT_MS);
            this.#pending = { seq, expected, resolve, reject, timer };
        });
        this.#writing = this.#writer.write(bytes).catch(error => {
            if (this.#pending?.seq === seq) this.#finish(error);
            if (!this.#closed) this.onLinkLost(error);
            throw error;
        }).finally(() => { this.#writing = null; });
        const [answer] = await Promise.all([reply, this.#writing]);
        return answer;
    }

    #finish(error, payload) {
        const request = this.#pending;
        if (!request) return;
        this.#pending = null;
        clearTimeout(request.timer);
        if (error) request.reject(error);
        else request.resolve(payload);
    }

    async #read() {
        let ended;
        try {
            for (;;) {
                const { value, done } = await this.#reader.read();
                if (done) break;
                for (const frame of this.#decoder.push(value)) {
                    const request = this.#pending;
                    if (request?.seq === frame.seq && request.expected === frame.type) {
                        this.#finish(null, frame.payload);
                    }
                }
            }
        } catch (error) {
            ended = error;
        } finally {
            this.#reader.releaseLock();
            this.#reader = null;
            if (!this.#closed) {
                const error = ended ?? new Error('串口已断开');
                this.#finish(error);
                this.onLinkLost(error);
            }
        }
    }

    async close() {
        if (this.#closed) return;
        this.#closed = true;
        this.#finish(new Error('连接已关闭'));
        await this.#reader?.cancel().catch(() => {});
        await this.#reading;
        if (this.#writing) await this.#writer.abort().catch(() => {});
        try { this.#writer?.releaseLock(); } catch { /* 设备已移除 */ }
        this.#writer = null;
        await this.#port?.close().catch(() => {});
        this.#port = null;
        this.#decoder.reset();
    }
}
