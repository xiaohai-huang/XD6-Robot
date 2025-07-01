export class WebSerial {
	private port: SerialPort | null = null;
	private reader: ReadableStreamDefaultReader<string> | null = null;
	private readStreamClosed: Promise<void> | undefined = undefined;

	private writer: WritableStreamDefaultWriter<string> | null = null;
	private writeStreamClosed: Promise<void> | undefined = undefined;

	// private textEncoder: TextEncoderStream | null = null;

	private isReading = false;
	private receiveCallbacks: Set<(data: string) => void> = new Set();

	public isConnected = false;

	private async readLoop(): Promise<void> {
		if (!this.reader) return;

		this.isReading = true;

		try {
			while (this.isReading) {
				const { value, done } = await this.reader.read();
				if (done) break;
				if (value) {
					console.log(
						`%c[${new Date().toLocaleTimeString()}] %cReceived: %c${value}`,
						"color: gray; font-weight: normal;", // timestamp style
						"color: purple; font-weight: bold;", // "Received:" label style
						"color: black;", // value style
					);
					for (const callback of this.receiveCallbacks) {
						callback(value);
					}
				}
			}
		} catch (error) {
			console.error("Read error:", error);
		}
	}

	public async connect(baudRate = 115200): Promise<void> {
		try {
			this.port = await navigator.serial.requestPort();
			await this.port.open({ baudRate, dataBits: 8, stopBits: 1 });
			if (!this.port || !this.port.writable || !this.port.readable) return;
			const textDecoder = new TextDecoderStream();
			const lineTransformer = new TransformStream(new LineBreakTransformer());

			// Pipe raw stream → text decoder → line splitter
			this.readStreamClosed = this.port.readable
				.pipeTo(textDecoder.writable)
				.catch(() => {});

			const lineReadable = textDecoder.readable.pipeThrough(lineTransformer);

			this.reader = lineReadable.getReader();

			// Writer
			const textEncoder = new TextEncoderStream();
			this.writeStreamClosed = textEncoder.readable.pipeTo(this.port.writable);
			this.writer = textEncoder.writable.getWriter();

			this.isConnected = true;

			this.readLoop();
		} catch (error) {
			this.port = null;
			this.isConnected = false;
			throw error;
		}
	}

	private async cleanup(): Promise<void> {
		this.isReading = false;

		try {
			await this.reader?.cancel();
			await this.writer?.close();
		} catch (_) {}

		try {
			await this.readStreamClosed;
			await this.writeStreamClosed;
		} catch (_) {}

		this.reader = null;
		this.writer = null;
		this.readStreamClosed = undefined;
		this.writeStreamClosed = undefined;
		this.isConnected = false;
	}

	public async disconnect(): Promise<void> {
		if (!this.port) return;

		await this.cleanup();

		try {
			await this.port.close();
		} catch (e) {
			console.error("Error closing port:", e);
		}

		this.port = null;
	}

	public async sendCommand(command: string): Promise<void> {
		if (!this.writer) return;

		try {
			await this.writer.write(command + "\n");
		} catch (error) {
			console.error("Write error:", error);
		}
	}

	// Add/remove listeners
	public addReceiveListener(callback: (resp: string) => void): void {
		this.receiveCallbacks.add(callback);
	}

	public removeReceiveListener(callback: (resp: string) => void): void {
		this.receiveCallbacks.delete(callback);
	}

	// Wait for specific line with timeout
	public listenFor(line: string, timeoutSeconds: number): Promise<string> {
		return new Promise((resolve, reject) => {
			const onData = (data: string) => {
				if (data.includes(line)) {
					this.removeReceiveListener(onData);
					clearTimeout(timer);
					resolve(data);
				}
			};

			const timer = setTimeout(() => {
				this.removeReceiveListener(onData);
				reject(
					new Error(
						`Timeout: "${line}" not received within ${timeoutSeconds}s`,
					),
				);
			}, timeoutSeconds * 1000);

			this.addReceiveListener(onData);
		});
	}
}

// 🧱 LineBreakTransformer: splits incoming stream into lines
class LineBreakTransformer implements Transformer<string, string> {
	private buffer = "";

	transform(
		chunk: string,
		controller: TransformStreamDefaultController<string>,
	) {
		this.buffer += chunk;
		const lines = this.buffer.split(/\r?\n/);
		this.buffer = lines.pop() || "";
		for (const line of lines) {
			controller.enqueue(line);
		}
	}

	flush(controller: TransformStreamDefaultController<string>) {
		if (this.buffer) {
			controller.enqueue(this.buffer);
			this.buffer = "";
		}
	}
}
