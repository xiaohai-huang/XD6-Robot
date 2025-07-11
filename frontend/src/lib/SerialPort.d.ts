/*~ https://wicg.github.io/serial/#dom-paritytype */
type ParityType = "none" | "even" | "odd";

/*~ https://wicg.github.io/serial/#dom-flowcontroltype */
type FlowControlType = "none" | "hardware";

/*~ https://wicg.github.io/serial/#dom-serialoptions */
interface SerialOptions {
    baudRate: number;
    dataBits?: 7 | 8 | undefined;
    stopBits?: 1 | 2 | undefined;
    parity?: ParityType | undefined;
    bufferSize?: number | undefined;
    flowControl?: FlowControlType | undefined;
}

/*~ https://wicg.github.io/serial/#dom-serialoutputsignals */
interface SerialOutputSignals {
    dataTerminalReady?: boolean | undefined;
    requestToSend?: boolean | undefined;
    break?: boolean | undefined;
}

/*~ https://wicg.github.io/serial/#dom-serialinputsignals */
interface SerialInputSignals {
    dataCarrierDetect: boolean;
    clearToSend: boolean;
    ringIndicator: boolean;
    dataSetReady: boolean;
}

/*~ https://wicg.github.io/serial/#serialportinfo-dictionary */
interface SerialPortInfo {
    usbVendorId?: number | undefined;
    usbProductId?: number | undefined;
    /** If the port is a service on a Bluetooth device this member will be a BluetoothServiceUUID
     * containing the service class UUID. Otherwise it will be undefined. */
    bluetoothServiceClassId?: number | string | undefined;
}

/*~ https://wicg.github.io/serial/#dom-serialport */
declare class SerialPort extends EventTarget {
    onconnect: ((this: this, ev: Event) => void) | null;
    ondisconnect: ((this: this, ev: Event) => void) | null;
    /** A flag indicating the logical connection state of serial port */
    readonly connected: boolean;
    readonly readable: ReadableStream<Uint8Array> | null;
    readonly writable: WritableStream<Uint8Array> | null;

    open(options: SerialOptions): Promise<void>;
    setSignals(signals: SerialOutputSignals): Promise<void>;
    getSignals(): Promise<SerialInputSignals>;
    getInfo(): SerialPortInfo;
    close(): Promise<void>;
    forget(): Promise<void>;

    addEventListener(
        type: "connect" | "disconnect",
        listener: (this: this, ev: Event) => void,
        useCapture?: boolean,
    ): void;
    addEventListener(
        type: string,
        listener: EventListenerOrEventListenerObject | null,
        options?: boolean | AddEventListenerOptions,
    ): void;
    removeEventListener(
        type: "connect" | "disconnect",
        callback: (this: this, ev: Event) => void,
        useCapture?: boolean,
    ): void;
    removeEventListener(
        type: string,
        callback: EventListenerOrEventListenerObject | null,
        options?: EventListenerOptions | boolean,
    ): void;
}

/*~ https://wicg.github.io/serial/#dom-serialportfilter */
interface SerialPortFilter {
    usbVendorId?: number | undefined;
    usbProductId?: number | undefined;
    bluetoothServiceClassId?: number | string | undefined;
}

/*~ https://wicg.github.io/serial/#dom-serialportrequestoptions */
interface SerialPortRequestOptions {
    filters?: SerialPortFilter[] | undefined;
    /** A list of BluetoothServiceUUID values representing Bluetooth service class IDs.
     * Bluetooth ports with custom service class IDs are excluded from the list of ports
     * presented to the user unless the service class ID is included in this list.
     *
     * {@link https://wicg.github.io/serial/#serialportrequestoptions-dictionary} */
    allowedBluetoothServiceClassIds?: Array<number | string> | undefined;
}

/*~ https://wicg.github.io/serial/#dom-serial */
declare class Serial extends EventTarget {
    onconnect: ((this: this, ev: Event) => void) | null;
    ondisconnect: ((this: this, ev: Event) => void) | null;

    getPorts(): Promise<SerialPort[]>;
    requestPort(options?: SerialPortRequestOptions): Promise<SerialPort>;
    addEventListener(
        type: "connect" | "disconnect",
        listener: (this: this, ev: Event) => void,
        useCapture?: boolean,
    ): void;
    addEventListener(
        type: string,
        listener: EventListenerOrEventListenerObject | null,
        options?: boolean | AddEventListenerOptions,
    ): void;
    removeEventListener(
        type: "connect" | "disconnect",
        callback: (this: this, ev: Event) => void,
        useCapture?: boolean,
    ): void;
    removeEventListener(
        type: string,
        callback: EventListenerOrEventListenerObject | null,
        options?: EventListenerOptions | boolean,
    ): void;
}

/*~ https://wicg.github.io/serial/#extensions-to-the-navigator-interface */
interface Navigator {
    readonly serial: Serial;
}

/*~ https://wicg.github.io/serial/#extensions-to-workernavigator-interface */
interface WorkerNavigator {
    readonly serial: Serial;
}