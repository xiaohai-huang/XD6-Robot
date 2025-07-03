import COMMANDS, { type Command } from "./commands";
import { JOINT_CONFIGS } from "./config";
import { WebSerial } from "./web-serial";

export type JointNum = 1 | 2 | 3 | 4 | 5 | 6;
export type CalibrationStatus = "no" | "done" | "in-progress";

const CALIBRATION_NUM_TO_STATUS = {
	"0": "no",
	"1": "in-progress",
	"2": "done",
} as const;

type RoboticArmEventMap = {
	calibrationStatusChanged: [
		CalibrationStatus,
		CalibrationStatus,
		CalibrationStatus,
		CalibrationStatus,
		CalibrationStatus,
		CalibrationStatus,
	];
	degreesChanged: [number, number, number, number, number, number];
};

interface Arm {
	rotateBy(jointNum: JointNum, degree: number): Promise<boolean>;
	rotateTo(jointNum: JointNum, targetDegree: number): Promise<boolean>;
	rotateAllTo(
		joint1Degree: number,
		joint2Degree: number,
		joint3Degree: number,
		joint4Degree: number,
		joint5Degree: number,
		joint6Degree: number,
	): Promise<boolean>;
	stopJoint(jointNum: JointNum): Promise<void>;
	stopAllJoint(): Promise<void>;
	calibrateJoint(jointNum: JointNum): Promise<boolean>;
	calibrateAll(): Promise<boolean>;
	getDegrees(): Promise<number[]>;
	getCalibrationStatus(): Promise<CalibrationStatus[]>;

	addEventListener<K extends keyof RoboticArmEventMap>(
		event: K,
		listener: (data: RoboticArmEventMap[K]) => void,
	): void;
	removeEventListener<K extends keyof RoboticArmEventMap>(
		event: K,
		listener: (data: RoboticArmEventMap[K]) => void,
	): void;
}

export default class RoboticArm implements Arm {
	private _serial: WebSerial;
	private _moveDuration: number;
	private _acceleration: number;

	private _eventListeners: Map<keyof RoboticArmEventMap, Set<Function>> =
		new Map();

	constructor(serial: WebSerial) {
		this._serial = serial;
		this._moveDuration = 2;
		this._acceleration = 0.4;
	}

	static create(): RoboticArm {
		const serial = new WebSerial();
		const arm = new RoboticArm(serial);
		return arm;
	}

	async connect() {
		await this._serial.connect();
		return new Promise((resolve) => setTimeout(resolve, 2000));
	}

	disconnect() {
		return this._serial.disconnect();
	}

	addEventListener<K extends keyof RoboticArmEventMap>(
		event: K,
		listener: (data: RoboticArmEventMap[K]) => void,
	) {
		if (!this._eventListeners.has(event)) {
			this._eventListeners.set(event, new Set());
		}
		this._eventListeners.get(event)!.add(listener as Function);
		return () => this.removeEventListener(event, listener);
	}

	removeEventListener<K extends keyof RoboticArmEventMap>(
		event: K,
		listener: (data: RoboticArmEventMap[K]) => void,
	) {
		this._eventListeners.get(event)?.delete(listener as Function);
	}

	private _emit<K extends keyof RoboticArmEventMap>(
		event: K,
		data: RoboticArmEventMap[K],
	) {
		this._eventListeners.get(event)?.forEach((listener) => {
			(listener as (data: RoboticArmEventMap[K]) => void)(data);
		});
	}

	sendCommand(command: Command, ...args: (string | number)[]): Promise<void> {
		console.log(
			`%c[${new Date().toLocaleTimeString()}] %cSent: %c${command} %c${args.join(", ")}`,
			"color: gray; font-weight: normal;", // timestamp style
			"color: blue; font-weight: bold;", // "Sent:" label style
			"color: black;", // command style
			"color: darkgreen;", // args style
		);

		return this._serial.sendCommand(
			args ? `${COMMANDS[command]} ${args.join(",")}` : `${command}`,
		);
	}

	async calibrateJoint(jointNum: JointNum): Promise<boolean> {
		await this.sendCommand("CALIBRATE_JOINTS", jointNum);
		await this.getCalibrationStatus();
		try {
			await this._serial.listenFor(
				`Calibration complete for Joint ${jointNum}`,
				90,
			);
		} catch (e: any) {
			console.error(e.message);
			return false;
		}
		await this.getCalibrationStatus();
		return true;
	}

	async calibrateAll(): Promise<boolean> {
		// calibrate first 3 joints then the rest
		const results = await Promise.all(
			[1, 2, 3].map((jointNum) => this.calibrateJoint(jointNum as JointNum)),
		);
		if (results.includes(false)) return false;
		const upperResults = await Promise.all(
			[4, 5, 6].map((jointNum) => this.calibrateJoint(jointNum as JointNum)),
		);
		return !upperResults.includes(false);
	}

	async getCalibrationStatus(): Promise<CalibrationStatus[]> {
		await this.sendCommand("PRINT_CALIBRATION_STATUS");
		const line = await this._serial.listenFor("CALIBRATION STATUS", 1);
		const statusString = line.split(":").pop()?.trim() || "";
		const statusArray = statusString
			.replace(/\[|\]/g, "")
			.split(",")
			.map((num) => CALIBRATION_NUM_TO_STATUS[num as "0" | "1" | "2"]);

		// Emit only if the array is the correct length
		if (statusArray.length === 6) {
			this._emit(
				"calibrationStatusChanged",
				statusArray as RoboticArmEventMap["calibrationStatusChanged"],
			);
		}
		return statusArray;
	}

	async getDegrees(): Promise<number[]> {
		await this.sendCommand("PRINT_POS");
		const line = await this._serial.listenFor("CURRENT POSITIONS", 1);
		const positionsString = line.split(":").pop()?.trim() || "";
		const positionsInDegrees = positionsString
			.replace(/\[|\]/g, "")
			.split(",")
			.map(Number)
			.map((steps, i) => RoboticArm.stepsToDegrees((i + 1) as JointNum, steps));

		if (positionsInDegrees.length === 6) {
			this._emit(
				"degreesChanged",
				positionsInDegrees as RoboticArmEventMap["degreesChanged"],
			);
		}
		return positionsInDegrees;
	}

	async stopJoint(jointNum: JointNum): Promise<void> {
		await this.sendCommand("STOP_JOINT", jointNum);
		await this._serial.listenFor(`STOP_J ${jointNum}`, 1);
		await this.getCalibrationStatus();
	}

	async stopAllJoint(): Promise<void> {
		await this.sendCommand("S");
		await this._serial.listenFor("All motors stopped", 2);
		await this.getCalibrationStatus();
	}

	async rotateAllTo(
		joint1Degree: number,
		joint2Degree: number,
		joint3Degree: number,
		joint4Degree: number,
		joint5Degree: number,
		joint6Degree: number,
	): Promise<boolean> {
		// MOVE_JOINTS j1_degree,j2_degree,j3_degree,j4_degree,j5_degree,j6_degree,duration_sec,accel_decel_percent
		await this.sendCommand(
			"MOVE_JOINTS",
			joint1Degree,
			joint2Degree,
			joint3Degree,
			joint4Degree,
			joint5Degree,
			joint6Degree,
			this._moveDuration,
			this._acceleration,
		);
		let success = true;
		await this._serial
			.listenFor("MOVE_JOINTS COMPLETE", this._moveDuration + 5)
			.catch(() => {
				success = false;
			});
		return success;
	}

	async rotateTo(jointNum: JointNum, targetDegree: number): Promise<boolean> {
		// MOVE_JOINT joint_num,targetDegrees
		let success = true;

		await this.sendCommand(
			"MOVE_JOINT",
			jointNum,
			targetDegree,
			this._moveDuration,
			this._acceleration,
		);
		await this._serial
			.listenFor(`MOVE_JOINT ${jointNum} COMPLETE`, 20)
			.catch(() => {
				success = false;
			});
		return success;
	}

	async rotateBy(jointNum: JointNum, degree: number): Promise<boolean> {
		// MOVE_JOINT_BY joint_num, degree

		let success = true;

		await this.sendCommand(
			"MOVE_JOINT_BY",
			jointNum,
			degree,
			this._moveDuration,
			this._acceleration,
		);
		await this._serial
			.listenFor(`MOVE_JOINT_BY ${jointNum} COMPLETE`, 20)
			.catch(() => {
				success = false;
			});
		return success;
	}

	static stepsToDegrees(jointNum: JointNum, steps: number): number {
		const { STEPS_PER_REV } = JOINT_CONFIGS[`J${jointNum}`];
		return (steps / STEPS_PER_REV) * 360;
	}
}
