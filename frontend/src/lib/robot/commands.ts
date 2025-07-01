const COMMANDS = {
	ECHO: "00",
	S: "01",
	STOP_JOINT: "02",
	MOVE_JOINTS: "03",
	CALIBRATE_JOINTS: "04",
	PRINT_POS: "05",
	PRINT_CALIBRATION_STATUS: "06",
	ADD: "07",
	MOVE_JOINT: "08",
	MOVE_JOINT_BY: "09",
} as const;

export type Command = keyof typeof COMMANDS;

export default COMMANDS;
