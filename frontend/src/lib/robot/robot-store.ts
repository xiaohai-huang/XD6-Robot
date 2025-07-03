import { Derived, Store } from "@tanstack/store";
import type { CalibrationStatus } from "@/components/robot/JointControlRow";

// user input angels
export const anglesStore = new Store({
	J1: 0,
	J2: 0,
	J3: 0,
	J4: 0,
	J5: 0,
	J6: 0,
});

export const anglesArray = new Derived({
	fn: () =>
		[
			anglesStore.state.J1,
			anglesStore.state.J2,
			anglesStore.state.J3,
			anglesStore.state.J4,
			anglesStore.state.J5,
			anglesStore.state.J6,
		] as const,
	deps: [anglesStore],
});

anglesArray.mount();

type CalibrationStatusType = {
	J1: CalibrationStatus;
	J2: CalibrationStatus;
	J3: CalibrationStatus;
	J4: CalibrationStatus;
	J5: CalibrationStatus;
	J6: CalibrationStatus;
};

export const calibrationStatusStore = new Store({
	J1: "no",
	J2: "no",
	J3: "no",
	J4: "no",
	J5: "no",
	J6: "no",
} as CalibrationStatusType);
