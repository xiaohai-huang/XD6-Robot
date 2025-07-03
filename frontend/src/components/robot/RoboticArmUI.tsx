import { useStore } from "@tanstack/react-store";
import { useEffect, useState } from "react";
import { useHotkeys } from "react-hotkeys-hook";
import { Button } from "@/components/ui/button";
import { Card } from "@/components/ui/card";
import { JOINT_CONFIGS } from "@/lib/robot/config";
import { anglesArray } from "@/lib/robot/robot-store";
import RoboticArm, {
	type CalibrationStatus,
	type JointNum,
} from "@/lib/robot/robotic-arm";
import JointControlRow from "./JointControlRow";

export const arm = RoboticArm.create();

export default function RoboticArmUI() {
	const [connected, setConnected] = useState(false);
	const [angles, setAngles] = useState<number[]>([0, 0, 0, 0, 0, 0]);
	const inputs = useStore(anglesArray);
	const [status, setStatus] = useState<CalibrationStatus[]>([
		"no",
		"no",
		"no",
		"no",
		"no",
		"no",
	]);

	const refreshAngles = async () => {
		const deg = await arm.getDegrees();
		setAngles(deg);
	};

	const connect = async () => {
		await arm.connect();
		setConnected(true);
		refreshAngles();
	};

	const disconnect = async () => {
		await arm.disconnect();
		setConnected(false);
	};

	const rotateAll = async () => {
		const success = await arm.rotateAllTo(...inputs);
		await refreshAngles();
	};

	const calibrateAll = async () => {
		const success = await arm.calibrateAll();
	};

	const stopAll = async () => {
		await arm.stopAllJoint();
	};

	useEffect(() => {
		return arm.addEventListener("calibrationStatusChanged", (statusArray) => {
			setStatus(statusArray);
		});
	}, []);

	useEffect(() => {
		return arm.addEventListener("degreesChanged", (degrees) => {
			setAngles(degrees);
		});
	}, []);

	useHotkeys("s", () => {
		arm.stopAllJoint();
	});
	return (
		<div className="max-w-5xl mx-auto mt-10">
			<Card className="p-6 shadow-xl rounded-2xl space-y-6">
				<h2 className="text-2xl font-bold text-center">
					Robotic Arm Controller
				</h2>

				<div className="flex justify-center space-x-4">
					<Button onClick={connect} disabled={connected}>
						Connect
					</Button>
					<Button onClick={disconnect} disabled={!connected}>
						Disconnect
					</Button>
					<Button onClick={stopAll} variant="destructive">
						Stop All
					</Button>
				</div>

				<div>
					{Object.values(JOINT_CONFIGS).map((config, jointIndex) => (
						<JointControlRow
							key={config.NAME}
							num={(jointIndex + 1) as JointNum}
							currentDegree={angles[jointIndex]}
							range={config.RANGE}
							calibrationStatus={status[jointIndex]}
						/>
					))}
				</div>

				<div className="flex justify-center gap-4">
					<Button onClick={rotateAll}>Move All</Button>
					<Button onClick={refreshAngles}>Refresh Angles</Button>
					<Button onClick={calibrateAll}>Calibrate All</Button>
				</div>

				{status.length > 0 && (
					<p className="text-sm mt-2 text-muted-foreground">
						Calibration: {JSON.stringify(status)}
					</p>
				)}
			</Card>
		</div>
	);
}
