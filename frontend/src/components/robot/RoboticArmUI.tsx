import { useEffect, useState } from "react";
import { useHotkeys } from "react-hotkeys-hook";
import { Button } from "@/components/ui/button";
import { Card } from "@/components/ui/card";
import { JOINT_CONFIGS } from "@/lib/robot/config";
import RoboticArm, {
	type CalibrationStatus,
	type JointNum,
} from "@/lib/robot/robotic-arm";
import JointControlRow from "./JointControlRow";

export const arm = RoboticArm.create();

export default function RoboticArmUI() {
	const [connected, setConnected] = useState(false);
	const [angles, setAngles] = useState<number[]>([0, 0, 0, 0, 0, 0]);
	const [inputs, setInputs] = useState([0, 0, 0, 0, 0, 0] as const);
	const [status, setStatus] = useState<CalibrationStatus[]>([
		"no",
		"no",
		"no",
		"no",
		"no",
		"no",
	]);
	const [log, setLog] = useState<string[]>([]);

	const logMessage = (msg: string) => {
		setLog((prev) => [`[${new Date().toLocaleTimeString()}] ${msg}`, ...prev]);
	};

	const refreshAngles = async () => {
		const deg = await arm.getDegrees();
		setAngles(deg);
		logMessage("Refreshed joint angles.");
	};

	const connect = async () => {
		await arm.connect();
		setConnected(true);
		logMessage("Connected to robotic arm.");
		refreshAngles();
	};

	const disconnect = async () => {
		await arm.disconnect();
		setConnected(false);
		logMessage("Disconnected.");
	};

	const rotateAll = async () => {
		const success = await arm.rotateAllTo(...inputs);
		logMessage(`Rotate All Joints: ${success ? "Success" : "Failed"}`);
		await refreshAngles();
	};

	const calibrateAll = async () => {
		const success = await arm.calibrateAll();
		logMessage(`Calibrate All Joints: ${success ? "Success" : "Failed"}`);
	};

	const stopAll = async () => {
		await arm.stopAllJoint();
		logMessage("All joints stopped.");
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
