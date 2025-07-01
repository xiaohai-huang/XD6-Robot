import {
	CheckCircle,
	Hammer,
	Loader2,
	Minus,
	Plus,
	RefreshCw,
	XCircle,
} from "lucide-react";
import { useEffect, useState } from "react";
import { z } from "zod"; // Import zod
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import {
	Tooltip,
	TooltipContent,
	TooltipTrigger,
} from "@/components/ui/tooltip";
import type { JointNum } from "@/lib/robot/robotic-arm";
import { cn } from "@/lib/utils";
import { arm } from "./RoboticArmUI";

export type CalibrationStatus = "no" | "done" | "in-progress";

export type JointControlRowProps = {
	num: JointNum;
	range: [number, number];
	currentDegree: number;
	calibrationStatus: CalibrationStatus;
};

// Zod schema for the degree input
const degreeSchema = (min: number, max: number) =>
	z.preprocess(
		(val) => {
			if (typeof val === "string" && val.trim() === "") {
				return undefined; // Treat empty string as undefined for optionality
			}
			return Number(val);
		},
		z
			.number({
				invalid_type_error: "Must be a number",
			})
			.min(min, `Too low (min: ${min}°)`)
			.max(max, `Too high (max: ${max}°)`)
			.optional(), // Allow empty input
	);

export default function JointControlRow({
	num,
	range,
	currentDegree = 0,
	calibrationStatus,
}: JointControlRowProps) {
	const [degreeInput, setDegreeInput] = useState<string>(
		currentDegree.toString(),
	);
	const [degree, setDegree] = useState<number>(currentDegree);
	const [step, setStep] = useState<number>(1);
	const [isDirty, setIsDirty] = useState<boolean>(false);
	const [error, setError] = useState<string | null>(null); // State for error message
	const [isExecuting, setIsExecuting] = useState<boolean>(false); // State for execution in progress

	const calibrated = calibrationStatus === "done";
	const isDisabled = !calibrated || isExecuting; // Consolidated disabled check

	useEffect(() => {
		setDegree(currentDegree);
		setDegreeInput(currentDegree.toString());
		setIsDirty(false);
		setError(null);
	}, [currentDegree]);

	const handleReset = () => {
		setDegree(currentDegree);
		setDegreeInput(currentDegree.toString());
		setIsDirty(false);
		setError(null);
	};

	const handleDegreeInputChange = (e: React.ChangeEvent<HTMLInputElement>) => {
		const val = e.currentTarget.value;
		setDegreeInput(val);

		const schema = degreeSchema(range[0], range[1]);
		const result = schema.safeParse(val);

		if (!result.success) {
			setError(result.error.issues[0].message);
			setIsDirty(true);
			setDegree(Number.NaN);
		} else {
			setError(null);
			if (result.data !== undefined) {
				setDegree(result.data);
				setIsDirty(result.data !== currentDegree);
			} else {
				setDegree(currentDegree);
				setIsDirty(val.trim() !== currentDegree.toString());
			}
		}
	};

	const handleDegreeInputKeyDown = (
		e: React.KeyboardEvent<HTMLInputElement>,
	) => {
		if (e.key === "Enter" && isDirty && !error && !isDisabled) {
			handleExecute();
		}
	};

	const handleJog = async (delta: number) => {
		const target = degree + delta;
		const schema = degreeSchema(range[0], range[1]);
		const result = schema.safeParse(target);

		if (!result.success) {
			setError(`Jog failed: ${result.error.issues[0].message}`);
			return;
		}

		setError(null);
		setIsExecuting(true); // Disable buttons immediately
		try {
			await arm.rotateBy(num, delta); // Immediate execution
			await arm.getDegrees();
		} finally {
			setIsExecuting(false); // Re-enable buttons after execution
		}
	};

	const handleExecute = async () => {
		const schema = degreeSchema(range[0], range[1]);
		const result = schema.safeParse(degreeInput); // Validate degreeInput again before execution

		if (!result.success) {
			setError(result.error.issues[0].message);
			return;
		}

		if (result.data === undefined) {
			setError("Input cannot be empty for execution.");
			return;
		}

		setError(null);
		setIsExecuting(true); // Disable inputs and buttons
		try {
			const clamped = result.data; // Zod has already validated and preprocessed
			await arm.rotateTo(num, clamped);
			setIsDirty(false);
			await arm.getDegrees();
		} finally {
			setIsExecuting(false); // Re-enable inputs and buttons
		}
	};

	const handleCalibrate = () => {
		arm.calibrateJoint(num);
	};

	const handleStop = () => {
		arm.stopJoint(num);
	};

	return (
		<div
			className={cn(
				"grid grid-cols-5 items-center gap-4 p-4 bg-white rounded-2xl shadow transition-all duration-200",
				error && "border-2 border-red-500", // Red border on error
			)}
		>
			{/* Joint label */}
			<div className="flex flex-col items-center">
				<span className="text-xs mb-1">&nbsp;</span>
				<div className="col-span-1 font-medium">Joint {num}</div>
			</div>
			{/* Jog controls & current degree */}
			<div className="col-span-2 flex items-center space-x-2">
				<div className="flex flex-col items-center">
					<span className="text-xs text-gray-500 mb-1">{range[0]}°</span>
					<Button
						variant="outline"
						size="sm"
						onClick={() => handleJog(-1 * step)}
						disabled={isDisabled || error !== null} // Disable jog if error or executing
					>
						<Minus size={16} />
					</Button>
				</div>
				<div className="flex flex-col items-center">
					<span className="text-xs text-gray-500 mb-1">&nbsp;</span>

					<Input
						type="text" // Use type="text" to allow empty string and handle parsing manually
						inputMode="decimal" // Hint for mobile keyboards
						className={cn(
							"text-center [&::-webkit-inner-spin-button]:appearance-none [&::-webkit-outer-spin-button]:appearance-none",
							isDirty && "border-yellow-500 ring-1 ring-yellow-500",
							error && "border-red-500 ring-1 ring-red-500", // Red border for error state
						)}
						value={degreeInput}
						onChange={handleDegreeInputChange}
						onKeyDown={handleDegreeInputKeyDown} // Add onKeyDown handler
						disabled={isDisabled}
					/>
					{error && (
						<p className="text-red-500 text-xs flex items-center mt-1">
							<XCircle size={14} className="mr-1" /> {error}
						</p>
					)}
				</div>
				<div className="flex flex-col items-center">
					<span className="text-xs text-gray-500 mb-1">{range[1]}°</span>
					<Button
						variant="outline"
						size="sm"
						onClick={() => handleJog(1 * step)}
						disabled={isDisabled || error !== null} // Disable jog if error or executing
					>
						<Plus size={16} />
					</Button>
				</div>

				{isDirty &&
					!error && ( // Only show execute/reset if dirty AND no error
						<div className="flex flex-col items-center">
							<span className="text-xs mb-1">&nbsp;</span>

							<div className="flex items-center gap-0.5">
								<Button
									variant="outline"
									size="sm"
									onClick={handleExecute}
									disabled={isDisabled}
								>
									Execute
								</Button>
								<Button
									variant="ghost"
									size="sm"
									onClick={handleReset}
									disabled={isDisabled}
								>
									<RefreshCw size={16} />
								</Button>
							</div>
						</div>
					)}
			</div>

			{/* Step input */}
			<div className="col-span-1 flex flex-col items-center">
				<label
					htmlFor={`step-input-${num}`}
					className="text-xs text-gray-500 mb-1"
				>
					Step Size
				</label>

				<Input
					id={`step-input-${num}`} // Add id for label association
					type="number"
					className="w-16 text-center [&::-webkit-inner-spin-button]:appearance-none [&::-webkit-outer-spin-button]:appearance-none"
					value={step}
					min={1}
					onChange={(e) => setStep(Math.max(1, Number(e.currentTarget.value)))}
					disabled={isDisabled}
				/>
			</div>

			{/* Calibrate, Stop, and Status */}
			<div className="col-span-1 flex items-center space-x-2">
				<Tooltip>
					<TooltipTrigger asChild>
						<div className="flex flex-col items-center">
							<span className="text-xs mb-1">&nbsp;</span>

							<Button
								variant="outline"
								size="sm"
								onClick={handleCalibrate}
								disabled={calibrationStatus === "in-progress" || isExecuting}
							>
								{calibrationStatus === "in-progress" ? (
									<Loader2 className="animate-spin text-blue-500" size={16} />
								) : calibrationStatus === "done" ? (
									<CheckCircle className="text-green-500" size={16} />
								) : (
									<Hammer size={16} />
								)}
							</Button>
						</div>
					</TooltipTrigger>
					<TooltipContent>
						<p>
							{calibrationStatus === "in-progress"
								? "Calibrating..."
								: calibrationStatus === "done"
									? "Calibrated"
									: "Calibrate"}
						</p>
					</TooltipContent>
				</Tooltip>

				<Tooltip>
					<TooltipTrigger asChild>
						<div className="flex flex-col items-center">
							<span className="text-xs mb-1">&nbsp;</span>
							<Button variant="outline" size="sm" onClick={handleStop}>
								<XCircle className="text-red-500" size={16} />
							</Button>
						</div>
					</TooltipTrigger>
					<TooltipContent>
						<p>Stop</p>
					</TooltipContent>
				</Tooltip>
			</div>
		</div>
	);
}
