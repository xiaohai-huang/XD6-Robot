import { type ClassValue, clsx } from "clsx";
import { twMerge } from "tailwind-merge";

export function cn(...inputs: ClassValue[]) {
	return twMerge(clsx(inputs));
}

export function runTasksWithStaggeredStart<T>(
	tasks: (() => Promise<T>)[],
	delayMs: number,
): Promise<T[]> {
	const startDelayed = tasks.map((task, index) => {
		return new Promise<T>((resolve) => {
			setTimeout(() => {
				task().then(resolve);
			}, index * delayMs);
		});
	});

	return Promise.all(startDelayed);
}
