import { createFileRoute } from "@tanstack/react-router";
import RoboticArmUI from "@/components/robot/RoboticArmUI";

export const Route = createFileRoute("/")({
	component: App,
});

function App() {
	return <RoboticArmUI />;
}
