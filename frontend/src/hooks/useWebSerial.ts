import { useEffect, useRef, useState } from "react";
import { WebSerial } from "@/lib/robot/web-serial";

export function useWebSerial() {
	const webSerialRef = useRef<WebSerial | null>(null);
	const [isConnected, setIsConnected] = useState(false);
	const [latestMessage, setLatestMessage] = useState<string | null>(null);

	useEffect(() => {
		const ws = new WebSerial();
		webSerialRef.current = ws;

		ws.onReceive((message: string) => {
			console.log("got " + message);
			setLatestMessage(message);
		});

		return () => {
			ws.disconnect();
		};
	}, []);

	const connect = async () => {
		if (!webSerialRef.current) return;
		try {
			await webSerialRef.current.connect();
			setIsConnected(webSerialRef.current.isConnected);
		} catch (e) {
			console.error("Connection failed", e);
			setIsConnected(false);
		}
	};

	const disconnect = async () => {
		await webSerialRef.current?.disconnect();
		setIsConnected(false);
	};

	const sendCommand = async (command: string) => {
		await webSerialRef.current?.sendCommand(command);
	};

	return {
		isConnected,
		latestMessage,
		connect,
		disconnect,
		sendCommand,
	};
}
