import { useWebSocketContext } from "@/app/lib/web-socket-context";
import SerialPort from "./serial-port";
import { useEffect, useState } from "react";

type SerialPortType = {
    name: string;
    port: string;
    topics: string[];
};

export default function SerialPortDisplay() {
    const { messages, sendToServer } = useWebSocketContext();
    const [serialPorts, setSerialPorts] = useState<SerialPortType[]>([]);
    useEffect(() => {
        if (messages.length == 0) return;
        const portMessage = messages[messages.length - 1];
        if (portMessage.type === "serial_port_state") {
            console.log("Serial port state: ", portMessage?.message);
            setSerialPorts(portMessage?.message?.states || []);
        }
    }, [messages]);
    const handleUploadCode = (port: string) => {
        if (window.confirm("Are you sure you want to upload code? This will disconnect the board briefly and RESET it!")) {
            console.log(`Uploading code to ${port}`);
            sendToServer("uploadCode", { port: port });
        }
    };
    return (
        <div className="flex flex-wrap items-center gap-5 justify-center pt-4">
            {serialPorts.map((serialPort, index) => (
                <SerialPort
                    key={index}
                    name={serialPort.name}
                    port={serialPort.port}
                    topics={serialPort.topics}
                    handleUploadCode={handleUploadCode}
                />
            ))}
        </div>
    )
}