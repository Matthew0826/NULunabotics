import SerialPort from "../components/serial-port";
import { useEffect, useState } from "react";
import {useWebSocketContext} from "@/app/contexts/web-socket-context";

type SerialPortType = {
    name: string;
    port: string;
    topics: string[];
};

export default function SerialPortPanel() {
    const { latestMessages, sendToServer } = useWebSocketContext();
    const [serialPorts, setSerialPorts] = useState<SerialPortType[]>([]);

    useEffect(() => {
        if (latestMessages.length == 0) return;
        for(const message of latestMessages) {
            if (message.type === "serial_port_state") {
                const data = message.message;
                setSerialPorts(data.states || []);
            }
        }
    }, [latestMessages]);

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
