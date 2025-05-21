import { useMemo, useState } from "react";
import Slider from "../components/slider";
import {useWebSocketContext} from "@/app/contexts/web-socket-context";

export default function OrientationCorrectionPanel() {
    const [orientationCorrection, setOrientationCorrection] = useState(0.0);
    const { sendToServer } = useWebSocketContext();

    // Send to server when the orientation correction changes
    useMemo(() => {
        // Send the orientation correction to the server
        sendToServer("orientationCorrection", {
            orientationCorrection: orientationCorrection,
        });
    }, [orientationCorrection]);
    return (
        <Slider labels={[0.0, 90.0, 180.0, 270.0, 360.0]} value={orientationCorrection} setValue={setOrientationCorrection} />
    )
}
