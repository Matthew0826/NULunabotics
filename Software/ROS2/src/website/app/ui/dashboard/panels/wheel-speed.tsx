import { useGamepadManagerContext } from "@/app/contexts/gamepad-context";
import Slider from "../components/slider";

export default function WheelSpeedPanel() {
    const { state, setState, speed, setSpeed } = useGamepadManagerContext();
    return <Slider labels={[0.0, 0.5, 1.0]} value={speed} setValue={setSpeed} />
}
