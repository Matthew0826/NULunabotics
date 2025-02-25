import DashboardPanels from "./ui/dashboard/dashboard-panels";

export default function Page() {
    const handleReceiveData = (data: any) => {};
    return (
        <main
            className="flex h-screen w-full"
            style={{ justifySelf: "center" }}
        >
            <DashboardPanels />
            {/* <GamepadTester /> */}
        </main>
    );
}
