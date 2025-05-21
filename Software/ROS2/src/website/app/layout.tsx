import type { Metadata } from "next";
import "@/app/ui/global.css";
import WebSocketProvider from "./lib/web-socket-context";
import GamepadStateProvider from "./ui/dashboard/gamepad-state-provider";
import RobotContextProvider from "@/app/lib/robot-context";

export const metadata: Metadata = {
    title: "Dashboard",
    description: "Northeastern University Lunabotics Dashboard",
};

export default function RootLayout({
    children,
}: {
    children: React.ReactNode;
}) {
    return (
        <html lang="en">
            <head>
                <title>Dashboard</title>
                <link rel="icon" href="/favicon.ico" />
                <meta
                    name="viewport"
                    content="width=device-width, initial-scale=1.0"
                />
            </head>
            <body className="bg-zinc-800">
                <WebSocketProvider>
                    {/* <RobotContextProvider> */}
                    <GamepadStateProvider>{children}</GamepadStateProvider>
                    {/* </RobotContextProvider> */}
                </WebSocketProvider>
            </body>
        </html>
    );
}
