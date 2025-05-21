import type { Metadata } from "next";
import "@/app/ui/global.css";
import GamepadContext from "@/app/contexts/gamepad-context";
import WebSocketProvider from "@/app/contexts/web-socket-context";
import RobotContextProvider from "@/app/contexts/robot-context";

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
                     <RobotContextProvider>
                        <GamepadContext>{children}</GamepadContext>
                     </RobotContextProvider>
                </WebSocketProvider>
            </body>
        </html>
    );
}
