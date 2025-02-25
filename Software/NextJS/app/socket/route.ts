import { IncomingMessage } from "node:http";
import WebSocket, { WebSocketServer } from "ws";

export function GET() {
    const headers = new Headers();
    headers.set("Connection", "Upgrade");
    headers.set("Upgrade", "websocket");
    return new Response("Upgrade Required", { status: 426, headers });
}

export function SOCKET(
    client: WebSocket,
    _request: IncomingMessage,
    server: WebSocketServer
) {
    // for (const other of server.clients)
    //     if (client !== other && other.readyState === other.OPEN)
    //         other.send("A new user joined the chat");

    client.on("message", (message: WebSocket.RawData) => {
        // // Forward the message to all other clients
        // for (const other of server.clients)
        //     if (client !== other && other.readyState === other.OPEN)
        //         other.send(message);

        const messageString = message.toString();
        console.log("received: ", messageString);
    });

    // client.send(
    //     `Welcome to the chat! There ${
    //         server.clients.size - 1 === 1
    //             ? "is 1 other user"
    //             : `are ${server.clients.size - 1 || "no"} other users`
    //     } online`
    // );

    let counter = 0;

    const interval = setInterval(() => {
        client.send(
            JSON.stringify([
                {
                    graph: "Power",
                    dataSet: "Test Data",
                    newData: [Math.floor(Math.random() * 30)],
                },
                {
                    graph: "Other Graph",
                    dataSet: "Sin",
                    newData: [Math.sin(counter / Math.PI) * 25],
                },
                {
                    graph: "Other Graph",
                    dataSet: "Cos",
                    newData: [Math.cos(counter / Math.PI) * 25],
                },
            ])
        );
        counter++;
    }, 1000);

    return () => {
        clearInterval(interval);
        // for (const other of server.clients)
        //     if (client !== other && other.readyState === other.OPEN)
        // other.send("A user left the chat");
    };
}
