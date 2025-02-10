export function GET() {
    const headers = new Headers();
    headers.set("Connection", "Upgrade");
    headers.set("Upgrade", "websocket");
    return new Response("Upgrade Required", { status: 426, headers });
}

export function SOCKET(
    client: import("ws").WebSocket,
    _request: import("node:http").IncomingMessage,
    server: import("ws").WebSocketServer
) {
    for (const other of server.clients)
        if (client !== other && other.readyState === other.OPEN)
            other.send("A new user joined the chat");

    client.on("message", (message: any) => {
        // Forward the message to all other clients
        for (const other of server.clients)
            if (client !== other && other.readyState === other.OPEN)
                other.send(message);
        console.log("received: %s", message);
    });

    client.send(
        `Welcome to the chat! There ${
            server.clients.size - 1 === 1
                ? "is 1 other user"
                : `are ${server.clients.size - 1 || "no"} other users`
        } online`
    );

    // Run code every 3 seconds
    const interval = setInterval(() => {
        client.send("Hello from the server every 3 seconds!");
    }, 3000);

    return () => {
        for (const other of server.clients)
            if (client !== other && other.readyState === other.OPEN)
                other.send("A user left the chat");
    };
}
