import type { NextConfig } from "next";

const nextConfig: NextConfig = {
    output: "standalone",
    serverExternalPackages: ["rclnodejs"],
};

export default nextConfig;
