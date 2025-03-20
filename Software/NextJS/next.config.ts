import type { NextConfig } from "next";
import type { Configuration } from "webpack";

const nextConfig: NextConfig = {
    output: "standalone",
    serverExternalPackages: ["rclnodejs"],
    webpack(config: Configuration, { isServer }: { isServer: boolean }) {
        // Ignore source map files to prevent parsing issues
        config.module?.rules?.push({
            test: /\.js\.map$/,
            loader: "ignore-loader",
        });
        if (isServer) {
            // Ensure the config.externals is typed properly
            config.externals = [
                {
                    rclnodejs: "commonjs rclnodejs", // Treat rclnodejs as a commonjs external
                },
            ];
        }

        return config;
    },
};

export default nextConfig;
