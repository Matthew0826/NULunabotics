import path from "path";
import fs from "fs";
import { ConfigType, Profiles } from "../ui/dashboard/config-panel";
import { warn } from "console";

export function setConfigFromProfile(config: ConfigType | null, profileName: string) {
    // Ensure not null!
    if (config == null) {
        warn("Config is null, cannot save profile");
        return
    };

    const configPath = path.join(process.cwd(), "public", "config.json");

    let profiles: Profiles = {};

    // Ensure config file exists and we can load it
    if (fs.existsSync(configPath)) {
        try {
            const data = fs.readFileSync(configPath, "utf-8");
            profiles = JSON.parse(data);
        } catch (err) {
            console.error("Failed to read config.json:", err);
        }
    }

    // Update or add the profile
    profiles[profileName] = config;

    // Ensure we can write the updated profile back to the file
    try {
        fs.writeFileSync(configPath, JSON.stringify(profiles, null, 2), "utf-8");
        console.log(`Saved profile "${profileName}" to config.json`);
    } catch (err) {
        console.error("Failed to write config.json:", err);
    }
}

// create a zeroed config type
function createZeroedConfig(): ConfigType {
    return {
        name: ""
    }
}

// Create a zeroed version of the config (if exists then zero the existing values)
// Use default as template to generate zeroed version?
function initConfigProfile(profileName: string): ConfigType {
    // make the zeroed version of the config
    const zeroedConfig: ConfigType = createZeroedConfig();

    // set this zeroed config to the profile
    setConfigFromProfile(zeroedConfig, profileName);
    console.log(`Created new profile "${profileName}" in config.json`);

    // pass along the values
    return zeroedConfig;
}

export function getConfigFromProfile(profileName: string): ConfigType {
    const configPath = path.join(process.cwd(), "public", "config.json");

    let profiles: Profiles = {};

    // Ensure we can properly read the config file
    try {
        if (fs.existsSync(configPath)) {
            const data = fs.readFileSync(configPath, "utf-8");
            const parsed = JSON.parse(data);

            if (typeof parsed === "object" && !Array.isArray(parsed)) {
                profiles = parsed;
            } else {
                console.warn("config.json is not a valid object. Treating as empty.");
            }
        }
    } catch (err) {
        console.error("Failed to read config.json:", err);
    }

    // Case 1: Exact profile exists, return values easy
    if (profiles[profileName]) {
        return profiles[profileName];
    }

    // Case 2: Does not exist, make a new zeroed version.
    console.warn(`Profile "${profileName}" not found. Generating zeroed config.`);
    return initConfigProfile(profileName);
}