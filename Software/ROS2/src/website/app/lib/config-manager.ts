import path from "path";
import fs from "fs";
import { warn } from "console";
import {ConfigType, Profiles} from "@/app/types/config";

export function setConfigFromProfile(config: ConfigType[] | null, profileName: string) {
    console.log("SET PROFILE");
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



// Create a zeroed version of the config (if exists then zero the existing values)
// Use default as template to generate zeroed version?
function initConfigProfile(profileName: string): ConfigType[] {
    console.log("INIT PROFILE");
    const templatePath = path.join(process.cwd(), "public", "config.template.json");

    let defaultConfig: ConfigType[] = [{
        node: "placeholder",
        categories: []
    }];

    try {
        if (fs.existsSync(templatePath)) {
            const data = fs.readFileSync(templatePath, "utf-8");
            const parsed = JSON.parse(data);
            if (Array.isArray(parsed)) {
                defaultConfig = parsed;
            }
        } else {
            console.warn("Template config file not found. Using fallback empty config.");
        }
    } catch (err) {
        console.error("Failed to load template config:", err);
    }

    setConfigFromProfile(defaultConfig, profileName);
    console.log(`Created new profile "${profileName}" in config.json`);

    return defaultConfig;
}

export function getConfigFromProfile(profileName: string): ConfigType[] {
    console.log("GET PROFILE");
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
    // if (profiles[profileName]) {
    //     return profiles[profileName];
    // }
    if (profiles.hasOwnProperty(profileName)) {
        return profiles[profileName];
    }

    // Case 2: Does not exist, make a new zeroed version.
    console.warn(`Profile "${profileName}" not found. Generating zeroed config.`);
    return initConfigProfile(profileName);
}
