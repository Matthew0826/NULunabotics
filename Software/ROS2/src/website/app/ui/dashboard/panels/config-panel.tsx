import { useEffect, useState } from "react";
import {ConfigType, Profiles} from "@/app/types/config";
import {useWebSocketContext} from "@/app/contexts/web-socket-context";
import Config from "@/app/ui/dashboard/components/config";

export default function ConfigPanel() {
    // const [configState, setConfig] = useState<ConfigType | null>(null);

    // important states
    const [loadedConfigState, setLoadedConfigState] = useState<Profiles | null>(null);
    const [profileState, setProfileState] = useState<string | null>(null);

    const { latestMessages, allMessages, sendToServer } = useWebSocketContext();

    // using this to listen to the messages from the server
    useEffect(() => {
        if (latestMessages.length <= 0) return;

        for (const message of latestMessages) {
            if (message.type === "loadConfigProfile") {
                console.log("GOT NEW CONFIG");
                const newProfile = message.message;

                // stop unnecessary state updates
                setLoadedConfigState((prevConfig) => {
                    const existing = prevConfig?.[newProfile.profile];
                    const isSame = existing && JSON.stringify(existing) === JSON.stringify(newProfile.config);

                    if (isSame) return prevConfig;

                    return {
                        ...prevConfig,
                        [newProfile.profile]: newProfile.config
                    };
                });

                // only update profileState if different
                setProfileState((prev) =>
                    prev === newProfile.profile ? prev : newProfile.profile
                );
            }
        }
    }, [latestMessages]);

    // request the config from the server for the new profile
    const handleLoadProfile = (profileName: string) => {
        console.log("HANDLE TIME:", profileName);
        sendToServer("loadConfigProfile", { profile: profileName });
    };

    // request the server save current config to current profile
    const handleSaveProfile = () => {
        if (loadedConfigState && profileState) {
            sendToServer("saveConfigProfile", {
                profile: profileState,
                config: loadedConfigState[profileState],
            });
        }
    };

    // update our config state from user interaction
    const handleTextboxInteraction = (profile: string, node: string, category: string, setting: string, value: string) => {
        setLoadedConfigState((prev) => {
            if (!prev) return prev;

            const newConfig = [...prev[profile]];
            const nodeIndex = newConfig.findIndex((item) => item.node === node);
            if (nodeIndex !== -1) {
                const categoryIndex = newConfig[nodeIndex].categories.findIndex((cat) => cat.category === category);
                if (categoryIndex !== -1) {
                    const settingIndex = newConfig[nodeIndex].categories[categoryIndex].settings.findIndex((set) => set.setting === setting);
                    if (settingIndex !== -1) {
                        newConfig[nodeIndex].categories[categoryIndex].settings[settingIndex].value = value;
                    }
                }
            }
            const newProfiles = { ...prev };
            newProfiles[profile] = newConfig;
            return newProfiles;
        });
    }

    // ensure we have loaded before rendering
    if (!loadedConfigState || !profileState) {
        return <div className="flex flex-col gap-4 p-6 h-full w-full">Loading configuration...</div>;
    }

    // actual rendering of the config panel
    return (
        <div className="flex flex-col gap-4 p-6 h-full w-full">
            <p className="text-xs">Note: DOES save! Remember to press "save config."</p>
            
            {/* Profile Load */}
            <div className="flex items-center gap-2">
                <input
                    type="text"
                    placeholder="Enter profile name"
                    className="border px-2 py-1 rounded"
                    onKeyDown={(e) => {
                        if (e.key === "Enter") {
                            const value = (e.target as HTMLInputElement).value.trim();
                            console.log("VAL?", value);
                            if (value !== "") {
                                console.log("VAL!", value);
                                setProfileState(null);
                                handleLoadProfile(value);
                                (e.target as HTMLInputElement).value = "";
                            }
                        }
                    }}
                />
                <span className="text-xs text-gray-500">Loaded Profile: {profileState}</span>
            </div>

            {/* Config Display */}
            {profileState && loadedConfigState[profileState] && loadedConfigState[profileState].map((item, nodeIndex) => (
                <div className="flex flex-col gap-2 border-2 border-slate-800 p-4 rounded-lg" key={`node-${item.node}-${nodeIndex}`}>
                    <b className="text-center">{item.node}</b>
                    {item.categories.map((category, catIndex) => (
                        <div key={`category-${item.node}-${category.category}-${catIndex}`} className="mb-4">
                            <p className="text-center">{category.category.replaceAll("_", " ")}</p>
                            <div className="text-sm text-gray-700 flex flex-row gap-2">
                                {category.settings.map((setting, settingIndex) => (
                                    <Config
                                        key={`setting-${item.node}-${category.category}-${setting.setting}-${settingIndex}`}
                                        setting={setting.setting}
                                        value={setting.value.toString()}
                                        handleChange={(value) => handleTextboxInteraction(profileState, item.node, category.category, setting.setting, value)}
                                    />
                                ))}
                            </div>
                        </div>
                    ))}
                </div>
            ))}

            {/* Save Button */}
            <button
                onClick={handleSaveProfile}
                className="mt-4 px-4 py-2 bg-blue-600 text-white rounded hover:bg-blue-700 transition"
            >
                Save Config
            </button>
        </div>
    );
}
