import { useEffect, useState } from "react";
import config from "public/config.json";
import Config from "./config";
import { useWebSocketContext } from "@/app/lib/web-socket-context";

// type ConfigType = {
//     node: string;
//     categories: {
//         category: string;
//         settings: {
//             setting: string;
//             value: string | number | boolean;
//         }[];
//     }[];
// }[];

type ConfigType = {
    node: string;
    category: string;
    settings: {
        setting: string;
        value: string | number | boolean;
    }[];
};

type Profiles = {
    [key: string]: ConfigType;
};

export default function ConfigPanel() {
    // const [configState, setConfig] = useState<ConfigType | null>(null);

    // important states
    const [loadedConfigState, setLoadedConfigState] = useState<Profiles | null>(null);
    const [profileState, setProfileState] = useState<string | null>(null);

    // what is this?
    const { messages, sendToServer } = useWebSocketContext();

    // using this to listen to the messages from the server
    useEffect(() => {
        // return if no message from the server
        if (messages.length <= 0) { return; }

        // get the last message
        const lastMessage = messages[messages.length - 1];

        // we can now load the config profile
        if (lastMessage.type === "loadConfigProfile") {
            const newProfile = lastMessage.message;
            setProfileState(newProfile);
        }
    }, [messages]);

    // request the config from the server for the new profile
    const handleLoadProfile = (profileName: string) => {
        sendToServer("loadConfigProfile", profileName);
    };

    // request the server save current config to current profile
    const handleSaveProfile = () => {
        if (loadedConfigState && profileState) {
            sendToServer("saveConfigProfile", {
                profileName: profileState,
                config: configState,
            });
        }
    };

    // update our config state from user interaction
    const handleTextboxInteraction = (node: string, category: string, setting: string, value: string) => {
        return;
    }

    // const handleChangeConfig = (node: string, category: string, setting: string, value: string) => {
    //     setConfig((prevConfig) => {
    //         if (!prevConfig) return prevConfig;

    //         const newConfig = [...prevConfig];
    //         const nodeIndex = newConfig.findIndex((item) => item.node === node);
    //         if (nodeIndex !== -1) {
    //             const categoryIndex = newConfig[nodeIndex].categories.findIndex((cat) => cat.category === category);
    //             if (categoryIndex !== -1) {
    //                 const settingIndex = newConfig[nodeIndex].categories[categoryIndex].settings.findIndex((set) => set.setting === setting);
    //                 if (settingIndex !== -1) {
    //                     newConfig[nodeIndex].categories[categoryIndex].settings[settingIndex].value = value;
    //                 }
    //             }
    //         }
    //         return newConfig;
    //     });

    //     sendToServer("config", {
    //         node: node,
    //         category: category,
    //         setting: setting,
    //         value: value,
    //     });
    // };

    // ensure we have loaded before rendering
    if (!loadedConfigState || !profileState) {
        return <div className="flex flex-col gap-4 p-6 h-full w-full">Loading configuration...</div>;
    }

    // actual rendering of the config panel
    return (
        <div className="flex flex-col gap-4 p-6 h-full w-full">
            <p className="text-xs">Note: doesn't save! Remember to paste your values somewhere.</p>
            {loadedConfigState.map((item, nodeIndex) => (
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
                                        handleChange={(value) => handleTextboxInteraction(item.node, category.category, setting.setting, value)}
                                    />
                                ))}
                            </div>
                        </div>
                    ))}
                </div>
            ))}
        </div>
    );
}