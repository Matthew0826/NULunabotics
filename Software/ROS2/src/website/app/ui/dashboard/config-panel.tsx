import { useEffect, useState } from "react";
import config from "public/config.json";
import Config from "./config";
import { useWebSocketContext } from "@/app/lib/web-socket-context";

export default function ConfigPanel() {
    const [configState, setConfig] = useState(config);
    const { messages, sendToServer } = useWebSocketContext();
    const handleChangeConfig = (node: string, category: string, setting: string, value: string) => {
        // Update the config state with the new value
        setConfig((prevConfig) => {
            const newConfig = [...prevConfig];
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
            return newConfig;
        });
        sendToServer("config", {
            node: node,
            category: category,
            setting: setting,
            value: value,
        })
    }
    return (
        <div className="flex flex-col gap-4 p-6 h-full w-full">
            <p>Note: doesn't save! Remember to paste your values somewhere.</p>
            {configState.map((item, index) => (
                <div className="flex flex-col gap-2 border-2 border-slate-800 p-2 rounded-lg" key={index}>
                    <b className="text-center">{item.node}</b>
                    {
                        item.categories.map((category) => (
                            <>
                                <p className="text-center">{category.category}</p>
                                <div key={category.category} className="text-sm text-gray-700 flex flex-row gap-2">
                                    {category.settings.map((setting) => (
                                        <Config key={setting.setting} setting={setting.setting} value={setting.value.toString()} handleChange={(value) => handleChangeConfig(item.node, category.category, setting.setting, value)} />
                                    ))}
                                </div>
                            </>
                        ))
                    }
                </div>
            ))}
        </div>
    );
}