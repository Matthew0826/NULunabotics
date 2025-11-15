export type ConfigType = {
    node: string;
    categories: {
        category: string;
        settings: {
            setting: string;
            value: string;
        }[];
    }[];
};

export type Profiles = {
    [key: string]: ConfigType[];
};
