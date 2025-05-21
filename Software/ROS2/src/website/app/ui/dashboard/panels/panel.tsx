import {ReactNode, useState} from "react";

export default function Panel({
    children,
    title,
    canHide = true,
    hiddenByDefault = false
}: {
    children: ReactNode;
    title: string;
    canHide?: boolean;
    hiddenByDefault?: boolean;
}) {
    const [open, setOpen] = useState(!hiddenByDefault);
    const togglePanel = () => {
        setOpen(!open);
    }
    return (
        <div className="flex-col h-full items-center rounded-xl p-6 gap-x-4 bg-black-900/50 border border-black-800 bg-slate-100">
            {(title || canHide) && <div className="flex flex-row items-center gap-4 justify-center w-full">
                {title && (
                    <h1 className="font-bold text-gray-800 text-3xl text-center">
                        {title}
                    </h1>
                )}
                {canHide && <button
                    className="text-2xl text-gray-500 font-thin hover:text-gray-700 focus:outline-none font-mono"
                    onClick={togglePanel}
                >
                    {open ? (
                        <span>⮝</span>
                    ) : (
                        <span>⮟</span>
                    )}
                </button>}
            </div>}
            {open && children}
        </div>
    );
}
