import {ReactNode, useState} from "react";

export default function Panel({
    children,
    title,
    canHide = true,
    hiddenByDefault = false,
    contentPadding = true,
}: {
    children: ReactNode;
    title: string;
    canHide?: boolean;
    hiddenByDefault?: boolean;
    contentPadding?: boolean;
}) {
    const [open, setOpen] = useState(!hiddenByDefault);
    const togglePanel = () => setOpen(!open);

    return (
        <div className={"flex-col h-full items-center rounded-xl gap-x-4 bg-black-900/50 border border-black-800 bg-slate-100 "}>
            {(title || canHide) && (
                <div className={"flex flex-row items-center rounded-t-xl gap-4 justify-center w-full py-2 bg-slate-200 " + (open ? "border-b-3 accent-slate-600" : "rounded-b-xl")}>
                    {title && (
                        <h1 className="font-bold text-gray-800 text-2xl text-center">
                            {title}
                        </h1>
                    )}
                    {canHide && (
                        <button
                            className="text-2xl text-gray-500 font-thin hover:text-gray-700 focus:outline-none font-mono"
                            onClick={togglePanel}
                        >
                            {open ? <span>⮝</span> : <span>⮟</span>}
                        </button>
                    )}
                </div>
            )}
            {/* Prevent layout shift & scroll reset by avoiding conditional mount */}
            {/*<div className={`transition-all duration-300 overflow-hidden ${open ? "max-h-[1000px] p-3" : "max-h-0 px-3"}`}>*/}
            <div className={`transition-all duration-300 overflow-hidden ${open && contentPadding ? "max-h-[1000px] p-3" : (open ? "max-h-[1000px] p-0" : "max-h-0 px-3")}`}>
                {children}
            </div>
        </div>
    );
}

