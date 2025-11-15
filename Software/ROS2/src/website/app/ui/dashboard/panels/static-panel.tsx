export default function StaticPanel({
    children,
    title
}: {
    children: React.ReactNode;
    title: string;
}) {

    return (
        <div className="flex-col h-full items-center rounded-xl p-6 gap-x-4 bg-black-900/50 border border-black-800 bg-slate-100">
            {(title) && (
                <div className="flex flex-row items-center gap-4 justify-center w-full">
                    <h1 className="font-bold text-gray-800 text-3xl text-center">
                        {title}
                    </h1>
                </div>
            )}

            {/* Prevent layout shift & scroll reset by avoiding conditional mount */}
            <div className={`transition-all duration-300 overflow-hidden`}>
                {children}
            </div>
        </div>
    );
}

