export default function Panel({
    children,
    title,
}: {
    children: React.ReactNode;
    title: string;
}) {
    return (
        <div className="flex-col items-center rounded-x1 p-6 gap-x-4 bg-black-900/50 border border-black-800 grow-1">
            <h1 className="text-2xl font-bold text-gray-800 md:text-3xl text-center">
                {title}
            </h1>
            {children}
        </div>
    );
}
