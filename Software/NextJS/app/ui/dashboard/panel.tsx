export default function Panel({ children }: { children: React.ReactNode }) {
    return (
        <div className="mx-auto flex items-center rounded-x1 p-6 gap-x-4">
            {children}
        </div>
    );
}
