export default function Config({ setting, value, handleChange }: { setting: string, value: string, handleChange: (value: string) => void }) {
    return (
        <div>
            <label className="text-sm text-gray-500">{setting}</label>
            <input
                type="text"
                value={value}
                onChange={(e) => handleChange(e.target.value)}
                className="border border-gray-300 rounded-md p-2 w-full"
            />
        </div>
    )
}