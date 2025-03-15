// import { HiMiniBattery0 } from "react-icons/hi2";
import './BatteryWidget.css';
import { useEffect, useState } from "react";

function BatteryWidget() {

    const [batteryState, setBatteryState] = useState(null);
    const [loading, setLoading] = useState(true);
    const [error, setError] = useState(null);

    useEffect(() => {
        const fetchBatteryState = async () => {
            try {
                const response = await fetch("http://127.0.0.1:8000/cmd_vel_button_key/");
                if (!response.ok) {
                    throw new Error("Error fetching battery state");
                }
                const data = await response.json();
                setBatteryState(data);
            } catch (error: any) {
                setError(error.message);
            } finally {
                setLoading(false);
            }
        };

        fetchBatteryState();
    }, []);

    if (loading) {
        return <div>Loding...</div>;
    }

    if (error) {
        return <div>Error: {error}</div>;
    }

    if (batteryState === null) {
        return <div>No battery state available</div>;
    }

    return (
        <>
            <div className="battery-widget">
                <h1>80 %</h1>
                {/* <HiMiniBattery0 className="battery-icon" ></HiMiniBattery0> */}

                <table className="battery-details">
                    <tr>
                        <td className="left">Voltage</td>
                        <td className="right">{batteryState} V</td>
                    </tr>

                    <tr>
                        <td className="left">Current</td>
                        <td className="right">12.3 V</td>
                    </tr>

                    <tr>
                        <td className="left">Temperature</td>
                        <td className="right">12.3 V</td>
                    </tr>

                    <tr>
                        <td className="left">Capacity</td>
                        <td className="right">12.3 V</td>
                    </tr>

                    <tr>
                        <td className="left">Design capacity</td>
                        <td className="right">12.3 V</td>
                    </tr>
                </table>
            </div>
        </>
    );
}

export default BatteryWidget;