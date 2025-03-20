import './BatteryWidget.css';
import { useEffect, useState } from "react";

interface BatteryState {
    voltage: number;
    percentage: number;
    current: number;
    charge: number;
    capacity: number;
    design_capacity: number;
}

function BatteryWidget() {

    const backendURL = window.location.origin;

    const [batteryState, setBatteryState] = useState<BatteryState | null>(null);
    const [loading, setLoading] = useState(true);
    const [error, setError] = useState(null);

    useEffect(() => {
        const fetchBatteryState = async () => {
            try {
                const response = await fetch(`${backendURL}/get_battery_state/`);
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

        const intervalId = setInterval(fetchBatteryState, 1000);

        return () => clearInterval(intervalId);
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

    let percentage: number = parseFloat(batteryState.percentage.toFixed(4)) * 100;

    if (percentage < 0) {
        percentage = 0;
    }

    if (percentage > 100) {
        percentage = 100;
    }

    return (
        <>
            <div className="battery-widget">

                <div className='battery-percentage'>
                    <h1>{percentage} %</h1>
                </div>

                <div className='battery-icon'>
                    <div className='battery-body'
                        style={{
                            background: `linear-gradient(to right, green ${percentage}%, transparent ${100 - percentage}%)`
                        }}
                    ></div>

                    <div className='battery-cap'></div>
                </div>

                <table className="battery-details">
                    <tr>
                        <td className="left">Voltage</td>
                        <td className="right">{batteryState.voltage} V</td>
                    </tr>

                    <tr>
                        <td className="left">Current</td>
                        <td className="right">{batteryState.current} A</td>
                    </tr>

                    <tr>
                        <td className="left">Charge</td>
                        <td className="right">{batteryState.charge} Ah</td>
                    </tr>

                    <tr>
                        <td className="left">Capacity</td>
                        <td className="right">{batteryState.capacity} Ah</td>
                    </tr>

                    <tr>
                        <td className="left">Design capacity</td>
                        <td className="right">{batteryState.design_capacity} Ah</td>
                    </tr>
                </table>
            </div >
        </>
    );
}

export default BatteryWidget;