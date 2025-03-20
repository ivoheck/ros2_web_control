import { useEffect, useState } from "react";
import MapField from "./MapField";
import './MapWidget.css';

interface MapState {
    width: number;
    height: number;
    resolution: number;
    data: number[];
}

function MapWidget() {

    const backendURL = window.location.origin;

    console.log(backendURL);

    const [mapState, setMapState] = useState<MapState | null>(null);
    const [loading, setLoading] = useState(true);
    const [error, setError] = useState(null);

    useEffect(() => {
        const fetchMapState = async () => {
            try {
                const response = await fetch(`${backendURL}/get_map/`);
                if (!response.ok) {
                    throw new Error("Error map data");
                }
                const data = await response.json();
                setMapState(data);
            } catch (error: any) {
                setError(error.message);
            } finally {
                setLoading(false);
            }
        };

        const intervalId = setInterval(fetchMapState, 1000);

        return () => clearInterval(intervalId);
    }, []);

    if (loading) {
        return <div>Loding...</div>;
    }

    if (error) {
        return <div>Error: {error}</div>;
    }

    if (mapState === null) {
        return <div className="nomap">No map available. Make sure to publish an OccupancyGrid message on the specified topic</div>;
    }

    return (
        <>
            <div className="map-grid"
                style={{
                    gridTemplateColumns: `repeat(${mapState.width}, 1fr)`,
                    gridTemplateRows: `repeat(${mapState.height}, 1fr)`,
                }}
            >
                {mapState.data.map((value) => (
                    <MapField fielStatus={value} ></MapField>
                ))
                }
            </div>
        </>
    );
}

export default MapWidget;