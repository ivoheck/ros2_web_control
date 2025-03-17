import { useEffect, useState } from "react";

interface MapState {
    width: number;
    height: number;
    resolution: number;
    data: number[];
}

function MapWidget() {

    const backendURL = window.location.origin;

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

        fetchMapState();
    }, []);

    if (loading) {
        return <div>Loding...</div>;
    }

    if (error) {
        return <div>Error: {error}</div>;
    }

    if (mapState === null) {
        return <div>No Map available</div>;
    }

    return (
        <>
            <h1>{mapState.data}</h1>
        </>
    );
}

export default MapWidget;