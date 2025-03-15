import { useEffect, useState } from "react";

function MapWidget() {

    const [mapState, setMapState] = useState(null);
    const [loading, setLoading] = useState(true);
    const [error, setError] = useState(null);

    useEffect(() => {
        const fetchMapState = async () => {
            try {
                const response = await fetch("http://127.0.0.1:8000/get_map/");
                if (!response.ok) {
                    throw new Error("Error fetching battery state");
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
        return <div>No Map state available</div>;
    }

    return (
        <>
            <h1>{mapState}</h1>
        </>
    );
}

export default MapWidget;