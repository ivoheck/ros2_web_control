import { useEffect, useState, useRef } from "react";
// import MapField from "./MapField";
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
    const canvasRef = useRef<HTMLCanvasElement>(null);

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

    useEffect(() => {
        if (!mapState || !canvasRef.current) return;

        const canvas = canvasRef.current;
        const ctx = canvas.getContext("2d");
        if (!ctx) return;

        const { width, height, data } = mapState;

        canvas.width = width;
        canvas.height = height;
        const imageData = ctx.createImageData(width, height);

        for (let i = 0; i < data.length; i++) {
            const value = data[i];

            const color = value === 100 ? [0, 0, 0] : value === 0 ? [255, 255, 255] : [150, 150, 150];

            imageData.data[i * 4] = color[0];
            imageData.data[i * 4 + 1] = color[1];
            imageData.data[i * 4 + 2] = color[2];
            imageData.data[i * 4 + 3] = 255;
        }

        ctx.putImageData(imageData, 0, 0);
    }, [mapState]);

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
            <canvas ref={canvasRef} style={{ width: "100%", height: "auto" }} />
        </>
    );
}

export default MapWidget;