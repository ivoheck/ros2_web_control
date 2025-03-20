import { useEffect, useState } from "react";

function CameraWidget() {

    const backendURL = window.location.origin;

    const [picture, setPicture] = useState<string | null>(null);
    const [loading, setLoading] = useState(true);
    const [error, setError] = useState(null);

    useEffect(() => {
        const fetchCameraImage = async () => {
            try {
                const response = await fetch(`${backendURL}/get_camera_image/`);
                if (!response.ok) {
                    throw new Error("Error fetching image");
                }
                const blob = await response.blob();
                const imageUrl = URL.createObjectURL(blob);
                setPicture(imageUrl);
            } catch (error: any) {
                setError(error.message);
            } finally {
                setLoading(false);
            }
        };

        const intervalId = setInterval(fetchCameraImage, 1000);

        return () => clearInterval(intervalId);
    }, []);

    if (loading) {
        return <div>Loding...</div>;
    }

    if (error) {
        return <div>Error: {error}</div>;
    }

    if (picture === null || picture === "") {
        return <div>No Image available</div>;
    }

    return (
        <>
            {picture && <img src={picture} alt="No Image available" />}
        </>
    );
}

export default CameraWidget;

