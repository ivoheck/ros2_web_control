import './CmdVelControlWidget.css';
import { useState, useEffect } from 'react';

function CmdVelControlWidget() {

    const [currentPress, setIsPressed] = useState(-1);

    useEffect(() => {
        let intervalId: NodeJS.Timeout | null = null;

        if (currentPress !== -1) {
            intervalId = setInterval(() => {
                console.log(`Sending key ${currentPress} to backend`);

                const buttonKey = {
                    key: currentPress
                };

                fetch("http://127.0.0.1:8000/cmd_vel_button_key/", {
                    method: "POST",
                    headers: {
                        "Content-Type": "application/json",
                    },
                    body: JSON.stringify(buttonKey),
                })
                    .then(response => response.json())
                    .then(data => {
                        console.log("Server-Antwort:", data);
                    })
                    .catch(error => {
                        console.error("Fehler beim Senden der Anfrage:", error);
                    });

            }, 100);
        } else {
            if (intervalId) {
                clearInterval(intervalId);
            }
        }

        // Cleanup
        return () => {
            if (intervalId) {
                clearInterval(intervalId);
            }
        };
    }, [currentPress]);

    const handleMouseDown = (key: number) => {
        console.log("Mouse Down");
        setIsPressed(key);
    }

    const handleMouseUp = () => {
        console.log("Mouse Up");
        setIsPressed(-1);
    };

    const handleMouseLeave = () => {
        console.log("Mouse Leave");
        setIsPressed(-1);
    };

    return (
        <>
            <div className="container">
                <div className="grid-container">
                    <button className="grid-item"
                        onMouseDown={() => handleMouseDown(1)}
                        onMouseUp={() => handleMouseUp()}
                        onMouseLeave={() => handleMouseLeave()}>
                        <div className="arrow arrow-up-left"></div>
                    </button>

                    <button
                        className="grid-item"
                        onMouseDown={() => handleMouseDown(2)}
                        onMouseUp={() => handleMouseUp()}
                        onMouseLeave={() => handleMouseLeave()}>
                        <div className="arrow arrow-up"></div>
                    </button>

                    <button className="grid-item"
                        onMouseDown={() => handleMouseDown(3)}
                        onMouseUp={() => handleMouseUp()}
                        onMouseLeave={() => handleMouseLeave()}>
                        <div className="arrow arrow-up-right"></div>
                    </button>

                    <button className="grid-item"
                        onMouseDown={() => handleMouseDown(4)}
                        onMouseUp={() => handleMouseUp()}
                        onMouseLeave={() => handleMouseLeave()}>
                        <div className="arrow arrow-left"></div>
                    </button>

                    <button className="grid-item"
                        onMouseDown={() => handleMouseDown(-1)}
                        onMouseUp={() => handleMouseUp()}
                        onMouseLeave={() => handleMouseLeave()}>
                    </button>

                    <button className="grid-item"
                        onMouseDown={() => handleMouseDown(5)}
                        onMouseUp={() => handleMouseUp()}
                        onMouseLeave={() => handleMouseLeave()}>
                        <div className="arrow arrow-right"></div>
                    </button>

                    <button className="grid-item"
                        onMouseDown={() => handleMouseDown(6)}
                        onMouseUp={() => handleMouseUp()}
                        onMouseLeave={() => handleMouseLeave()}>
                        <div className="arrow arrow-down-left"></div>
                    </button>

                    <button className="grid-item"
                        onMouseDown={() => handleMouseDown(7)}
                        onMouseUp={() => handleMouseUp()}
                        onMouseLeave={() => handleMouseLeave()}>
                        <div className="arrow arrow-down"></div>
                    </button>

                    <button className="grid-item"
                        onMouseDown={() => handleMouseDown(8)}
                        onMouseUp={() => handleMouseUp()}
                        onMouseLeave={() => handleMouseLeave()}>
                        <div className="arrow arrow-down-right"></div>
                    </button>
                </div>
            </div>
        </>
    );
}

export default CmdVelControlWidget;