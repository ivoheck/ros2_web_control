import './CmdVelControlWidget.css';
import { useState, useEffect } from 'react';

function CmdVelControlWidget() {

    const [currentPress, setIsPressed] = useState(-1);
    const backendURL = window.location.origin;

    useEffect(() => {
        let intervalId: NodeJS.Timeout | null = null;

        if (currentPress !== -1) {
            intervalId = setInterval(() => {
                fetch(`${backendURL}/cmd_vel_button_key/`, {
                    method: "POST",
                    headers: {
                        "Content-Type": "application/json",
                    },
                    body: JSON.stringify({ key: currentPress }),
                }).catch(error => console.error("Error:", error));
            }, 100);
        }

        return () => {
            if (intervalId) clearInterval(intervalId);
        };
    }, [currentPress]);

    const handleEventDown = (key: number) => {
        console.log("Mouse Down");
        setIsPressed(key);
    }

    const handleEventUp = () => {
        console.log("Mouse Up");
        setIsPressed(-1);
    };

    const handleEventLeave = () => {
        console.log("Mouse Leave");
        setIsPressed(-1);
    };

    return (
        <>
            <div className="container">
                <div className="grid-container">
                    <button className="grid-item"
                        onMouseDown={() => handleEventDown(1)}
                        onMouseUp={() => handleEventUp()}
                        onMouseLeave={() => handleEventLeave()}

                        onTouchStart={() => handleEventDown(1)}
                        onTouchEnd={() => handleEventUp()}
                        onTouchCancel={() => handleEventLeave()}>
                        <div className="arrow arrow-up-left"></div>
                    </button>

                    <button
                        className="grid-item"
                        onMouseDown={() => handleEventDown(2)}
                        onMouseUp={() => handleEventUp()}
                        onMouseLeave={() => handleEventLeave()}

                        onTouchStart={() => handleEventDown(2)}
                        onTouchEnd={() => handleEventUp()}
                        onTouchCancel={() => handleEventLeave()}>
                        <div className="arrow arrow-up"></div>
                    </button>

                    <button className="grid-item"
                        onMouseDown={() => handleEventDown(3)}
                        onMouseUp={() => handleEventUp()}
                        onMouseLeave={() => handleEventLeave()}

                        onTouchStart={() => handleEventDown(3)}
                        onTouchEnd={() => handleEventUp()}
                        onTouchCancel={() => handleEventLeave()}
                    >
                        <div className="arrow arrow-up-right"></div>
                    </button>

                    <button className="grid-item"
                        onMouseDown={() => handleEventDown(4)}
                        onMouseUp={() => handleEventUp()}
                        onMouseLeave={() => handleEventLeave()}

                        onTouchStart={() => handleEventDown(4)}
                        onTouchEnd={() => handleEventUp()}
                        onTouchCancel={() => handleEventLeave()}
                    >
                        <div className="arrow arrow-left"></div>
                    </button>

                    <button className="grid-item"
                        onMouseDown={() => handleEventDown(5)}
                        onMouseUp={() => handleEventUp()}
                        onMouseLeave={() => handleEventLeave()}

                        onTouchStart={() => handleEventDown(5)}
                        onTouchEnd={() => handleEventUp()}
                        onTouchCancel={() => handleEventLeave()}>
                    </button>

                    <button className="grid-item"
                        onMouseDown={() => handleEventDown(6)}
                        onMouseUp={() => handleEventUp()}
                        onMouseLeave={() => handleEventLeave()}

                        onTouchStart={() => handleEventDown(6)}
                        onTouchEnd={() => handleEventUp()}
                        onTouchCancel={() => handleEventLeave()}>
                        <div className="arrow arrow-right"></div>
                    </button>

                    <button className="grid-item"
                        onMouseDown={() => handleEventDown(7)}
                        onMouseUp={() => handleEventUp()}
                        onMouseLeave={() => handleEventLeave()}

                        onTouchStart={() => handleEventDown(7)}
                        onTouchEnd={() => handleEventUp()}
                        onTouchCancel={() => handleEventLeave()}>
                        <div className="arrow arrow-down-left"></div>
                    </button>

                    <button className="grid-item"
                        onMouseDown={() => handleEventDown(8)}
                        onMouseUp={() => handleEventUp()}
                        onMouseLeave={() => handleEventLeave()}

                        onTouchStart={() => handleEventDown(8)}
                        onTouchEnd={() => handleEventUp()}
                        onTouchCancel={() => handleEventLeave()}>
                        <div className="arrow arrow-down"></div>
                    </button>

                    <button className="grid-item"
                        onMouseDown={() => handleEventDown(9)}
                        onMouseUp={() => handleEventUp()}
                        onMouseLeave={() => handleEventLeave()}

                        onTouchStart={() => handleEventDown(9)}
                        onTouchEnd={() => handleEventUp()}
                        onTouchCancel={() => handleEventLeave()}>
                        <div className="arrow arrow-down-right"></div>
                    </button>
                </div>
            </div>
        </>
    );
}

export default CmdVelControlWidget;