import './BottomBar.css';
import { IoGameController } from "react-icons/io5";
import { FaBatteryThreeQuarters } from "react-icons/fa6";
import { FaMapMarkedAlt } from "react-icons/fa";
//import { IoMdSettings } from "react-icons/io";
import { FaCamera } from "react-icons/fa";

import { useNavigate } from "react-router-dom";

interface BottomBarProps {
    currentPage: string;
}


const BottomBar: React.FC<BottomBarProps> = ({ currentPage: string }) => {
    const navigate = useNavigate();

    const isCameraSelected = string === 'camera';
    const isMapSelected = string === 'map';
    const isCmdVelControlSelected = string === 'cmdVelControl';
    const isBatterySelected = string === 'battery';

    return (
        <>
            <div className="bottom-bar">
                <button className={`bar-button ${isCameraSelected && 'bar-button-selected'}`} onClick={() => navigate('/camera')} >
                    <FaCamera />
                </button>
                <button className={`bar-button ${isMapSelected && 'bar-button-selected'}`} onClick={() => navigate('/map')}>
                    <FaMapMarkedAlt />
                </button>
                <button className={`bar-button ${isCmdVelControlSelected && 'bar-button-selected'}`} onClick={() => navigate('/page')}>
                    <IoGameController />
                </button>
                <button className={`bar-button ${isBatterySelected && 'bar-button-selected'}`} onClick={() => navigate('/battery')}>
                    <FaBatteryThreeQuarters />
                </button>
            </div>
        </>
    );
}

export default BottomBar;