import './MapField.css';

interface MapFieldProps {
    fielStatus: number;
}

const MapField: React.FC<MapFieldProps> = ({ fielStatus }) => {
    switch (fielStatus) {
        case 0:
            return (
                <>
                    <div className="map-field unoccupied"></div>
                </>
            );
        case 1:
            return (
                <>
                    <div className="map-field occupied"></div>
                </>
            );
        case -1:
            return (
                <>
                    <div className="map-field unknown"></div>
                </>
            );
        default:
            return (
                <>
                    <div className="map-field unknown"></div>
                </>
            );

    }
}

export default MapField;