import BottomBar from "../../wigets/BottomBar/BottomBar";
import PageHeader from "../../wigets/header/PageHeader";
import MapWidget from "../../wigets/MapWidget/MapWidget";

function MapPage() {
    return (
        <>
            <PageHeader title="Map"></PageHeader>
            <MapWidget></MapWidget>
            <BottomBar currentPage={"map"}></BottomBar>
        </>
    );
}

export default MapPage;