import BottomBar from "../../wigets/BottomBar/BottomBar";
import PageHeader from "../../wigets/header/PageHeader";
import CameraWidget from "../../wigets/CameraWidget/CameraWidget";

function CameraPage() {
    return (
        <>
            <PageHeader title="Camera"></PageHeader>
            <CameraWidget></CameraWidget>
            <BottomBar currentPage={"camera"}></BottomBar>
        </>
    );
}

export default CameraPage;