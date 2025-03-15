import BottomBar from "../../wigets/BottomBar/BottomBar";
import PageHeader from "../../wigets/header/PageHeader";

function CameraPage() {
    return (
        <>
            <PageHeader title="Camera"></PageHeader>
            <BottomBar currentPage={"camera"}></BottomBar>
        </>
    );
}

export default CameraPage;