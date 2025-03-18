import BottomBar from "../../wigets/BottomBar/BottomBar";
import PageHeader from "../../wigets/header/PageHeader";
import BatteryWidget from "../../wigets/BatteryWidget/BatteryWidget";

function BatteryPage() {
    return (
        <>
            <PageHeader title="Battery"></PageHeader>
            <BatteryWidget></BatteryWidget>
            <BottomBar currentPage={"battery"}></BottomBar>
        </>
    );
}

export default BatteryPage;