import BottomBar from "../../wigets/BottomBar/BottomBar";
import CmdVelControlWidget from "../../wigets/CmdVelControlWidget/CmdVelControlWidget"
import PageHeader from "../../wigets/header/PageHeader";

function CmdVelControlPage() {
    return (
        <>
            <PageHeader title="Cmd Vel Control"></PageHeader>
            <CmdVelControlWidget></CmdVelControlWidget>
            <BottomBar currentPage={"cmdVelControl"} ></BottomBar>
        </>
    );
}

export default CmdVelControlPage;