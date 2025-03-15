import { BrowserRouter as Router, Routes, Route } from "react-router-dom";
import CmdVelControlPage from "./componets/pages/CmdVelControlPage/CmdVelControlPage"
import BatteryPage from "./componets/pages/BatteryPage/BatteryPage";
import MapPage from "./componets/pages/MapPage/MapPage";
import CameraPage from "./componets/pages/CameraPage/CameraPage";

function App() {
  return (
    <Router>
      <Routes>
        <Route path="/" element={<CmdVelControlPage />} />
        <Route path="/battery" element={<BatteryPage />} />
        <Route path="/map" element={<MapPage />} />
        <Route path="/camera" element={<CameraPage />} />
      </Routes>
    </Router>
  );
}

export default App;