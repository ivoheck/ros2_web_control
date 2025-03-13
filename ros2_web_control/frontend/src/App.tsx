import { BrowserRouter as Router, Routes, Route } from "react-router-dom";
import CmdVelControlPage from "./componets/pages/CmdVelControlPage/CmdVelControlPage"

function App() {
  return (
    <Router>
      <Routes>
        <Route path="/" element={<CmdVelControlPage />} />
      </Routes>
    </Router>
  );
}

export default App;