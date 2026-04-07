import { useState, useEffect } from "react";
import { CssBaseline, ThemeProvider } from "@mui/material";
import { useMode } from "./theme";
import PilotDashboard from "./scenes/dashboard";
import ControlPanel from "./components/ControlPanel";

function App() {
  const theme = useMode();
  const [isControl, setIsControl] = useState(
    () => window.location.hash === "#control"
  );

  useEffect(() => {
    const onHash = () => setIsControl(window.location.hash === "#control");
    window.addEventListener("hashchange", onHash);
    return () => window.removeEventListener("hashchange", onHash);
  }, []);

  return (
    <ThemeProvider theme={theme}>
      <CssBaseline />
      {isControl ? <ControlPanel /> : <PilotDashboard />}
    </ThemeProvider>
  );
}

export default App;
