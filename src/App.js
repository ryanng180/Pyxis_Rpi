import { CssBaseline, ThemeProvider } from "@mui/material";
import { useMode } from "./theme";
import PilotDashboard from "./scenes/dashboard";

function App() {
  const theme = useMode();

  return (
    <ThemeProvider theme={theme}>
      <CssBaseline />
      <PilotDashboard />
    </ThemeProvider>
  );
}

export default App;
