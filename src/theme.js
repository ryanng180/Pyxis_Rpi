import { useMemo } from "react";
import { createTheme } from "@mui/material/styles";

// Maritime safety color tokens
export const safetyColors = {
  safe: "#00ff88",
  caution: "#ffaa00",
  danger: "#ff2222",
};

// Color tokens for the maritime dark theme
export const tokens = () => ({
  grey: {
    100: "#f0f0f0",
    200: "#c2c2c2",
    300: "#a3a3a3",
    400: "#858585",
    500: "#666666",
    600: "#525252",
    700: "#3d3d3d",
    800: "#292929",
    900: "#141414",
  },
  primary: {
    100: "#d0d1d5",
    200: "#a1a4ab",
    300: "#727681",
    400: "#1a2035",
    500: "#121828",
    600: "#0e1320",
    700: "#0a0e1a",
    800: "#070a14",
    900: "#04060a",
  },
  greenAccent: {
    100: "#d4ffe8",
    200: "#99ffcc",
    300: "#66ffaa",
    400: "#33ff99",
    500: "#00ff88",
    600: "#00cc6d",
    700: "#009952",
    800: "#006637",
    900: "#00331b",
  },
  redAccent: {
    100: "#ffd4d4",
    200: "#ffaaaa",
    300: "#ff7777",
    400: "#ff4444",
    500: "#ff2222",
    600: "#cc1b1b",
    700: "#991414",
    800: "#660e0e",
    900: "#330707",
  },
  blueAccent: {
    100: "#d4e4ff",
    200: "#a8c8ff",
    300: "#7dadff",
    400: "#5191ff",
    500: "#2676ff",
    600: "#1e5ecc",
    700: "#174799",
    800: "#0f2f66",
    900: "#081833",
  },
});

// Maritime dark theme settings
export const themeSettings = () => {
  const colors = tokens();
  return {
    palette: {
      mode: "dark",
      primary: {
        main: colors.primary[500],
      },
      secondary: {
        main: colors.greenAccent[500],
      },
      neutral: {
        dark: colors.grey[700],
        main: colors.grey[500],
        light: colors.grey[100],
      },
      background: {
        default: colors.primary[700],
        paper: colors.primary[400],
      },
      safety: {
        safe: safetyColors.safe,
        caution: safetyColors.caution,
        danger: safetyColors.danger,
      },
    },
    typography: {
      fontFamily: ["Source Sans Pro", "sans-serif"].join(","),
      fontSize: 14,
      h1: {
        fontFamily: ["Source Sans Pro", "sans-serif"].join(","),
        fontSize: 40,
        fontWeight: 700,
      },
      h2: {
        fontFamily: ["Source Sans Pro", "sans-serif"].join(","),
        fontSize: 32,
        fontWeight: 700,
      },
      h3: {
        fontFamily: ["Source Sans Pro", "sans-serif"].join(","),
        fontSize: 24,
        fontWeight: 600,
      },
      h4: {
        fontFamily: ["Source Sans Pro", "sans-serif"].join(","),
        fontSize: 20,
        fontWeight: 600,
      },
      h5: {
        fontFamily: ["Source Sans Pro", "sans-serif"].join(","),
        fontSize: 16,
      },
      h6: {
        fontFamily: ["Source Sans Pro", "sans-serif"].join(","),
        fontSize: 14,
      },
    },
  };
};

export const useMode = () => {
  const theme = useMemo(() => createTheme(themeSettings()), []);
  return theme;
};
