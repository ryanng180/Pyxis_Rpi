import React from "react";
import { Box } from "@mui/material";

/**
 * Top-down boat silhouette SVG.
 * Semi-transparent, centered in Camera 2 panel.
 */
const BoatIcon = ({ width = 80, opacity = 0.25 }) => {
  const aspectRatio = 3.2;
  const h = width * aspectRatio;

  return (
    <Box
      sx={{
        position: "absolute",
        top: "50%",
        left: "50%",
        transform: "translate(-50%, -50%)",
        zIndex: 4,
        pointerEvents: "none",
        opacity,
      }}
    >
      <svg
        width={width}
        height={h}
        viewBox="0 0 100 320"
        fill="none"
        xmlns="http://www.w3.org/2000/svg"
      >
        {/* Hull */}
        <path
          d="M50 10
             C55 10, 70 30, 75 60
             L80 120
             L82 200
             L80 260
             C78 280, 65 305, 50 310
             C35 305, 22 280, 20 260
             L18 200
             L20 120
             L25 60
             C30 30, 45 10, 50 10Z"
          fill="#ccc"
          stroke="#888"
          strokeWidth="2"
        />
        {/* Cabin */}
        <rect x="32" y="100" width="36" height="70" rx="6" fill="#999" />
        {/* Windshield */}
        <rect x="36" y="105" width="28" height="25" rx="3" fill="#66aacc" opacity="0.6" />
        {/* Bow marker */}
        <circle cx="50" cy="30" r="4" fill="#ffaa00" />
        {/* Stern marker */}
        <circle cx="50" cy="290" r="4" fill="#ff4444" />
        {/* Fenders (port & starboard) */}
        <ellipse cx="16" cy="160" rx="5" ry="12" fill="#cc9900" />
        <ellipse cx="84" cy="160" rx="5" ry="12" fill="#cc9900" />
        <ellipse cx="16" cy="220" rx="5" ry="12" fill="#cc9900" />
        <ellipse cx="84" cy="220" rx="5" ry="12" fill="#cc9900" />
      </svg>
    </Box>
  );
};

export default React.memo(BoatIcon);
