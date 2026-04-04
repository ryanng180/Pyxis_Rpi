import React from "react";
import { Box, Typography, Button } from "@mui/material";
import { PYXIS_ACCENT } from "../config";

/**
 * System diagnostics view — engineering/debug screen.
 * Shows all raw sensor values, subsystem status, and data quality metrics.
 */

const DiagCard = ({ title, children, span = 1 }) => (
  <Box
    sx={{
      gridColumn: `span ${span}`,
      backgroundColor: "#0d1117",
      border: "1px solid #1a2035",
      borderRadius: "6px",
      p: 2,
    }}
  >
    <Typography
      sx={{
        fontSize: "13px",
        fontWeight: 700,
        color: PYXIS_ACCENT,
        letterSpacing: "2px",
        mb: 1.5,
        textTransform: "uppercase",
      }}
    >
      {title}
    </Typography>
    {children}
  </Box>
);

const DiagRow = ({ label, value, unit, color }) => (
  <Box
    sx={{
      display: "flex",
      justifyContent: "space-between",
      alignItems: "baseline",
      py: 0.4,
      borderBottom: "1px solid #1a203520",
    }}
  >
    <Typography sx={{ fontSize: "13px", color: "#889" }}>{label}</Typography>
    <Box sx={{ display: "flex", alignItems: "baseline", gap: 0.5 }}>
      <Typography
        sx={{
          fontFamily: "'JetBrains Mono', monospace",
          fontSize: "15px",
          fontWeight: 600,
          color: color || "#e0e0e0",
        }}
      >
        {value ?? "---"}
      </Typography>
      {unit && (
        <Typography sx={{ fontSize: "12px", color: "#667" }}>{unit}</Typography>
      )}
    </Box>
  </Box>
);

const StatusDot = ({ active, label }) => (
  <Box sx={{ display: "flex", alignItems: "center", gap: 1, py: 0.3 }}>
    <Box
      sx={{
        width: 10,
        height: 10,
        borderRadius: "50%",
        backgroundColor: active ? "#00ff88" : "#ff2222",
        boxShadow: active ? "0 0 6px #00ff8860" : "0 0 6px #ff222260",
      }}
    />
    <Typography sx={{ fontSize: "14px", color: active ? "#00ff88" : "#ff2222", fontWeight: 600 }}>
      {label}
    </Typography>
  </Box>
);

const ZONE_COLORS = {
  TOO_CLOSE: "#ff2222",
  OPTIMAL: "#00ff88",
  TOO_FAR: "#ffaa00",
};

const MODEL_OPTIONS = [
  { id: "y11m_t0_960",        label: "YOLOv11m T0 960",       deprecated: true },
  { id: "combined_y11s_1280", label: "YOLOv11s Combined 1280", deprecated: false },
  { id: "sg_y11s_960",        label: "YOLOv11s SG 960",        deprecated: false },
];

const DiagnosticsView = ({
  connected = false,
  distance = null,
  zone = null,
  proximityStatus = null,
  detections = [],
  gimbalYaw = null,
  gimbalPitch = null,
  ladderDistance = null,
  ladderStatus = null,
  approachHeading = null,
  rawHeading = null,
  headingOffset = 0,
  approachZone = null,
  approachProfile = null,
  boatPitch = null,
  boatRoll = null,
  lastUpdate = null,
  currentModel = null,
  onSwitchModel,
  onZeroHeading,
  onResetHeading,
}) => {
  const ladderDetected = detections.some((d) => d.label?.toLowerCase() === "ladder");

  return (
    <Box
      sx={{
        flex: 1,
        display: "grid",
        gridTemplateColumns: "repeat(4, 1fr)",
        gap: 1.5,
        p: 2,
        overflow: "auto",
        minHeight: 0,
      }}
    >
      {/* System Status */}
      <DiagCard title="System Status">
        <StatusDot active={connected} label={connected ? "WebSocket Connected" : "WebSocket Disconnected"} />
        <StatusDot active={distance !== null} label={distance !== null ? "LiDAR Active" : "LiDAR No Data"} />
        <StatusDot active={ladderDetected} label={ladderDetected ? "Ladder Detected" : "Ladder Not Detected"} />
        <StatusDot active={gimbalYaw !== null} label={gimbalYaw !== null ? "Gimbal Online" : "Gimbal Offline"} />
        <StatusDot active={boatPitch !== null} label={boatPitch !== null ? "IMU Active" : "IMU No Data"} />
        <StatusDot active={approachProfile !== null} label={approachProfile ? "Approach Profile" : "No Profile"} />
        {lastUpdate && (
          <Typography sx={{ fontSize: "12px", color: "#556", mt: 1 }}>
            Last: {new Date(lastUpdate * 1000).toLocaleTimeString("en-GB")}
          </Typography>
        )}
      </DiagCard>

      {/* Proximity / LiDAR */}
      <DiagCard title="Proximity (LiDAR)">
        <DiagRow
          label="Hull Distance"
          value={distance?.toFixed(3)}
          unit="m"
          color={ZONE_COLORS[zone]}
        />
        <DiagRow label="Zone" value={zone} color={ZONE_COLORS[zone]} />
        <DiagRow
          label="Raw LiDAR"
          value={proximityStatus?.raw_lidar_m?.toFixed(3)}
          unit="m"
        />
        <DiagRow
          label="Hull Offset"
          value={proximityStatus?.hull_offset_m?.toFixed(3)}
          unit="m"
        />
        <DiagRow
          label="Point Count"
          value={proximityStatus?.point_count}
        />
        <DiagRow
          label="Too Close Threshold"
          value={proximityStatus?.too_close_m?.toFixed(2)}
          unit="m"
          color="#ff222288"
        />
        <DiagRow
          label="Too Far Threshold"
          value={proximityStatus?.too_far_m?.toFixed(2)}
          unit="m"
          color="#ffaa0088"
        />
      </DiagCard>

      {/* Gimbal */}
      <DiagCard title="Gimbal (STorM32)">
        <DiagRow label="Yaw" value={gimbalYaw?.toFixed(1)} unit="°" color={PYXIS_ACCENT} />
        <DiagRow label="Pitch" value={gimbalPitch?.toFixed(1)} unit="°" color={PYXIS_ACCENT} />
      </DiagCard>

      {/* IMU / Boat Tilt */}
      <DiagCard title="IMU / Boat Tilt">
        <DiagRow label="Roll" value={boatRoll?.toFixed(2)} unit="°" />
        <DiagRow label="Pitch" value={boatPitch?.toFixed(2)} unit="°" />
      </DiagCard>

      {/* Ladder Distance */}
      <DiagCard title="Ladder (Parallax-Corrected)">
        <DiagRow
          label="Boarding Gate Dist"
          value={ladderDistance?.toFixed(3)}
          unit="m"
          color={PYXIS_ACCENT}
        />
        <DiagRow
          label="From Mount"
          value={ladderStatus?.dist_from_mount_m?.toFixed(3)}
          unit="m"
        />
        <DiagRow
          label="LiDAR Range"
          value={ladderStatus?.ladder_dist_m?.toFixed(3)}
          unit="m"
        />
        <DiagRow
          label="Lateral from Hull"
          value={ladderStatus?.lateral_from_hull_m?.toFixed(3)}
          unit="m"
        />
        <DiagRow
          label="Gimbal Yaw"
          value={ladderStatus?.gimbal_yaw_deg?.toFixed(2)}
          unit="°"
        />
        <DiagRow
          label="LiDAR Angle"
          value={ladderStatus?.lidar_angle_deg?.toFixed(2)}
          unit="°"
        />
        <DiagRow
          label="Parallax Error"
          value={ladderStatus?.parallax_error_deg?.toFixed(3)}
          unit="°"
        />
      </DiagCard>

      {/* Approach / Hull Profile */}
      <DiagCard title="Cargo Approach" span={2}>
        <DiagRow
          label="Heading (corrected)"
          value={approachHeading?.toFixed(2)}
          unit="°"
          color={approachHeading !== null
            ? Math.abs(approachHeading) < 3 ? "#00ff88" : Math.abs(approachHeading) < 8 ? "#ffaa00" : "#ff2222"
            : undefined}
        />
        <DiagRow label="Raw Heading" value={rawHeading?.toFixed(2)} unit="°" color="#889" />
        <DiagRow label="Offset" value={headingOffset.toFixed(2)} unit="°" color={headingOffset !== 0 ? "#dd8800" : "#889"} />
        <Box sx={{ display: "flex", gap: 1, mt: 1, mb: 1 }}>
          <Button
            variant="outlined"
            size="small"
            onClick={onZeroHeading}
            disabled={rawHeading === null}
            sx={{
              color: PYXIS_ACCENT,
              borderColor: PYXIS_ACCENT + "66",
              fontSize: "12px",
              fontWeight: 700,
              letterSpacing: "1px",
              "&:hover": { borderColor: PYXIS_ACCENT, backgroundColor: PYXIS_ACCENT + "15" },
              "&:disabled": { color: "#556", borderColor: "#334" },
            }}
          >
            ZERO HEADING
          </Button>
          {headingOffset !== 0 && (
            <Button
              variant="outlined"
              size="small"
              onClick={onResetHeading}
              sx={{
                color: "#dd8800",
                borderColor: "#dd880066",
                fontSize: "12px",
                fontWeight: 700,
                letterSpacing: "1px",
                "&:hover": { borderColor: "#dd8800", backgroundColor: "#dd880015" },
              }}
            >
              RESET
            </Button>
          )}
        </Box>
        <DiagRow
          label="Overall Zone"
          value={approachZone}
          color={ZONE_COLORS[approachZone]}
        />
        <DiagRow
          label="Closest Gap"
          value={approachProfile?.closest_gap_m?.toFixed(3)}
          unit="m"
          color={ZONE_COLORS[approachProfile?.closest_zone]}
        />
        <DiagRow
          label="Fit R²"
          value={approachProfile?.fit_r_squared?.toFixed(4)}
        />
        <DiagRow
          label="Line Length"
          value={approachProfile?.line_length_m?.toFixed(2)}
          unit="m"
        />
        <DiagRow
          label="Point Count"
          value={approachProfile?.point_count}
        />
        <DiagRow
          label="Cargo Detected"
          value={approachProfile?.cargo_detected ? "YES" : "NO"}
          color={approachProfile?.cargo_detected ? "#00ff88" : "#ff2222"}
        />
      </DiagCard>

      {/* CV Detections */}
      <DiagCard title="CV Detections">
        <DiagRow label="Count" value={detections.length} />
        {detections.slice(0, 5).map((d, i) => (
          <DiagRow
            key={i}
            label={d.label}
            value={`${(d.confidence * 100).toFixed(0)}%`}
            color={d.label === "ladder" ? "#00ff88" : "#ffaa00"}
          />
        ))}
        {detections.length === 0 && (
          <Typography sx={{ fontSize: "13px", color: "#556", mt: 0.5 }}>
            No detections
          </Typography>
        )}
      </DiagCard>

      {/* YOLO Model Selector */}
      <DiagCard title="Inference Model">
        <DiagRow
          label="Active Model"
          value={currentModel || "unknown"}
          color={PYXIS_ACCENT}
        />
        <Box sx={{ display: "flex", flexDirection: "column", gap: 1, mt: 1.5 }}>
          {MODEL_OPTIONS.map((m) => {
            const isActive = currentModel === m.id;
            const color = m.deprecated ? "#ff222288" : "#00ff88";
            return (
              <Button
                key={m.id}
                variant="outlined"
                size="small"
                disabled={isActive}
                onClick={() => onSwitchModel?.(m.id)}
                sx={{
                  justifyContent: "flex-start",
                  textTransform: "none",
                  color: isActive ? color : "#aab",
                  borderColor: isActive ? color + "88" : "#334",
                  backgroundColor: isActive ? color + "12" : "transparent",
                  fontSize: "12px",
                  fontWeight: 600,
                  py: 0.8,
                  "&:hover": {
                    borderColor: color,
                    backgroundColor: color + "15",
                  },
                  "&:disabled": {
                    color: color,
                    borderColor: color + "66",
                    backgroundColor: color + "12",
                  },
                }}
              >
                {m.label}
                {m.deprecated && (
                  <Typography
                    component="span"
                    sx={{
                      fontSize: "10px",
                      fontWeight: 700,
                      color: "#ff2222",
                      ml: 1,
                      letterSpacing: "1px",
                    }}
                  >
                    DEPRECATED
                  </Typography>
                )}
                {isActive && (
                  <Typography
                    component="span"
                    sx={{
                      fontSize: "10px",
                      fontWeight: 700,
                      color,
                      ml: 1,
                      letterSpacing: "1px",
                    }}
                  >
                    ACTIVE
                  </Typography>
                )}
              </Button>
            );
          })}
        </Box>
      </DiagCard>
    </Box>
  );
};

export default React.memo(DiagnosticsView);
