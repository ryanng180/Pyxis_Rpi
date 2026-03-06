import React, { useRef, useState } from "react";
import { Box, Button, Typography, Grid } from "@mui/material";
import Header from "../../components/Header";

const SingleCamera = ({ label }) => {
  const videoRef = useRef(null);
  const streamRef = useRef(null);
  const [isOn, setIsOn] = useState(false);

  const startCamera = async () => {
    const stream = await navigator.mediaDevices.getUserMedia({
      video: true,
      audio: false,
    });

    streamRef.current = stream;
    videoRef.current.srcObject = stream;
    await videoRef.current.play();
    setIsOn(true);
  };

  const stopCamera = () => {
    const stream = streamRef.current;
    if (stream) stream.getTracks().forEach((t) => t.stop());
    setIsOn(false);
  };

  return (
    <Box>
      <Typography variant="h5" mb={1}>
        {label}
      </Typography>

      <Box
        sx={{
          width: "100%",
          aspectRatio: "16 / 9",
          borderRadius: 2,
          overflow: "hidden",
          bgcolor: "rgba(0,0,0,0.15)",
        }}
      >
        <video
          ref={videoRef}
          playsInline
          muted
          style={{ width: "100%", height: "100%", objectFit: "cover" }}
        />
      </Box>

      <Box mt={2}>
        <Button variant="contained" onClick={startCamera} disabled={isOn}>
          Start
        </Button>
        <Button variant="outlined" onClick={stopCamera} sx={{ ml: 1 }}>
          Stop
        </Button>
      </Box>
    </Box>
  );
};

const Camera = () => {
  return (
    <Box m="20px">
      <Header title="CAMERA" subtitle="Two camera layout" />

      <Grid container spacing={3}>
        <Grid item xs={12} md={6}>
          <SingleCamera label="Camera 1" />
        </Grid>

        <Grid item xs={12} md={6}>
          <SingleCamera label="Camera 2" />
        </Grid>
      </Grid>
    </Box>
  );
};

export default Camera;