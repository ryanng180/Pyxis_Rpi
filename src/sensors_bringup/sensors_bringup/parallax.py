#!/usr/bin/env python3
"""
Parallax correction utility.
Converts gimbal yaw angle to the correct LiDAR angle,
accounting for physical offset between gimbal and LiDAR on mount.

Mount geometry (top view):
  Drawing convention:  x = starboard (right), y = forward
  ROS convention:      x = forward,           y = left (port)

  From mount centre (all mm):
    Gimbal:    x=-34.15, y=0       (port side of mount)
    LiDAR+IMU: x=+45.7,  y=-11.61 (starboard, slightly aft)
                LiDAR Z = 40mm below mount, IMU Z = 0

  LiDAR relative to Gimbal:
    starboard offset = 45.7 - (-34.15) = 79.85 mm
    aft offset       = -11.61 mm

  In ROS frame (x=forward, y=left):
    LIDAR_OFFSET_X = -0.01161  (LiDAR is 11.61mm behind gimbal)
    LIDAR_OFFSET_Y = -0.07985  (LiDAR is 79.85mm to starboard of gimbal)

Key vessel positions (metres, from mount centre):
  Drawing (x=stbd, y=fwd)        ROS (x=fwd, y=port)
  Pilot boarding:  (0, 2.2)      (2.20, 0)
  Ferry bow:       (-1.22, 2.94) (2.94, 1.22)
  Starboard hull:  X = 1.123     y = -1.123
  Hull centre:     (1.123, -4.566)  (-4.566, -1.123)
  Bow transition:  (0, 0.987)    (0.987, 0)   hull curves at -12° CCW
  Hull rear:       (1.123, -12.066) (-12.066, -1.123)  7.5m aft of hull centre
"""

import math

# ── Mount offsets: LiDAR position relative to Gimbal (ROS frame) ──────────
# x = forward (+) / aft (-)
# y = port/left (+) / starboard/right (-)
LIDAR_OFFSET_X = -0.01161   # 11.61mm aft of gimbal
LIDAR_OFFSET_Y = -0.07985   # 79.85mm starboard of gimbal

# ── LiDAR position relative to mount centre (ROS frame) ──────────────────
LIDAR_MOUNT_X = -0.01161    # 11.61mm aft of mount centre
LIDAR_MOUNT_Y = -0.0457     # 45.7mm starboard of mount centre

# ── Vessel reference points (from mount centre, ROS frame) ────────────────
PILOT_BOARDING_X =  2.20    # pilot boarding gate, 2.2m forward of mount
PILOT_BOARDING_Y =  0.00    # on mount centreline
STARBOARD_HULL_Y = -1.123   # starboard hull line, 1.123m to starboard

# ── Starboard hull extent (ROS frame, x = forward) ───────────────────────
HULL_BOW_TRANSITION_X =   0.987   # where hull curves into bow
HULL_REAR_X           = -12.066   # rear of straight hull (7.5m aft of hull centre)
HULL_LENGTH_M         =  13.053   # bow transition to rear

# ── Hull offset: perpendicular distance from LiDAR to hull edge ──────────
HULL_OFFSET_M = abs(STARBOARD_HULL_Y) - abs(LIDAR_MOUNT_Y)  # ~1.077m

# ── Scan arc (degrees, in LiDAR scan convention) ────────────────────────
# LiDAR mounted upside-down with inverted:True in driver.
# Actual scan convention: 0°=bow, +90°=starboard, +180°=stern, -90°=port.
# Full starboard arc: 0° to 180°
SCAN_ARC_MIN_DEG =    0.0   # forward (bow)
SCAN_ARC_MAX_DEG =  180.0   # aft (through starboard)


def get_lidar_angle(gimbal_yaw_deg: float,
                    initial_dist_m: float,
                    max_iter: int = 10,
                    tol_m: float = 0.001) -> float:
    """
    Convert gimbal yaw angle to corrected LiDAR angle.
    Iterates until position converges.

    Args:
        gimbal_yaw_deg:  Gimbal pan in degrees (0=forward, +=port/left)
        initial_dist_m:  Starting distance estimate (metres)
        max_iter:        Max iterations
        tol_m:           Convergence tolerance

    Returns:
        Corrected LiDAR angle in degrees
    """
    theta_g = math.radians(gimbal_yaw_deg)
    d = initial_dist_m

    for _ in range(max_iter):
        # Target position from gimbal origin (ROS frame)
        px = d * math.cos(theta_g)
        py = d * math.sin(theta_g)

        # Vector from LiDAR to target
        dx = px - LIDAR_OFFSET_X
        dy = py - LIDAR_OFFSET_Y

        theta_l = math.atan2(dy, dx)
        d_new   = math.sqrt(dx*dx + dy*dy)

        if abs(d_new - d) < tol_m:
            return math.degrees(theta_l)
        d = d_new

    return math.degrees(theta_l)


def get_ladder_distance(gimbal_yaw_deg: float,
                        lidar_range_m: float) -> dict:
    """
    Full parallax-corrected ladder measurement.

    Args:
        gimbal_yaw_deg:  YOLO gimbal pan angle (degrees)
        lidar_range_m:   LiDAR range at the corrected angle (metres)

    Returns dict:
        lidar_angle_deg:         corrected LiDAR angle to query
        ladder_dist_m:           slant distance from LiDAR to ladder
        dist_from_mount_m:       distance from mount centre to ladder
        dist_to_boarding_gate_m: distance from pilot boarding gate to ladder
        lateral_from_hull_m:     how far ladder is from starboard hull
        angle_error_deg:         parallax correction applied (diagnostic)
    """
    if lidar_range_m <= 0:
        return {
            'lidar_angle_deg':          0.0,
            'ladder_dist_m':           -1.0,
            'dist_from_mount_m':       -1.0,
            'dist_to_boarding_gate_m': -1.0,
            'lateral_from_hull_m':      0.0,
            'angle_error_deg':          0.0,
        }

    corrected_angle = get_lidar_angle(gimbal_yaw_deg, lidar_range_m)
    angle_error     = corrected_angle - gimbal_yaw_deg

    # Ladder absolute position from mount centre (ROS frame)
    theta_l = math.radians(corrected_angle)
    # LiDAR origin in mount frame
    lx = LIDAR_OFFSET_X
    ly = LIDAR_OFFSET_Y
    # Ladder position in mount frame
    ladder_x = lx + lidar_range_m * math.cos(theta_l)
    ladder_y = ly + lidar_range_m * math.sin(theta_l)

    # Distance from mount centre
    dist_from_mount = math.sqrt(ladder_x**2 + ladder_y**2)

    # Distance from pilot boarding gate to ladder
    dx_gate = ladder_x - PILOT_BOARDING_X
    dy_gate = ladder_y - PILOT_BOARDING_Y
    dist_to_gate = math.sqrt(dx_gate**2 + dy_gate**2)

    # How far the ladder is from the starboard hull (laterally)
    lateral_from_hull = abs(ladder_y - STARBOARD_HULL_Y)

    return {
        'lidar_angle_deg':          round(corrected_angle, 3),
        'ladder_dist_m':            round(lidar_range_m, 3),
        'dist_from_mount_m':        round(dist_from_mount, 3),
        'dist_to_boarding_gate_m':  round(dist_to_gate, 3),
        'lateral_from_hull_m':      round(lateral_from_hull, 3),
        'angle_error_deg':          round(angle_error, 4),
    }


# ── Sanity test ───────────────────────────────────────────────────────────
if __name__ == '__main__':
    print("Parallax correction — updated geometry")
    print(f"  Gimbal:  mount origin")
    print(f"  LiDAR:   {LIDAR_OFFSET_X*1000:.2f}mm fwd,  {LIDAR_OFFSET_Y*1000:.2f}mm lateral")
    print(f"  Boarding gate: {PILOT_BOARDING_X}m fwd, {PILOT_BOARDING_Y}m lateral")
    print(f"  Starboard hull: {STARBOARD_HULL_Y}m lateral\n")

    print(f"{'Case':<38} {'Gimbal':>8} {'Corrected':>10} {'Error':>8} "
          f"{'LiDAR dist':>11} {'→Gate':>8}")
    print("─" * 85)

    test_cases = [
        ( -12.0, 10.0, "Ladder fwd-stbd 10m (arc edge)"),
        ( -45.0, 10.0, "Ladder 45° stbd, 10m"),
        ( -90.0, 10.0, "Ladder abeam stbd, 10m"),
        ( -90.0,  5.0, "Ladder abeam stbd, 5m"),
        ( -90.0,  3.0, "Ladder abeam stbd, 3m (close)"),
        (-110.0, 10.0, "Ladder 110° stbd (arc edge)"),
        ( -45.0,  3.0, "Ladder 45° stbd, 3m (close)"),
    ]

    for gimbal_deg, dist, label in test_cases:
        r = get_ladder_distance(gimbal_deg, dist)
        print(f"{label:<38} {gimbal_deg:>7.1f}° {r['lidar_angle_deg']:>9.3f}° "
              f"{r['angle_error_deg']:>7.3f}° {r['ladder_dist_m']:>10.2f}m "
              f"{r['dist_to_boarding_gate_m']:>7.2f}m")

    print("\nBoarding gate distances for reference:")
    print(f"  Ladder at abeam 5m  → gate distance: "
          f"{get_ladder_distance(-90.0, 5.0)['dist_to_boarding_gate_m']:.3f}m")
    print(f"  Ladder at abeam 3m  → gate distance: "
          f"{get_ladder_distance(-90.0, 3.0)['dist_to_boarding_gate_m']:.3f}m")
    print(f"  Ladder at 45° 5m    → gate distance: "
          f"{get_ladder_distance(-45.0, 5.0)['dist_to_boarding_gate_m']:.3f}m")
