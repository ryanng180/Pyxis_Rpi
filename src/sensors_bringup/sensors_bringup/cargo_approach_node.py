#!/usr/bin/env python3
"""
Cargo Approach Node

Measures hull-to-cargo gap using direct angle-to-hull mapping.
Each LiDAR ray geometrically maps to a specific position along the
ferry's starboard hull. The gap is measured per-ray — no extrapolation.

Algorithm:
  1. For each valid scan point beyond the ferry hull:
     a. Compute mount-frame position (mx, my)
     b. Perpendicular gap = |my - hull_y|
     c. Project ray back to hull line to find hull position
     d. Bin by 1m hull segments
  2. Per-bin median gap → zone classification
  3. Line fit on cargo points → heading error (only use)

Subscribes:
  /scan/filtered  (sensor_msgs/LaserScan) — ferry-frame scan

Publishes:
  /approach/heading  (Float32) — heading error in degrees (0° = parallel)
  /approach/closest  (Float32) — closest hull-to-hull gap in metres
  /approach/zone     (String)  — overall zone based on closest gap
  /approach/profile  (String)  — full JSON with profile, heading, fit quality
"""

import math
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String


class CargoApproachNode(Node):

    def __init__(self):
        super().__init__('cargo_approach_node')

        # ── Ferry hull geometry (ROS frame: x=fwd, y=port) ───────────
        # Hull is a straight line at y = -hull_starboard_m
        self.declare_parameter('hull_starboard_m', 1.123)
        self.declare_parameter('hull_bow_x',       0.987)    # forward extent
        self.declare_parameter('hull_rear_x',     -12.066)   # aft extent

        # LiDAR position in mount frame (ROS: x=fwd, y=port)
        self.declare_parameter('lidar_x', -0.01161)
        self.declare_parameter('lidar_y', -0.0457)

        # Profile and zone params
        self.declare_parameter('profile_step_m', 0.5)
        self.declare_parameter('too_close_m',    0.3)
        self.declare_parameter('too_far_m',      1.0)

        # Heading fit thresholds
        self.declare_parameter('min_points',      10)
        self.declare_parameter('max_residual_m',  0.15)

        self.hull_stbd   = self.get_parameter('hull_starboard_m').value
        self.hull_bow_x  = self.get_parameter('hull_bow_x').value
        self.hull_rear_x = self.get_parameter('hull_rear_x').value
        self.lidar_x     = self.get_parameter('lidar_x').value
        self.lidar_y     = self.get_parameter('lidar_y').value
        self.step        = self.get_parameter('profile_step_m').value
        self.too_close   = self.get_parameter('too_close_m').value
        self.too_far     = self.get_parameter('too_far_m').value
        self.min_pts     = self.get_parameter('min_points').value
        self.max_resid   = self.get_parameter('max_residual_m').value

        # Hull line in ROS frame: y = -hull_stbd (constant y, runs along x)
        self.hull_y = -self.hull_stbd

        # Number of 1m bins along the hull
        hull_len = self.hull_bow_x - self.hull_rear_x
        self.n_bins = int(hull_len / self.step) + 1

        self._sub = self.create_subscription(
            LaserScan, '/scan/filtered', self._scan_cb, 10)

        self._pub_heading = self.create_publisher(Float32, '/approach/heading', 10)
        self._pub_closest = self.create_publisher(Float32, '/approach/closest', 10)
        self._pub_zone    = self.create_publisher(String,  '/approach/zone',    10)
        self._pub_profile = self.create_publisher(String,  '/approach/profile', 10)

        self._last_zone = None
        self._dbg_count = 0
        self.get_logger().info(
            f'Cargo approach node ready (direct mapping). '
            f'Hull y={self.hull_y:.3f}m, '
            f'x=[{self.hull_rear_x:.1f}, {self.hull_bow_x:.1f}] ({hull_len:.1f}m), '
            f'{self.n_bins} bins')

    def _scan_cb(self, msg: LaserScan):
        # ── Step 1: Per-ray gap measurement + binning ────────────────
        bins = [[] for _ in range(self.n_bins)]
        cargo_points = []  # for heading line fit

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue

            angle = msg.angle_min + i * msg.angle_increment

            # Point in laser_frame → mount frame (180° yaw TF: negate both)
            lx = r * math.cos(angle)
            ly = r * math.sin(angle)
            mx = -lx + self.lidar_x
            my = -ly + self.lidar_y

            # Only keep points beyond the ferry hull (starboard side)
            if my >= self.hull_y:
                continue

            cargo_points.append((mx, my))

            # Perpendicular gap from cargo point to hull line
            gap = abs(my - self.hull_y)

            # Which hull position? Project ray from LiDAR through this
            # point back to the hull line (y = hull_y).
            # Ray: P = LiDAR + t * (point - LiDAR)
            # At y = hull_y: t = (hull_y - lidar_y) / (my - lidar_y)
            dy_point = my - self.lidar_y
            if abs(dy_point) < 1e-6:
                continue
            t = (self.hull_y - self.lidar_y) / dy_point
            x_hull = self.lidar_x + t * (mx - self.lidar_x)

            # Position along hull (0 = bow transition, increases toward stern)
            pos = self.hull_bow_x - x_hull
            bin_idx = int(pos / self.step)
            if 0 <= bin_idx < self.n_bins:
                bins[bin_idx].append(gap)

        # Debug logging
        self._dbg_count += 1
        if self._dbg_count % 20 == 1:
            filled = sum(1 for b in bins if b)
            self.get_logger().info(
                f'DEBUG: {len(cargo_points)} cargo pts, '
                f'{filled}/{self.n_bins} bins with data')

        # ── Step 2: Build profile from bins ──────────────────────────
        profile = []
        closest_gap = float('inf')
        has_data = False

        for i in range(self.n_bins):
            pos_along_hull = round(i * self.step, 1)

            if not bins[i]:
                profile.append({
                    'position_m': pos_along_hull,
                    'gap_m':      -1.0,
                    'zone':       'NO_DATA',
                })
                continue

            has_data = True
            # Median gap for this bin
            sorted_gaps = sorted(bins[i])
            median_gap = sorted_gaps[len(sorted_gaps) // 2]

            if median_gap < self.too_close:
                zone = 'TOO_CLOSE'
            elif median_gap > self.too_far:
                zone = 'TOO_FAR'
            else:
                zone = 'OPTIMAL'

            profile.append({
                'position_m': pos_along_hull,
                'gap_m':      round(median_gap, 3),
                'zone':       zone,
            })

            if median_gap < closest_gap:
                closest_gap = median_gap

        if not has_data:
            self._publish_no_detection()
            return

        # Overall zone
        if closest_gap < self.too_close:
            overall_zone = 'TOO_CLOSE'
        elif closest_gap > self.too_far:
            overall_zone = 'TOO_FAR'
        else:
            overall_zone = 'OPTIMAL'

        # ── Step 3: Heading from line fit ─────────────────────────────
        heading_error = 0.0
        r_squared = 0.0
        line_length = 0.0

        if len(cargo_points) >= self.min_pts:
            n = len(cargo_points)
            sx = sy = sxx = sxy = syy = 0.0
            for px, py in cargo_points:
                sx  += px
                sy  += py
                sxx += px * px
                sxy += px * py
                syy += py * py

            denom = n * sxx - sx * sx
            if abs(denom) > 1e-12:
                slope     = (n * sxy - sx * sy) / denom
                intercept = (sy - slope * sx) / n

                # Mean perpendicular residual
                a_fit = slope
                b_fit = -1.0
                c_fit = intercept
                norm_fit = math.sqrt(a_fit * a_fit + b_fit * b_fit)
                sum_perp = 0.0
                for px, py in cargo_points:
                    sum_perp += abs(a_fit * px + b_fit * py + c_fit)
                mean_residual = sum_perp / (n * norm_fit)

                if mean_residual <= self.max_resid:
                    heading_error = math.degrees(math.atan(slope))

                    # R² for diagnostics
                    y_mean = sy / n
                    ss_tot = syy - n * y_mean * y_mean
                    ss_res = 0.0
                    for px, py in cargo_points:
                        ss_res += (py - (slope * px + intercept)) ** 2
                    r_squared = 1.0 - (ss_res / ss_tot) if abs(ss_tot) > 1e-12 else 0.0

            xs = [p[0] for p in cargo_points]
            line_length = max(xs) - min(xs)

        # ── Step 4: Publish ───────────────────────────────────────────
        heading_msg = Float32()
        heading_msg.data = heading_error
        self._pub_heading.publish(heading_msg)

        closest_msg = Float32()
        closest_msg.data = closest_gap
        self._pub_closest.publish(closest_msg)

        zone_msg = String()
        zone_msg.data = overall_zone
        self._pub_zone.publish(zone_msg)

        if overall_zone != self._last_zone:
            self.get_logger().warn(
                f'ZONE: {self._last_zone} -> {overall_zone}  '
                f'closest={closest_gap:.2f}m  heading={heading_error:.1f}°')
            self._last_zone = overall_zone

        status = {
            'cargo_detected':    True,
            'heading_error_deg': round(heading_error, 2),
            'closest_gap_m':     round(closest_gap, 3),
            'closest_zone':      overall_zone,
            'fit_r_squared':     round(r_squared, 3),
            'line_length_m':     round(line_length, 2),
            'point_count':       len(cargo_points),
            'hull_profile':      profile,
        }
        profile_msg = String()
        profile_msg.data = json.dumps(status)
        self._pub_profile.publish(profile_msg)

    def _publish_no_detection(self):
        """Publish empty state when cargo hull is not detected."""
        status = {
            'cargo_detected':    False,
            'heading_error_deg': 0.0,
            'closest_gap_m':     -1.0,
            'closest_zone':      'NO_DETECTION',
            'fit_r_squared':     0.0,
            'line_length_m':     0.0,
            'point_count':       0,
            'hull_profile':      [],
        }
        profile_msg = String()
        profile_msg.data = json.dumps(status)
        self._pub_profile.publish(profile_msg)

        zone_msg = String()
        zone_msg.data = 'NO_DETECTION'
        self._pub_zone.publish(zone_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CargoApproachNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
