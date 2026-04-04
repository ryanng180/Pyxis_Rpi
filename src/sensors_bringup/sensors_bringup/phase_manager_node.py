#!/usr/bin/env python3
"""
Phase Manager Node
Observes proximity data and determines the operational phase of the
pilot transfer manoeuvre. Publishes phase as a String topic.

Phases:
  APPROACHING — no hull detected or distance > 5m (cameras only)
  ZONING      — hull detected, within 1.2–5.0m, aligning with cargo
  HOLDING     — stable within OPTIMAL range (0.3–1.2m) for 5 seconds

Hysteresis prevents flickering at phase boundaries:
  - Must sustain new-phase condition for debounce period before transition
  - Asymmetric: harder to leave HOLDING (safety phase) than to enter it
  - HOLDING requires sustained OPTIMAL distance (not just crossing threshold)

Publishes:
  /system/phase  (std_msgs/String)  — current phase name

This node is a pure observer — it reads existing topics and publishes
a derived state. If it crashes, all other nodes continue unchanged.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import time


class PhaseManagerNode(Node):

    # Phase constants
    APPROACHING = 'APPROACHING'
    ZONING = 'ZONING'
    HOLDING = 'HOLDING'

    # Distance thresholds (metres)
    OPTIMAL_MIN = 0.3            # closer than this = TOO_CLOSE (danger)
    OPTIMAL_MAX = 1.2            # OPTIMAL range upper bound (with noise margin)
    APPROACHING_THRESHOLD = 5.0  # > 5.0m = APPROACHING

    # Debounce times (seconds) — asymmetric for safety
    DEBOUNCE_TO_APPROACHING = 3.0  # need 3s of >5m before going APPROACHING
    DEBOUNCE_TO_ZONING = 2.0       # need 2s of 1.2-5m before going ZONING
    DEBOUNCE_TO_HOLDING = 5.0      # need 5s sustained in OPTIMAL before HOLDING
    DEBOUNCE_EXIT_HOLDING = 3.0    # need 3s outside OPTIMAL before leaving HOLDING

    # If no distance data for this long, assume APPROACHING
    NO_DATA_TIMEOUT = 5.0

    def __init__(self):
        super().__init__('phase_manager_node')

        self._phase = self.APPROACHING
        self._candidate_phase = None
        self._candidate_start = None
        self._last_distance = None
        self._last_distance_time = 0.0

        # Subscribe to proximity data
        self.create_subscription(
            Float32, '/proximity/distance', self._distance_cb, 10)

        # Publish phase at low rate
        self._pub_phase = self.create_publisher(String, '/system/phase', 10)

        # Check phase at 2 Hz (plenty fast for debounce logic)
        self.create_timer(0.5, self._check_phase)

        # Publish initial phase immediately
        self._publish_phase()

        self.get_logger().info(
            f'Phase manager ready. '
            f'OPTIMAL={self.OPTIMAL_MIN}-{self.OPTIMAL_MAX}m (HOLDING after {self.DEBOUNCE_TO_HOLDING}s), '
            f'APPROACHING>{self.APPROACHING_THRESHOLD}m')

    def _distance_cb(self, msg):
        self._last_distance = msg.data
        self._last_distance_time = time.monotonic()

    def _in_optimal_range(self, d):
        """Is the distance within OPTIMAL range (safe boarding position)?"""
        return d is not None and self.OPTIMAL_MIN <= d <= self.OPTIMAL_MAX

    def _determine_target_phase(self):
        """What phase SHOULD we be in based on current data?"""
        now = time.monotonic()

        # No data timeout → APPROACHING
        if now - self._last_distance_time > self.NO_DATA_TIMEOUT:
            return self.APPROACHING

        d = self._last_distance
        if d is None:
            return self.APPROACHING

        if d > self.APPROACHING_THRESHOLD:
            return self.APPROACHING
        elif self._in_optimal_range(d):
            return self.HOLDING
        else:
            # Either TOO_CLOSE (<0.3m) or CLOSING/APPROACHING (1.2-5.0m)
            return self.ZONING

    def _get_debounce_time(self, target_phase):
        """How long must the target phase be sustained before transitioning?"""
        if target_phase == self.APPROACHING:
            return self.DEBOUNCE_TO_APPROACHING
        if target_phase == self.ZONING:
            # Leaving HOLDING requires longer debounce (safety)
            if self._phase == self.HOLDING:
                return self.DEBOUNCE_EXIT_HOLDING
            return self.DEBOUNCE_TO_ZONING
        if target_phase == self.HOLDING:
            return self.DEBOUNCE_TO_HOLDING
        return 2.0

    def _check_phase(self):
        """Called at 2 Hz. Manages debounced phase transitions."""
        target = self._determine_target_phase()
        now = time.monotonic()

        if target == self._phase:
            # Already in correct phase — clear any pending transition
            self._candidate_phase = None
            self._candidate_start = None
            # Still publish periodically so new subscribers get current phase
            self._publish_phase()
            return

        # Different target — start or continue debounce
        if target != self._candidate_phase:
            # New candidate — start timer
            self._candidate_phase = target
            self._candidate_start = now
            return

        # Same candidate — check if debounce elapsed
        debounce = self._get_debounce_time(target)
        elapsed = now - self._candidate_start

        if elapsed >= debounce:
            old = self._phase
            self._phase = target
            self._candidate_phase = None
            self._candidate_start = None
            self._publish_phase()
            self.get_logger().warn(
                f'PHASE CHANGE: {old} -> {self._phase}  '
                f'(distance={self._last_distance:.2f}m, '
                f'debounce={debounce:.1f}s)')

    def _publish_phase(self):
        msg = String()
        msg.data = self._phase
        self._pub_phase.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PhaseManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
