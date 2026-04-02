#!/usr/bin/env python3
"""Compare /scan/filtered vs /scan/transformed at a fixed index to test tilt correction."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class TiltTest(Node):
    def __init__(self):
        super().__init__('tilt_test')
        self._filtered = None
        self._transformed = None
        self.create_subscription(LaserScan, '/scan/filtered', self._filt_cb, 10)
        self.create_subscription(LaserScan, '/scan/transformed', self._trans_cb, 10)
        self.create_timer(0.5, self._print)

    def _closest(self, msg):
        valid = [(i, r) for i, r in enumerate(msg.ranges)
                 if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        return min(valid, key=lambda x: x[1]) if valid else None

    def _filt_cb(self, msg):
        self._filtered = self._closest(msg)

    def _trans_cb(self, msg):
        self._transformed = self._closest(msg)

    def _print(self):
        f = self._filtered
        t = self._transformed
        if f and t:
            diff = t[1] - f[1]
            print(f'filtered={f[1]:.3f}  transformed={t[1]:.3f}  diff={diff:+.3f}')


def main():
    rclpy.init()
    node = TiltTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
