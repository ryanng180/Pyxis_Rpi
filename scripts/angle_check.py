#!/usr/bin/env python3
"""Quick check: find the closest scan point near ±90° and print its sign."""
import rclpy
import math
from sensor_msgs.msg import LaserScan

rclpy.init()
node = rclpy.create_node('angle_check')
msg = [None]


def cb(m):
    msg[0] = m


node.create_subscription(LaserScan, '/scan', cb, 1)

print('Waiting for /scan ...')
while msg[0] is None:
    rclpy.spin_once(node)

scan = msg[0]
print(f'angle_min={math.degrees(scan.angle_min):.1f}  angle_max={math.degrees(scan.angle_max):.1f}')
print(f'increment={math.degrees(scan.angle_increment):.3f}  points={len(scan.ranges)}')

best_r = float('inf')
best_a = 0
for i, r in enumerate(scan.ranges):
    a = math.degrees(scan.angle_min + i * scan.angle_increment)
    if 0.1 < r < 20 and 70 < abs(a) < 110:
        if r < best_r:
            best_r = r
            best_a = a

if best_r < float('inf'):
    sign = 'POSITIVE (port side)' if best_a > 0 else 'NEGATIVE (starboard side)'
    print(f'Closest point near 90deg: {best_r:.3f}m at {best_a:+.1f} deg  -->  {sign}')
else:
    print('No valid points found between 70-110 deg on either side.')

node.destroy_node()
rclpy.shutdown()
