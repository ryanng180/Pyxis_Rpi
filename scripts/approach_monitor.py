#!/usr/bin/env python3
"""
Live approach monitor — pretty-prints the cargo approach profile.
Run: python3 ~/ros2_ws/scripts/approach_monitor.py
"""

import json
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


ZONE_COLORS = {
    'TOO_CLOSE':    '\033[91m',  # red
    'OPTIMAL':      '\033[92m',  # green
    'TOO_FAR':      '\033[91m',  # red
    'NO_DETECTION': '\033[90m',  # grey
    'NO_DATA':      '\033[90m',  # grey
}
RESET = '\033[0m'
BOLD  = '\033[1m'


def bar(gap_m, max_m=2.0, width=20):
    filled = int(min(gap_m / max_m, 1.0) * width)
    return '\u2588' * filled + '\u2591' * (width - filled)


class ApproachMonitor(Node):
    def __init__(self):
        super().__init__('approach_monitor')
        self.create_subscription(
            String, '/approach/profile', self.cb, 10)
        self.get_logger().info('Monitoring /approach/profile ...')

    def cb(self, msg):
        data = json.loads(msg.data)

        os.system('clear')

        detected = data.get('cargo_detected', False)
        if not detected:
            print(f'{BOLD}Cargo: {ZONE_COLORS["NO_DETECTION"]}NOT DETECTED{RESET}')
            print('Waiting for cargo ship hull in scan...')
            return

        heading  = data.get('heading_error_deg', 0)
        closest  = data.get('closest_gap_m', -1)
        zone     = data.get('closest_zone', '?')
        r2       = data.get('fit_r_squared', 0)
        pts      = data.get('point_count', 0)
        line_len = data.get('line_length_m', 0)
        profile  = data.get('hull_profile', [])

        zc = ZONE_COLORS.get(zone, '')

        if abs(heading) < 1e-6 and r2 < 0.01:
            heading_str = '  N/A (insufficient line fit)'
        else:
            heading_str = f'{heading:+.1f}\u00b0 from parallel'

        print(f'{BOLD}=== CARGO APPROACH MONITOR ==={RESET}')
        print()
        print(f'  Heading:  {heading_str}')
        print(f'  Closest:  {zc}{BOLD}{closest:.3f}m  [{zone}]{RESET}')
        print(f'  Fit:      R\u00b2={r2:.3f}  points={pts}  line={line_len:.1f}m')
        print()
        print(f'{BOLD}  {"Pos":>5}  {"Gap":>6}  {"Bar":<22} Zone{RESET}')
        print(f'  {"---":>5}  {"---":>6}  {"---":<22} ----')

        # Collapse consecutive NO_DATA into a single summary line
        i = 0
        while i < len(profile):
            seg = profile[i]
            sz  = seg['zone']
            if sz == 'NO_DATA':
                start = seg['position_m']
                end = start
                while i < len(profile) and profile[i]['zone'] == 'NO_DATA':
                    end = profile[i]['position_m']
                    i += 1
                sc = ZONE_COLORS.get('NO_DATA', '')
                if start == end:
                    print(f'  {start:5.1f}m  {"---":>5}   {sc}{"·" * 20}{RESET}  {sc}NO_DATA{RESET}')
                else:
                    print(f'  {start:.1f}-{end:.1f}m {"---":>3}   {sc}{"·" * 20}{RESET}  {sc}NO_DATA{RESET}')
            else:
                pos = seg['position_m']
                gap = seg['gap_m']
                sc  = ZONE_COLORS.get(sz, '')
                print(f'  {pos:5.1f}m  {gap:5.3f}m  {sc}{bar(gap)}{RESET}  {sc}{sz}{RESET}')
                i += 1

        print()
        print(f'  {BOLD}Legend:{RESET} '
              f'{ZONE_COLORS["TOO_CLOSE"]}\u2588 TOO_CLOSE (<0.3m){RESET}  '
              f'{ZONE_COLORS["OPTIMAL"]}\u2588 OPTIMAL (0.3-1.0m){RESET}  '
              f'{ZONE_COLORS["TOO_FAR"]}\u2588 TOO_FAR (>1.0m){RESET}')


def main():
    rclpy.init()
    node = ApproachMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
