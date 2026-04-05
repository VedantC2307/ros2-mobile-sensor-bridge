#!/usr/bin/env python3
"""
Magnetometer data subscriber and analyzer.
Subscribes to /mobile_sensor/magnetometer and prints live statistics.

Usage:
    python3 magnetometer_analyzer.py
    python3 magnetometer_analyzer.py --window 50 --no-color
"""

import argparse
import math
import sys
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField


# ANSI color codes
class Color:
    RESET  = '\033[0m'
    BOLD   = '\033[1m'
    RED    = '\033[91m'
    GREEN  = '\033[92m'
    YELLOW = '\033[93m'
    CYAN   = '\033[96m'
    WHITE  = '\033[97m'
    GRAY   = '\033[90m'


def no_color(text):
    return text


class MagnetometerAnalyzer(Node):
    def __init__(self, window_size: int, use_color: bool):
        super().__init__('magnetometer_analyzer')

        self.use_color = use_color
        self.c = Color if use_color else type('C', (), {k: '' for k in vars(Color) if not k.startswith('_')})()

        # Sliding window buffers
        self.window = window_size
        self.buf_x    = deque(maxlen=window_size)
        self.buf_y    = deque(maxlen=window_size)
        self.buf_z    = deque(maxlen=window_size)
        self.buf_mag  = deque(maxlen=window_size)
        self.buf_hdg  = deque(maxlen=window_size)

        # Counters
        self.total_msgs    = 0
        self.api_msgs      = 0
        self.orient_msgs   = 0
        self.unknown_msgs  = 0
        self.start_time    = time.time()
        self.last_stamp    = None
        self.lock          = threading.Lock()

        self.subscription = self.create_subscription(
            MagneticField,
            '/mobile_sensor/magnetometer',
            self._callback,
            10
        )

        # Print timer — update display at ~5 Hz
        self.create_timer(0.2, self._print_stats)

        self.get_logger().info(
            f'Listening on /mobile_sensor/magnetometer  '
            f'(window={window_size} samples)'
        )

    # -------------------------------------------------------------------------

    def _callback(self, msg: MagneticField):
        x = msg.magnetic_field.x
        y = msg.magnetic_field.y
        z = msg.magnetic_field.z

        magnitude = math.sqrt(x**2 + y**2 + z**2)
        # Heading in the XY plane (compass direction, degrees from +Y / North)
        heading_deg = math.degrees(math.atan2(x, y)) % 360

        # Detect data source from covariance signature
        cov = list(msg.magnetic_field_covariance)
        if len(cov) >= 1 and cov[0] == -1.0 and any(v == 0.0 for v in cov[1:]):
            source = 'api'
        elif all(v == -1.0 for v in cov):
            source = 'orientation'
        else:
            source = 'unknown'

        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        with self.lock:
            self.total_msgs += 1
            if source == 'api':
                self.api_msgs += 1
            elif source == 'orientation':
                self.orient_msgs += 1
            else:
                self.unknown_msgs += 1

            self.buf_x.append(x)
            self.buf_y.append(y)
            self.buf_z.append(z)
            self.buf_mag.append(magnitude)
            self.buf_hdg.append(heading_deg)
            self.last_stamp = stamp_sec

    # -------------------------------------------------------------------------

    def _print_stats(self):
        with self.lock:
            if self.total_msgs == 0:
                print(f'\r{self.c.YELLOW}Waiting for messages on /mobile_sensor/magnetometer ...{self.c.RESET}',
                      end='', flush=True)
                return

            # Snapshot buffers
            xs   = list(self.buf_x)
            ys   = list(self.buf_y)
            zs   = list(self.buf_z)
            mags = list(self.buf_mag)
            hdgs = list(self.buf_hdg)
            total   = self.total_msgs
            api_n   = self.api_msgs
            ori_n   = self.orient_msgs
            unk_n   = self.unknown_msgs
            elapsed = time.time() - self.start_time

        n = len(mags)
        if n == 0:
            return

        def stats(vals):
            mn = min(vals)
            mx = max(vals)
            avg = sum(vals) / len(vals)
            std = math.sqrt(sum((v - avg)**2 for v in vals) / len(vals))
            return mn, mx, avg, std

        x_mn,  x_mx,  x_avg,  x_std  = stats(xs)
        y_mn,  y_mx,  y_avg,  y_std  = stats(ys)
        z_mn,  z_mx,  z_avg,  z_std  = stats(zs)
        m_mn,  m_mx,  m_avg,  m_std  = stats(mags)
        h_mn,  h_mx,  h_avg,  _      = stats(hdgs)

        rate = total / elapsed if elapsed > 0 else 0.0
        heading_label = self._heading_label(h_avg)

        c = self.c
        sep = f'{c.GRAY}{"─" * 60}{c.RESET}'

        lines = [
            '',
            sep,
            f'{c.BOLD}{c.CYAN}  Magnetometer Analyzer{c.RESET}  '
            f'{c.GRAY}[{n}/{self.window} samples in window]{c.RESET}',
            sep,
            f'  {c.WHITE}Messages:{c.RESET} {total}   '
            f'{c.GREEN}API:{c.RESET} {api_n}   '
            f'{c.YELLOW}Orientation:{c.RESET} {ori_n}   '
            f'{c.GRAY}Unknown:{c.RESET} {unk_n}   '
            f'{c.WHITE}Rate:{c.RESET} {rate:.1f} Hz',
            '',
            f'  {c.BOLD}Component         min         max         avg         std{c.RESET}',
            f'  {c.CYAN}X (T){c.RESET}         {x_mn:+.3e}  {x_mx:+.3e}  {x_avg:+.3e}  {x_std:.3e}',
            f'  {c.CYAN}Y (T){c.RESET}         {y_mn:+.3e}  {y_mx:+.3e}  {y_avg:+.3e}  {y_std:.3e}',
            f'  {c.CYAN}Z (T){c.RESET}         {z_mn:+.3e}  {z_mx:+.3e}  {z_avg:+.3e}  {z_std:.3e}',
            '',
            f'  {c.BOLD}Magnitude (T):{c.RESET}  '
            f'avg {c.GREEN}{m_avg:.3e}{c.RESET}  '
            f'min {m_mn:.3e}  max {m_mx:.3e}  std {m_std:.3e}',
            '',
            f'  {c.BOLD}Heading (XY plane):{c.RESET}  '
            f'{c.GREEN}{h_avg:.1f}°{c.RESET} {c.BOLD}{heading_label}{c.RESET}  '
            f'(range {h_mn:.1f}° – {h_mx:.1f}°)',
            '',
            f'  {c.BOLD}Field strength reference:{c.RESET}  '
            f'{c.GRAY}Earth ≈ 25–65 µT (2.5e-5 – 6.5e-5 T){c.RESET}',
            sep,
        ]

        # Move cursor up by the number of lines we printed last time, then reprint
        num_lines = getattr(self, '_last_num_lines', 0)
        sys.stdout.write(f'\033[{num_lines}A' if num_lines else '')
        output = '\n'.join(lines) + '\n'
        sys.stdout.write(output)
        sys.stdout.flush()
        self._last_num_lines = output.count('\n')

    @staticmethod
    def _heading_label(deg: float) -> str:
        dirs = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW', 'N']
        idx = round(deg / 45) % 8
        return dirs[idx]


# -----------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description='Magnetometer subscriber and analyzer')
    parser.add_argument('--window', type=int, default=30,
                        help='Sliding window size for statistics (default: 30)')
    parser.add_argument('--no-color', action='store_true',
                        help='Disable ANSI color output')
    args = parser.parse_args()

    rclpy.init()
    node = MagnetometerAnalyzer(window_size=args.window, use_color=not args.no_color)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
