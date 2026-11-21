#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Anthony Tracy
# SPDX-License-Identifier: MIT

"""
Stream state_estimation's health reports, one line per report.

`ros2 topic echo --once /diagnostics` shows a single report, and a slip WARN
is latched for only one report (about 1 s), so it is easy to miss. This
prints every report as it arrives, coloured by level:

  12:04:31  OK     ok                   slip_events=0  cov_trace=0.0012
  12:04:32  WARN   wheel slip detected  slip_events=1  cov_trace=0.0450

slip_events counts every slip since the last reset or activate, so a rise
between two lines is a slip even if its WARN scrolled past. Ctrl+C stops.
"""

import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.executors import ExternalShutdownException

LEVELS = {
    DiagnosticStatus.OK: ('OK', ''),
    DiagnosticStatus.WARN: ('WARN', '\033[33m'),     # yellow
    DiagnosticStatus.ERROR: ('ERROR', '\033[31m'),   # red
    DiagnosticStatus.STALE: ('STALE', '\033[90m'),   # grey
}
RESET = '\033[0m'


def main():
    rclpy.init()
    node = rclpy.create_node('se_watch')
    colour = sys.stdout.isatty()

    def on_diagnostics(msg):
        for status in msg.status:
            if 'state_estimation' not in status.name:
                continue
            name, code = LEVELS.get(status.level, (str(status.level), ''))
            values = {v.key: v.value for v in status.values}
            line = (f'{time.strftime("%H:%M:%S")}  {name:<5}  {status.message:<40}'
                    f'  slip_events={values.get("slip_events", "-")}')
            if 'covariance_trace' in values:
                line += f'  cov_trace={float(values["covariance_trace"]):.4f}'
            print(f'{code}{line}{RESET}' if colour and code else line, flush=True)

    node.create_subscription(DiagnosticArray, '/diagnostics', on_diagnostics, 10)
    print('Watching /diagnostics for state_estimation (Ctrl+C stops)...', flush=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
