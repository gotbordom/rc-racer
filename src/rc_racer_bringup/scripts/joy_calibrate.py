#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Anthony Tracy
# SPDX-License-Identifier: MIT

"""
Find out which buttons and axes a game controller really sends.

Walks through the controls one at a time ("hold LB", "push the left stick
up", ...) and detects which /joy button or axis changed, and which way.

Two modes:

  joy_calibrate.py            Read /joy (game_controller_node, as launched by
                              sim_manual.launch.py) and apply the result to
                              the running teleop_twist_joy_node. A quick fix
                              that lasts until teleop restarts.

  joy_calibrate.py --sdl-map  Read the pad's RAW layout (starts its own
                              joy_node on /joy_raw) and print an SDL mapping
                              line for SDL_GAMECONTROLLERCONFIG. The proper
                              fix for a pad SDL does not know: with a mapping,
                              it gets the standard layout and
                              config/xbox_teleop.yaml works unchanged.

Leave the pad untouched while it starts: the first 2 seconds are the rest
values. Stick axes must swing both ways from rest; an axis that only moves
one way is a trigger (or a stick SDL has mis-mapped as one).
"""

import re
import subprocess
import sys
import time

import rclpy
from sensor_msgs.msg import Joy

REST_SECONDS = 2.0
SWEEP_SECONDS = 2.0
MOVED = 0.4        # change from rest that counts as "moved"
AT_REST = 0.3      # within this of rest counts as "let go"
TELEOP = '/teleop_twist_joy_node'


class Calibrator:
    """Read one /joy topic and detect what each prompted control changes."""

    def __init__(self, topic):
        """Subscribe to the topic and keep the latest message."""
        self.node = rclpy.create_node('joy_calibrate')
        self.latest = None
        self.node.create_subscription(Joy, topic, self._on_joy, 10)
        self.rest_axes = []
        self.rest_buttons = []

    def _on_joy(self, msg):
        self.latest = msg

    def _current(self):
        rclpy.spin_once(self.node, timeout_sec=0.05)
        return self.latest

    def record_rest(self, timeout=15.0):
        """Store the values with nothing touched."""
        print(f'Leave the pad untouched for {REST_SECONDS:.0f} seconds...', flush=True)
        deadline = time.time() + timeout
        while self._current() is None:
            if time.time() > deadline:
                sys.exit('No messages on the joy topic: is the pad connected and joy running?')
        time.sleep(REST_SECONDS)
        self.latest = None
        while self._current() is None:
            pass
        self.rest_axes = list(self.latest.axes)
        self.rest_buttons = list(self.latest.buttons)
        held = [i for i, b in enumerate(self.rest_buttons) if b]
        print(f'Rest: axes {[round(a, 2) for a in self.rest_axes]}  '
              f'buttons already pressed {held}', flush=True)

    def _wait_for_rest(self):
        while True:
            m = self._current()
            if list(m.buttons) == self.rest_buttons and all(
                    abs(a - r) < AT_REST for a, r in zip(m.axes, self.rest_axes)):
                return

    def button(self, label):
        """Return the index of the button pressed for this prompt."""
        print(f'\n>>> {label}: press and HOLD', flush=True)
        while True:
            m = self._current()
            pressed = [i for i, (r, b) in enumerate(zip(self.rest_buttons, m.buttons))
                       if b and not r]
            if pressed:
                print(f'    button {pressed[0]} - now let go', flush=True)
                self._wait_for_rest()
                return pressed[0]

    def sweep(self, label):
        """Return each axis's largest change from rest while this is held."""
        print(f'\n>>> {label}: push fully and HOLD', flush=True)
        while not any(abs(a - r) > MOVED for a, r in zip(self._current().axes, self.rest_axes)):
            pass
        peak = [0.0] * len(self.rest_axes)
        end = time.time() + SWEEP_SECONDS
        while time.time() < end:
            for i, (r, a) in enumerate(zip(self.rest_axes, self._current().axes)):
                if abs(a - r) > abs(peak[i]):
                    peak[i] = a - r
        moved = {i: round(p, 2) for i, p in enumerate(peak) if abs(p) > 0.2}
        print(f'    changes: {moved} - now let go', flush=True)
        self._wait_for_rest()
        return peak

    def stick_axis(self, name, positive, negative):
        """Return (axis, sign) for a stick, from pushes in both directions."""
        pos = self.sweep(f'{name} {positive}')
        neg = self.sweep(f'{name} {negative}')
        # A stick axis swings both ways from rest: opposite signs, both large
        scores = {i: min(abs(p), abs(n)) for i, (p, n) in enumerate(zip(pos, neg)) if p * n < 0}
        if not scores:
            sys.exit(f'No axis swung both ways for {name}: is it mapped as a trigger? '
                     'Try --sdl-map.')
        i = max(scores, key=scores.get)
        sign = 1 if pos[i] > 0 else -1
        print(f'    => {name}: axis {i}, {positive} gives {"+" if sign > 0 else "-"} '
              f'(rest {self.rest_axes[i]:+.2f})', flush=True)
        if abs(self.rest_axes[i]) > 0.2:
            print('    WARNING: this axis does not rest near 0. SDL has probably mis-mapped '
                  'the pad; use --sdl-map', flush=True)
        return i, sign

    def trigger_axis(self, label):
        """Return the axis a trigger moves (one way only)."""
        peak = self.sweep(label)
        i = max(range(len(peak)), key=lambda k: abs(peak[k]))
        print(f'    => {label}: axis {i}', flush=True)
        return i


def teleop_mode():
    """Calibrate /joy and apply the result to the running teleop node."""
    cal = Calibrator('/joy')
    cal.record_rest()
    lb = cal.button('LB (left bumper)')
    rb = cal.button('RB (right bumper)')
    lin, lin_sign = cal.stick_axis('LEFT stick', 'UP', 'DOWN')
    ang, ang_sign = cal.stick_axis('RIGHT stick', 'LEFT', 'RIGHT')
    settings = {
        'enable_button': lb, 'enable_turbo_button': rb,
        'axis_linear.x': lin, 'axis_angular.yaw': ang,
        'scale_linear.x': 1.0 * lin_sign, 'scale_linear_turbo.x': 3.0 * lin_sign,
        'scale_angular.yaw': 1.0 * ang_sign, 'scale_angular_turbo.yaw': 1.5 * ang_sign,
    }
    print(f'\nYour pad: {settings}', flush=True)
    for name, value in settings.items():
        subprocess.run(['ros2', 'param', 'set', TELEOP, name, str(value)],
                       stdout=subprocess.DEVNULL, check=False)
    print(f'Applied to {TELEOP} until it restarts. To keep it, put these in '
          'config/xbox_teleop.yaml, or better, add an SDL mapping (--sdl-map).', flush=True)


def device_guid():
    """Return (guid, name) of joystick 0 from joy_enumerate_devices."""
    out = subprocess.run(['ros2', 'run', 'joy', 'joy_enumerate_devices'],
                         capture_output=True, text=True, timeout=20, check=False).stdout
    match = re.search(r'^\s*0\s*:\s*([0-9a-f]{32})\s*:[^:]*:[^:]*:\s*(.+)$', out, re.MULTILINE)
    if not match:
        sys.exit(f'No joystick found by joy_enumerate_devices:\n{out}')
    return match.group(1), match.group(2).strip()


def sdl_map_mode():
    """Calibrate the raw pad and print an SDL_GAMECONTROLLERCONFIG line."""
    guid, name = device_guid()
    print(f'Pad: {name}  GUID {guid}', flush=True)
    raw = subprocess.Popen(
        ['ros2', 'run', 'joy', 'joy_node', '--ros-args',
         '-r', '__node:=joy_raw', '-r', '/joy:=/joy_raw'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        cal = Calibrator('/joy_raw')
        cal.record_rest()
        buttons = {key: cal.button(label) for key, label in [
            ('a', 'A'), ('b', 'B'), ('x', 'X'), ('y', 'Y'),
            ('leftshoulder', 'LB (left bumper)'), ('rightshoulder', 'RB (right bumper)'),
            ('back', 'View / Back (left small button)'),
            ('start', 'Menu / Start (right small button)'),
            ('leftstick', 'LEFT stick click'), ('rightstick', 'RIGHT stick click')]}
        # joy_node publishes -raw, so a published + is a raw -. SDL wants raw +
        # for stick right / stick down; '~' inverts an axis.
        lx, lx_right = cal.stick_axis('LEFT stick', 'RIGHT', 'LEFT')
        ly, ly_down = cal.stick_axis('LEFT stick', 'DOWN', 'UP')
        rx, rx_right = cal.stick_axis('RIGHT stick', 'RIGHT', 'LEFT')
        ry, ry_down = cal.stick_axis('RIGHT stick', 'DOWN', 'UP')
        lt = cal.trigger_axis('LT (left trigger)')
        rt = cal.trigger_axis('RT (right trigger)')
    finally:
        raw.terminate()

    def axis(i, published_sign):
        return f'a{i}~' if published_sign > 0 else f'a{i}'

    parts = [f'{key}:b{index}' for key, index in buttons.items()] + [
        f'leftx:{axis(lx, lx_right)}', f'lefty:{axis(ly, ly_down)}',
        f'rightx:{axis(rx, rx_right)}', f'righty:{axis(ry, ry_down)}',
        f'lefttrigger:a{lt}', f'righttrigger:a{rt}',
        # The D-pad is a hat on Linux; joy_node cannot show which, so assume
        # the usual hat 0
        'dpup:h0.1', 'dpright:h0.2', 'dpdown:h0.4', 'dpleft:h0.8',
    ]
    line = f'{guid},{name},' + ','.join(parts) + ',platform:Linux,'
    print('\nSDL mapping. Add it to SDL_GAMECONTROLLERCONFIG in the sim-manual service '
          '(docker/docker-compose.yml) and restart:\n', flush=True)
    print(line, flush=True)


def main():
    """Run the calibration in the mode chosen on the command line."""
    rclpy.init()
    try:
        if '--sdl-map' in sys.argv:
            sdl_map_mode()
        else:
            teleop_mode()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
