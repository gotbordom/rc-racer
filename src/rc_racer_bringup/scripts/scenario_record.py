#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Anthony Tracy
# SPDX-License-Identifier: MIT

"""
Record a manual drive in the sim as a validation scenario (SDP §3.6).

Records /cmd_vel in sim time while the owner drives (pad or keyboard) and,
when Enter is pressed, writes a scenario file for the scenario system test
(test/system/scenarios/README.md):

  - each command's yaw rate is turned back into a steering fraction, the
    inverse of steering_relay: fraction = atan(L * w / v) / max_steering_angle
    (0 when stopped, where the yaw rate carries no steering);
  - Gazebo holds the last command, so each command lasts until the next;
    stretches that don't change (within TOLERANCE) are merged into one step;
  - idle time before the first command is dropped, and the drive ends with
    at least 2 s stopped;
  - the world and tire_mu are read from the running sim (Gazebo's topics and
    the URDF in robot_state_publisher), so they match the drive.

The checks are a template to fill in: a recording says what was driven,
not what should be true of it.

Usage: scenario_record.py --out FILE [--id VAL-xxx] [--title TEXT]
Parameters (must match the car, ICD-GNC-ACT): wheelbase, max_steering_angle.
"""

import argparse
import datetime
import math
import re
import subprocess
import sys
import threading

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import Parameter
from tf2_msgs.msg import TFMessage

TOLERANCE = 0.02            # m/s and steering fraction: merge steps closer than this
MIN_STEP_S = 0.05           # shorter steps are merged into the next
FINAL_STOP_S = 2.0
SPAWN_TOLERANCE_M = 0.3     # warn if the drive did not start at the spawn point


def steering_fraction(speed, yaw_rate, wheelbase, max_steering_angle):
    """Return the steering fraction steering_relay turns into this yaw rate."""
    if abs(speed) < 1e-3:
        return 0.0
    fraction = math.atan(wheelbase * yaw_rate / speed) / max_steering_angle
    return max(-1.0, min(1.0, fraction))


def to_steps(commands, end_time, wheelbase, max_steering_angle):
    """
    Turn [(t, speed, yaw_rate)] (sorted, sim time) into scenario steps.

    Each command holds until the next one, the last until end_time. Steps
    start at the first command; unchanged stretches are merged.
    """
    steps = []
    for i, (t, v, w) in enumerate(commands):
        t_next = commands[i + 1][0] if i + 1 < len(commands) else end_time
        speed = round(v, 2)
        steer = round(steering_fraction(v, w, wheelbase, max_steering_angle), 2)
        duration = t_next - t
        if steps and abs(steps[-1]['speed'] - speed) <= TOLERANCE + 1e-9 and \
                abs(steps[-1]['steer'] - steer) <= TOLERANCE + 1e-9:
            steps[-1]['duration'] += duration
        elif steps and steps[-1]['duration'] < MIN_STEP_S:
            steps[-1].update(speed=speed, steer=steer)
            steps[-1]['duration'] += duration
        else:
            steps.append({'duration': duration, 'speed': speed, 'steer': steer})
    for step in steps:
        step['duration'] = round(step['duration'], 2)
    steps = [s for s in steps if s['duration'] > 0.0]
    if steps and steps[-1]['speed'] == 0.0:
        steps[-1]['steer'] = 0.0
        steps[-1]['duration'] = round(max(steps[-1]['duration'], FINAL_STOP_S), 2)
    else:
        steps.append({'duration': FINAL_STOP_S, 'speed': 0.0, 'steer': 0.0})
    return steps


def scenario_text(steps, meta):
    """Return the scenario file for the steps and recording metadata."""
    lines = [
        f'# Recorded {meta["recorded"]} from a manual drive (scenario_record.py):',
        f'# {meta["samples"]} commands over {meta["duration"]:.1f} s of sim time, '
        f'world {meta["world"]}, tire_mu {meta["tire_mu"]}.',
        f'# Start pose (map): x {meta["start"][0]:.2f} m, y {meta["start"][1]:.2f} m, '
        f'yaw {math.degrees(meta["start"][2]):.1f} deg.',
    ]
    if meta.get('warning'):
        lines.append(f'# WARNING: {meta["warning"]}')
    lines += [
        '# TODO: say what this case is and why, and replace the template checks.',
        f'id: {meta["id"]}',
        f'title: "{meta["title"]}"',
        f'world: {meta["world"]}.world',
        f'tire_mu: {meta["tire_mu"]}',
        'steps:',
    ]
    for s in steps:
        lines.append(f'  - {{duration: {s["duration"]}, speed: {s["speed"]}, '
                     f'steer: {s["steer"]}}}')
    lines += [
        'checks:                     # template: set these for the case',
        '  never_error: true',
        '  ends_ok: true',
        '  settles: true',
    ]
    return '\n'.join(lines) + '\n'


def gazebo_world():
    """Return the running Gazebo world's name, from its topics."""
    topics = subprocess.run(['ign', 'topic', '-l'], capture_output=True, text=True,
                            timeout=10).stdout
    match = re.search(r'^/world/([^/]+)/', topics, re.M)
    if not match:
        raise RuntimeError('no Gazebo world found (is `make sim` running?)')
    return match.group(1)


def tire_mu(node):
    """Return the tyre friction from the URDF robot_state_publisher holds."""
    client = node.create_client(GetParameters, '/robot_state_publisher/get_parameters')
    if not client.wait_for_service(timeout_sec=10.0):
        raise RuntimeError('robot_state_publisher not found (is `make sim` running?)')
    future = client.call_async(GetParameters.Request(names=['robot_description']))
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    urdf = future.result().values[0].string_value
    values = set(re.findall(r'<mu1>\s*([0-9.eE+-]+)\s*</mu1>', urdf))
    if len(values) != 1:
        raise RuntimeError(f'expected one tyre mu1 in the URDF, found {sorted(values)}')
    return float(values.pop())


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument('--out', required=True, help='scenario file to write')
    parser.add_argument('--id', default='VAL-xxx', help='register row (SDP §3.6)')
    parser.add_argument('--title', default='Recorded manual drive')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = rclpy.create_node('scenario_record',
                             parameter_overrides=[Parameter('use_sim_time', value=True)])
    wheelbase = node.declare_parameter('wheelbase', 0.384).value
    max_angle = node.declare_parameter('max_steering_angle', 0.52).value
    world = gazebo_world()
    mu = tire_mu(node)

    lock = threading.Lock()
    commands = []
    pose = [None]
    start_pose = [None]

    def on_cmd(msg):
        t = node.get_clock().now().nanoseconds * 1e-9
        with lock:
            moving = abs(msg.linear.x) > 1e-3
            if commands or moving:
                commands.append((t, msg.linear.x, msg.angular.z))
                if start_pose[0] is None:
                    start_pose[0] = pose[0]

    def on_truth(msg):
        for tf in msg.transforms:
            if tf.child_frame_id == 'rc_racer':
                q, p = tf.transform.rotation, tf.transform.translation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                pose[0] = (p.x, p.y, yaw)

    node.create_subscription(Twist, '/cmd_vel', on_cmd, 50)
    node.create_subscription(TFMessage, '/sim/truth/poses', on_truth, 10)
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    print(f'Recording /cmd_vel in {world} (tire_mu {mu}). Drive now; '
          'press Enter to stop and save.', flush=True)
    try:
        sys.stdin.readline()
    except KeyboardInterrupt:
        print('Interrupted: nothing saved.')
        return 1
    end_time = node.get_clock().now().nanoseconds * 1e-9
    with lock:
        recorded = list(commands)
        start = start_pose[0] or pose[0] or (0.0, 0.0, 0.0)
    rclpy.try_shutdown()

    if not recorded:
        print('No driving commands were recorded: nothing saved.')
        return 1
    steps = to_steps(recorded, end_time, wheelbase, max_angle)
    meta = {
        'recorded': datetime.date.today().isoformat(),
        'samples': len(recorded), 'duration': end_time - recorded[0][0],
        'world': world, 'tire_mu': mu, 'start': start, 'id': args.id, 'title': args.title,
    }
    if math.hypot(start[0], start[1]) > SPAWN_TOLERANCE_M or abs(start[2]) > 0.1:
        meta['warning'] = ('the drive did not start at the spawn point; the replay will. '
                           'Restart `make sim` before recording for a faithful replay.')
        print(f'Warning: {meta["warning"]}')
    with open(args.out, 'w') as f:
        f.write(scenario_text(steps, meta))
    print(f'Saved {len(steps)} steps ({meta["duration"]:.1f} s) to {args.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
