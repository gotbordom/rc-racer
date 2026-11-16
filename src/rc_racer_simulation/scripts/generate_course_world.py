#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Anthony Tracy
# SPDX-License-Identifier: MIT

"""
Generate a Gazebo world from a RoboMagellan course file.

The course file (courses/<name>.yaml) is the single source of truth: it lists
the start point, waypoint cones, and obstacles in WGS84 lat/lon, the same way
the real competition hands them out. This script turns it into an SDF world
so the simulation and the course data can never drift apart.

World layout:
  - Based on a template world (plugins, lighting, ground plane)
  - The start point is the world origin; the car spawns there facing east (+X)
  - spherical_coordinates is set to the start point so NavSat reports real
    lat/lon, consistent with the course file
  - Each waypoint is an 18" orange traffic cone (dynamic, so it can be bumped)
  - Each obstacle is a static box or cylinder

Usage:
  generate_course_world.py --course courses/practice_course.yaml \\
      --template worlds/empty_course.world --output practice_course.world
"""

import argparse
import math
import xml.etree.ElementTree as ET

import yaml

# WGS84 ellipsoid
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)

# 18" traffic cone: base plate + three stacked cylinders approximating the taper
# (SDF 1.9 / Fortress has no <cone> geometry)
CONE_HEIGHT = 0.457
CONE_MASS = 1.0
CONE_BASE_SIZE = 0.27
CONE_BASE_HEIGHT = 0.03
CONE_SECTIONS = [  # (radius, height), bottom to top
    (0.12, 0.15),
    (0.085, 0.15),
    (0.05, CONE_HEIGHT - CONE_BASE_HEIGHT - 0.30),
]
CONE_COLOR = '1.0 0.4 0.0 1'
OBSTACLE_COLOR = '0.4 0.3 0.2 1'


def _geodetic_to_ecef(lat_deg, lon_deg, alt):
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * math.sin(lat) ** 2)
    x = (n + alt) * math.cos(lat) * math.cos(lon)
    y = (n + alt) * math.cos(lat) * math.sin(lon)
    z = (n * (1.0 - WGS84_E2) + alt) * math.sin(lat)
    return x, y, z


def geodetic_to_enu(lat_deg, lon_deg, alt, lat0_deg, lon0_deg, alt0):
    """Convert WGS84 lat/lon/alt to local East/North/Up metres about an origin."""
    x, y, z = _geodetic_to_ecef(lat_deg, lon_deg, alt)
    x0, y0, z0 = _geodetic_to_ecef(lat0_deg, lon0_deg, alt0)
    dx, dy, dz = x - x0, y - y0, z - z0

    lat0, lon0 = math.radians(lat0_deg), math.radians(lon0_deg)
    east = -math.sin(lon0) * dx + math.cos(lon0) * dy
    north = (-math.sin(lat0) * math.cos(lon0) * dx
             - math.sin(lat0) * math.sin(lon0) * dy
             + math.cos(lat0) * dz)
    up = (math.cos(lat0) * math.cos(lon0) * dx
          + math.cos(lat0) * math.sin(lon0) * dy
          + math.sin(lat0) * dz)
    return east, north, up


def enu_to_geodetic(east, north, up, lat0_deg, lon0_deg, alt0):
    """Convert local East/North/Up metres about an origin to WGS84 lat/lon/alt."""
    lat0, lon0 = math.radians(lat0_deg), math.radians(lon0_deg)
    x0, y0, z0 = _geodetic_to_ecef(lat0_deg, lon0_deg, alt0)
    x = (x0 - math.sin(lon0) * east
         - math.sin(lat0) * math.cos(lon0) * north
         + math.cos(lat0) * math.cos(lon0) * up)
    y = (y0 + math.cos(lon0) * east
         - math.sin(lat0) * math.sin(lon0) * north
         + math.cos(lat0) * math.sin(lon0) * up)
    z = z0 + math.cos(lat0) * north + math.sin(lat0) * up

    # ECEF -> geodetic (iterative; converges to < 1 mm in a few steps)
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - WGS84_E2))
    for _ in range(10):
        n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * math.sin(lat) ** 2)
        alt = p / math.cos(lat) - n
        lat = math.atan2(z, p * (1.0 - WGS84_E2 * n / (n + alt)))
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * math.sin(lat) ** 2)
    alt = p / math.cos(lat) - n
    return math.degrees(lat), math.degrees(lon), alt


def load_course(path):
    """Load and validate a course file."""
    with open(path) as f:
        course = yaml.safe_load(f)

    for key in ('name', 'start', 'waypoints'):
        if key not in course:
            raise ValueError(f'{path}: missing required key "{key}"')
    if course.get('datum', 'WGS84') != 'WGS84':
        raise ValueError(f'{path}: only the WGS84 datum is supported')

    names = [w['name'] for w in course['waypoints']] + \
        [o['name'] for o in course.get('obstacles', [])]
    duplicates = {n for n in names if names.count(n) > 1}
    if duplicates:
        raise ValueError(f'{path}: duplicate names {sorted(duplicates)}')

    for w in course['waypoints']:
        if w.get('type') not in ('mandatory', 'bonus', 'finish'):
            raise ValueError(f'{path}: waypoint {w["name"]} has invalid type {w.get("type")!r}')
    return course


def local_position(item, start):
    """East/North position of a course item relative to the start (world origin)."""
    east, north, _ = geodetic_to_enu(
        item['latitude'], item['longitude'], start.get('elevation', 0.0),
        start['latitude'], start['longitude'], start.get('elevation', 0.0))
    return east, north


def _sub(parent, tag, text=None, **attrib):
    el = ET.SubElement(parent, tag, attrib)
    if text is not None:
        el.text = str(text)
    return el


def _add_shape(link, kind, name, pose, geometry_tag, geometry, color):
    for element in ('collision', 'visual'):
        el = _sub(link, element, name=f'{name}_{element}')
        _sub(el, 'pose', pose)
        geom = _sub(_sub(el, 'geometry'), geometry_tag)
        for key, value in geometry.items():
            _sub(geom, key, value)
        if element == 'visual':
            material = _sub(el, 'material')
            _sub(material, 'ambient', color)
            _sub(material, 'diffuse', color)


def cone_model(name, x, y):
    """SDF model for an 18" orange traffic cone at (x, y)."""
    model = ET.Element('model', name=name)
    _sub(model, 'pose', f'{x:.3f} {y:.3f} 0 0 0 0')
    link = _sub(model, 'link', name='link')

    inertial = _sub(link, 'inertial')
    _sub(inertial, 'pose', '0 0 0.12 0 0 0')
    _sub(inertial, 'mass', CONE_MASS)
    r, h = 0.1, CONE_HEIGHT
    inertia = _sub(inertial, 'inertia')
    _sub(inertia, 'ixx', f'{CONE_MASS * (3 * r * r + h * h) / 12:.5f}')
    _sub(inertia, 'iyy', f'{CONE_MASS * (3 * r * r + h * h) / 12:.5f}')
    _sub(inertia, 'izz', f'{CONE_MASS * r * r / 2:.5f}')

    _add_shape(link, 'box', 'base', f'0 0 {CONE_BASE_HEIGHT / 2} 0 0 0', 'box',
               {'size': f'{CONE_BASE_SIZE} {CONE_BASE_SIZE} {CONE_BASE_HEIGHT}'}, CONE_COLOR)
    z = CONE_BASE_HEIGHT
    for i, (radius, height) in enumerate(CONE_SECTIONS):
        _add_shape(link, 'cylinder', f'section_{i}', f'0 0 {z + height / 2:.4f} 0 0 0',
                   'cylinder', {'radius': radius, 'length': f'{height:.4f}'}, CONE_COLOR)
        z += height
    return model


def obstacle_model(obstacle, x, y):
    """SDF model for a static box or cylinder obstacle at (x, y)."""
    name, shape = obstacle['name'], obstacle['shape']
    if shape == 'box':
        height = obstacle['size'][2]
        geometry = {'size': ' '.join(str(v) for v in obstacle['size'])}
    elif shape == 'cylinder':
        height = obstacle['height']
        geometry = {'radius': obstacle['radius'], 'length': height}
    else:
        raise ValueError(f'obstacle {name}: unsupported shape {shape!r}')

    model = ET.Element('model', name=name)
    _sub(model, 'static', 'true')
    yaw = math.radians(obstacle.get('yaw_deg', 0.0))
    _sub(model, 'pose', f'{x:.3f} {y:.3f} {height / 2} 0 0 {yaw:.4f}')
    link = _sub(model, 'link', name='link')
    _add_shape(link, shape, 'body', '0 0 0 0 0 0', shape, geometry, OBSTACLE_COLOR)
    return model


def course_to_world(course, template_path):
    """Build the SDF world tree for a course from a template world."""
    tree = ET.parse(template_path)
    world = tree.getroot().find('world')
    world.set('name', course['name'])

    # World origin = start point, so NavSat lat/lon matches the course file
    start = course['start']
    for old in world.findall('spherical_coordinates'):
        world.remove(old)
    sc = _sub(world, 'spherical_coordinates')
    _sub(sc, 'surface_model', 'EARTH_WGS84')
    _sub(sc, 'world_frame_orientation', 'ENU')
    _sub(sc, 'latitude_deg', start['latitude'])
    _sub(sc, 'longitude_deg', start['longitude'])
    _sub(sc, 'elevation', start.get('elevation', 0.0))
    _sub(sc, 'heading_deg', 0)

    for waypoint in course['waypoints']:
        world.append(cone_model(f'cone_{waypoint["name"]}', *local_position(waypoint, start)))
    for obstacle in course.get('obstacles', []):
        world.append(obstacle_model(obstacle, *local_position(obstacle, start)))

    ET.indent(tree, space='  ')
    return tree


def main():
    """Command-line entry point."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--course', required=True, help='course YAML file')
    parser.add_argument('--template', required=True, help='template SDF world')
    parser.add_argument('--output', required=True, help='output SDF world')
    args = parser.parse_args()

    tree = course_to_world(load_course(args.course), args.template)
    with open(args.output, 'wb') as f:
        f.write(b'<?xml version="1.0"?>\n')
        f.write(f'<!-- GENERATED from {args.course} - do not edit -->\n'.encode())
        tree.write(f, encoding='utf-8', xml_declaration=False)


if __name__ == '__main__':
    main()
