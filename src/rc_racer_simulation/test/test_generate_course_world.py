# SPDX-FileCopyrightText: 2025 Anthony Tracy
# SPDX-License-Identifier: MIT

"""Unit tests for scripts/generate_course_world.py."""

import math
import os

import pytest
import yaml

from generate_course_world import (
    CONE_HEIGHT,
    course_to_world,
    enu_to_geodetic,
    geodetic_to_enu,
    load_course,
    local_position,
)

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TEMPLATE = os.path.join(PKG_DIR, 'worlds', 'empty_course.world')
COURSES_DIR = os.path.join(PKG_DIR, 'courses')
COURSE_FILES = sorted(f for f in os.listdir(COURSES_DIR) if f.endswith('.yaml'))

LAT0, LON0, ALT0 = 37.6602, -121.8890, 100.0
# One degree of latitude is ~111 km near 37.66 N
METRES_PER_DEG_LAT = 111_000.0


def _write_course(tmp_path, **overrides):
    course = {
        'name': 'test_course',
        'start': {'latitude': LAT0, 'longitude': LON0, 'elevation': ALT0},
        'waypoints': [
            {'name': 'M1', 'type': 'mandatory', 'order': 1,
             'latitude': LAT0 + 0.0001, 'longitude': LON0},
        ],
        'obstacles': [],
    }
    course.update(overrides)
    path = tmp_path / 'course.yaml'
    path.write_text(yaml.safe_dump(course))
    return path


class TestGeodetic:
    """WGS84 <-> local ENU conversion."""

    def test_origin_maps_to_zero(self):
        east, north, up = geodetic_to_enu(LAT0, LON0, ALT0, LAT0, LON0, ALT0)
        assert east == pytest.approx(0.0, abs=1e-6)
        assert north == pytest.approx(0.0, abs=1e-6)
        assert up == pytest.approx(0.0, abs=1e-6)

    def test_north_offset(self):
        east, north, _ = geodetic_to_enu(LAT0 + 0.001, LON0, ALT0, LAT0, LON0, ALT0)
        assert east == pytest.approx(0.0, abs=0.01)
        assert north == pytest.approx(0.001 * METRES_PER_DEG_LAT, rel=0.01)

    def test_east_offset(self):
        east, north, _ = geodetic_to_enu(LAT0, LON0 + 0.001, ALT0, LAT0, LON0, ALT0)
        expected = 0.001 * METRES_PER_DEG_LAT * math.cos(math.radians(LAT0))
        assert east == pytest.approx(expected, rel=0.01)
        assert north == pytest.approx(0.0, abs=0.01)

    @pytest.mark.parametrize('east,north', [(15, 5), (-300, 200), (0, 305)])
    def test_round_trip(self, east, north):
        lat, lon, alt = enu_to_geodetic(east, north, 0.0, LAT0, LON0, ALT0)
        e, n, u = geodetic_to_enu(lat, lon, alt, LAT0, LON0, ALT0)
        assert (e, n, u) == pytest.approx((east, north, 0.0), abs=1e-3)


class TestLoadCourse:
    """Course file validation."""

    def test_valid_course(self, tmp_path):
        course = load_course(_write_course(tmp_path))
        assert course['name'] == 'test_course'

    def test_missing_start(self, tmp_path):
        path = _write_course(tmp_path)
        data = yaml.safe_load(path.read_text())
        del data['start']
        path.write_text(yaml.safe_dump(data))
        with pytest.raises(ValueError, match='start'):
            load_course(path)

    def test_duplicate_names(self, tmp_path):
        obstacles = [{'name': 'M1', 'shape': 'box', 'size': [1, 1, 1],
                      'latitude': LAT0, 'longitude': LON0}]
        with pytest.raises(ValueError, match='duplicate'):
            load_course(_write_course(tmp_path, obstacles=obstacles))

    def test_invalid_waypoint_type(self, tmp_path):
        waypoints = [{'name': 'X', 'type': 'optional', 'latitude': LAT0, 'longitude': LON0}]
        with pytest.raises(ValueError, match='invalid type'):
            load_course(_write_course(tmp_path, waypoints=waypoints))

    def test_non_wgs84_datum(self, tmp_path):
        with pytest.raises(ValueError, match='WGS84'):
            load_course(_write_course(tmp_path, datum='NAD27'))


def _world(tmp_path, **overrides):
    course = load_course(_write_course(tmp_path, **overrides))
    return course_to_world(course, TEMPLATE).getroot().find('world')


class TestCourseToWorld:
    """Generated SDF world contents."""

    def test_world_named_after_course(self, tmp_path):
        world = _world(tmp_path)
        assert world.get('name') == 'test_course'

    def test_origin_is_start_point(self, tmp_path):
        world = _world(tmp_path)
        coords = world.findall('spherical_coordinates')
        assert len(coords) == 1, 'template spherical_coordinates must be replaced, not duplicated'
        assert float(coords[0].find('latitude_deg').text) == LAT0
        assert float(coords[0].find('longitude_deg').text) == LON0

    def test_template_plugins_kept(self, tmp_path):
        world = _world(tmp_path)
        plugins = {p.get('filename') for p in world.findall('plugin')}
        assert {'ignition-gazebo-imu-system', 'ignition-gazebo-navsat-system'} <= plugins

    def test_cone_placed_at_waypoint(self, tmp_path):
        course = load_course(_write_course(tmp_path))
        world = course_to_world(course, TEMPLATE).getroot().find('world')
        cone = world.find("model[@name='cone_M1']")
        assert cone is not None
        x, y = (float(v) for v in cone.find('pose').text.split()[:2])
        expected = local_position(course['waypoints'][0], course['start'])
        assert (x, y) == pytest.approx(expected, abs=0.01)
        assert cone.find('static') is None, 'cones must be dynamic so bumps move them'

    def test_cone_height(self, tmp_path):
        world = _world(tmp_path)
        cone = world.find("model[@name='cone_M1']")
        tops = []
        for visual in cone.iter('visual'):
            z = float(visual.find('pose').text.split()[2])
            geom = visual.find('geometry')[0]
            half = float((geom.find('length') if geom.tag == 'cylinder'
                          else geom.find('size')).text.split()[-1]) / 2
            tops.append(z + half)
        assert max(tops) == pytest.approx(CONE_HEIGHT, abs=0.005)

    def test_obstacles_are_static(self, tmp_path):
        obstacles = [
            {'name': 'rock', 'shape': 'box', 'size': [1, 1, 0.5],
             'latitude': LAT0, 'longitude': LON0 + 0.0001},
            {'name': 'tree', 'shape': 'cylinder', 'radius': 0.3, 'height': 3.0,
             'latitude': LAT0 + 0.0002, 'longitude': LON0},
        ]
        world = _world(tmp_path, obstacles=obstacles)
        for name in ('rock', 'tree'):
            model = world.find(f"model[@name='{name}']")
            assert model is not None
            assert model.find('static').text == 'true'

    def test_unsupported_obstacle_shape(self, tmp_path):
        obstacles = [{'name': 'blob', 'shape': 'sphere', 'radius': 1,
                      'latitude': LAT0, 'longitude': LON0}]
        with pytest.raises(ValueError, match='unsupported shape'):
            _world(tmp_path, obstacles=obstacles)


@pytest.mark.parametrize('course_file', COURSE_FILES)
def test_committed_courses_generate(course_file):
    """Every committed course file loads and produces one cone per waypoint."""
    course = load_course(os.path.join(COURSES_DIR, course_file))
    world = course_to_world(course, TEMPLATE).getroot().find('world')
    cones = [m for m in world.findall('model') if m.get('name').startswith('cone_')]
    assert len(cones) == len(course['waypoints'])
