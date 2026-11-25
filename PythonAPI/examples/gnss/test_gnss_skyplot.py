# Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
# de Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Unit tests for the GNSS sky plot prototype. No CARLA server needed:

    python -m unittest discover -s PythonAPI/examples/gnss -p 'test_*.py'

The scene tests replace the `carla` module with a small fake world (a flat
geo projection and one box-shaped building) to check the ray geometry.
"""

import math
import os
import types
import unittest
from collections import namedtuple

import numpy as np

import gnss_orbit
import gnss_skyplot

HERE = os.path.dirname(os.path.abspath(__file__))

NAV = os.path.join(HERE, 'data', 'BRDC00WRD_S_20260901200_01H_MN.rnx')
EPOCH = gnss_orbit.parse_utc('2026-03-31T14:00:00Z')


class TestNavParsing(unittest.TestCase):
    """Reading the bundled RINEX 4.02 multi-GNSS file."""

    @classmethod
    def setUpClass(cls):
        cls.nav = gnss_orbit.NavData.from_files(NAV)

    def test_header(self):
        """Leap seconds come from the header; GLONASS/SBAS are skipped."""
        self.assertEqual(self.nav.leap_seconds, 18)
        self.assertEqual(self.nav.skipped_systems, {'R', 'S'})

    def test_records_per_system(self):
        """Every Keplerian satellite in the file is read."""
        counts = {s: len(self.nav.satellites(s)) for s in 'GEJC'}
        self.assertEqual(counts, {'G': 32, 'E': 21, 'J': 5, 'C': 49})

    def test_first_gps_record(self):
        """Fixed-width fields land in the right slots."""
        # Values copied from the first record of the file (G01).
        eph = self.nav.ephemerides['G01'][0]
        self.assertEqual(eph.message, 'LNAV')
        self.assertAlmostEqual(eph.clock[0], 3.072498366237e-04, places=15)
        self.assertAlmostEqual(eph.orbit[7], 5.153623828888e+03, places=9)  # sqrt(A)
        self.assertEqual(eph.toe, 223200.0)
        self.assertEqual(eph.week, 2412.0)

    def test_qzss_health_is_per_signal(self):
        """Only the L1 C/A bit marks a QZSS satellite unusable."""
        # J02/J03/J07 broadcast health=1 (an L1 C/A-unrelated bit): usable.
        # J08 broadcasts 55, which includes the L1 C/A bit: not usable.
        self.assertTrue(self.nav.ephemerides['J07'][0].is_healthy())
        self.assertFalse(self.nav.ephemerides['J08'][0].is_healthy())


class TestOrbits(unittest.TestCase):
    """Satellite positions and geodesy."""

    @classmethod
    def setUpClass(cls):
        cls.nav = gnss_orbit.NavData.from_files(NAV)
        cls.gpst = gnss_orbit.gps_seconds(EPOCH, 18)

    def position(self, sat):
        """ECEF position of `sat` at the test epoch."""
        return gnss_orbit.satellite_position(self.nav.select(sat, self.gpst), self.gpst)

    def test_gps_position_matches_reference(self):
        """Agreement with an independent implementation."""
        # ECEF [m] at 2026-03-31 14:00:00 UTC computed independently with
        # gnss_lib_py 1.0.3 (sv_models.find_sv_states) from the same records.
        expected = {
            'G01': (15331495.538, -21062302.340, 4940868.328),
            'G04': (7056103.575, -14963058.865, -20707988.415),
            'G15': (-14624934.014, 6648197.923, 20615510.361),
            'G24': (-14657587.583, 18534937.575, 11217467.713),
            'J04': (-25846562.528, 25206091.691, 27400254.220),
            'J07': (-25407787.808, 33658626.170, 2050.206),
        }
        for sat, reference in expected.items():
            error = np.linalg.norm(self.position(sat) - np.array(reference))
            # QZSS differs by ~7 cm: within broadcast-orbit accuracy.
            self.assertLess(error, 0.1 if sat[0] == 'J' else 0.01, (sat, error))

    def test_orbit_radii_between_perigee_and_apogee(self):
        """Every orbit radius is consistent with its own a and e."""
        checked = 0
        for sat in self.nav.satellites('GEJC'):
            eph = self.nav.select(sat, self.gpst, healthy_only=False)
            if eph is None:
                continue
            a, ecc = eph.orbit[7] ** 2, eph.orbit[5]
            radius = np.linalg.norm(gnss_orbit.satellite_position(eph, self.gpst))
            # The harmonic corrections (Crs/Crc) move the radius by < 2 km.
            self.assertGreater(radius, a * (1.0 - ecc) - 2e3, sat)
            self.assertLess(radius, a * (1.0 + ecc) + 2e3, sat)
            checked += 1
        self.assertGreater(checked, 90)

    def test_beidou_geo_sits_at_its_slot(self):
        """BeiDou GEO (special rotation) lands on its nominal longitude."""
        # Nominal BeiDou GEO longitudes.
        for sat, longitude in (('C03', 110.5), ('C04', 160.0), ('C05', 58.75), ('C59', 140.0)):
            lat, lon, alt = gnss_orbit.ecef_to_geodetic(self.position(sat))
            self.assertAlmostEqual(lon, longitude, delta=0.5, msg=sat)
            self.assertLess(abs(lat), 5.0, sat)
            self.assertAlmostEqual(alt / 1e3, 35786, delta=100, msg=sat)

    def test_qzss_seen_from_tokyo(self):
        """Quasi-zenith and GEO QZSS satellites where expected."""
        sky = gnss_orbit.visible_satellites(self.nav, (35.681, 139.767, 40.0), EPOCH, 10.0)
        qzss = {s.sat: s for s in sky if s.system == 'J'}
        self.assertIn('J04', qzss)                 # quasi-zenith
        self.assertGreater(qzss['J04'].elevation, 80.0)
        self.assertIn('J07', qzss)                 # QZS GEO at 127 E, due SSW
        self.assertAlmostEqual(qzss['J07'].azimuth, 201, delta=3)
        self.assertAlmostEqual(qzss['J07'].elevation, 46, delta=3)
        self.assertNotIn('J08', qzss)              # unhealthy

    def test_geodetic_round_trip(self):
        """Geodetic -> ECEF -> geodetic is lossless."""
        for lat, lon, alt in ((35.681, 139.767, 40.0), (-33.9, 151.2, 0.0), (0.0, 0.0, 1e3)):
            back = gnss_orbit.ecef_to_geodetic(gnss_orbit.geodetic_to_ecef(lat, lon, alt))
            self.assertAlmostEqual(back[0], lat, places=9)
            self.assertAlmostEqual(back[1], lon, places=9)
            self.assertAlmostEqual(back[2], alt, places=4)

    def test_azimuth_elevation(self):
        """Azimuth is clockwise from north; enu_unit inverts it."""
        self.assertEqual(gnss_orbit.azimuth_elevation((0.0, 1.0, 0.0)), (0.0, 0.0))
        az, el = gnss_orbit.azimuth_elevation((1.0, 0.0, 1.0))
        self.assertAlmostEqual(az, 90.0)
        self.assertAlmostEqual(el, 45.0)
        enu = gnss_orbit.enu_unit(225.0, 30.0)
        az, el = gnss_orbit.azimuth_elevation(enu)
        self.assertAlmostEqual(az, 225.0)
        self.assertAlmostEqual(el, 30.0)


class TestDirectSatellites(unittest.TestCase):
    """Satellites placed by hand, alone or on top of a RINEX constellation."""

    REFERENCE = (35.681, 139.767, 40.0)

    def test_azimuth_elevation_round_trip(self):
        """A satellite placed by direction is seen in that direction."""
        satellites = gnss_orbit.SatelliteSet()
        for index, (azimuth, elevation) in enumerate(((0.0, 30.0), (123.4, 5.0), (300.0, 70.0))):
            satellites.set_azimuth_elevation(f'X{index}', azimuth, elevation, self.REFERENCE)
        sky = {s.sat: s for s in gnss_orbit.visible_satellites(satellites, self.REFERENCE, EPOCH)}
        self.assertAlmostEqual(sky['X0'].elevation, 30.0, places=6)
        self.assertAlmostEqual(sky['X1'].azimuth, 123.4, places=6)
        self.assertAlmostEqual(sky['X2'].elevation, 70.0, places=6)
        self.assertAlmostEqual(sky['X2'].range, gnss_orbit.DIRECT_SATELLITE_RANGE, delta=1e-3)

    def test_direction_is_stable_across_a_map(self):
        """1 km away from the reference the direction moves < 0.01 deg."""
        satellites = gnss_orbit.SatelliteSet()
        satellites.set_azimuth_elevation('X1', 45.0, 20.0, self.REFERENCE)
        moved = (self.REFERENCE[0] + 0.009, self.REFERENCE[1], self.REFERENCE[2])
        seen = gnss_orbit.visible_satellites(satellites, moved, EPOCH)[0]
        self.assertAlmostEqual(seen.azimuth, 45.0, delta=0.01)
        self.assertAlmostEqual(seen.elevation, 20.0, delta=0.01)

    def test_json_file(self):
        """The example file loads; azimuth/elevation needs a reference."""
        path = os.path.join(HERE, 'satellites_example.json')
        with self.assertRaises(ValueError):
            gnss_orbit.SatelliteSet.from_json(path)
        satellites = gnss_orbit.SatelliteSet.from_json(path, self.REFERENCE)
        self.assertEqual(sorted(satellites.positions), ['G99', 'X01', 'X02', 'X03'])

    def test_direct_overrides_rinex(self):
        """A directly placed satellite replaces the RINEX one with its id."""
        nav = gnss_orbit.NavData.from_files(NAV)
        satellites = gnss_orbit.SatelliteSet()
        satellites.set_azimuth_elevation('G24', 90.0, 45.0, self.REFERENCE)
        both = {s.sat: s for s in gnss_orbit.visible_satellites([nav, satellites], self.REFERENCE, EPOCH)}
        rinex = {s.sat: s for s in gnss_orbit.visible_satellites(nav, self.REFERENCE, EPOCH)}
        self.assertAlmostEqual(both['G24'].azimuth, 90.0, places=6)
        self.assertNotAlmostEqual(rinex['G24'].azimuth, 90.0, places=0)
        self.assertEqual(set(both), set(rinex))


class TestClassification(unittest.TestCase):
    """Turning the hits of one ray into a satellite status."""

    def test_clear_sky(self):
        """No hits, or only the sky dome: line of sight."""
        self.assertEqual(gnss_skyplot.classify_hits([]), (gnss_skyplot.LOS, None))
        self.assertEqual(gnss_skyplot.classify_hits([('Sky', 900.0)]), (gnss_skyplot.LOS, None))

    def test_building_blocks(self):
        """A building behind a tree still blocks."""
        status, first = gnss_skyplot.classify_hits([('Vegetation', 5.0), ('Buildings', 20.0)])
        self.assertEqual(status, gnss_skyplot.BLOCKED)
        self.assertEqual(first, ('Buildings', 20.0))

    def test_vegetation_attenuates(self):
        """Only thin objects and foliage: attenuated."""
        status, first = gnss_skyplot.classify_hits([('Vegetation', 8.0), ('Poles', 3.0)])
        self.assertEqual(status, gnss_skyplot.ATTENUATED)
        self.assertEqual(first, ('Poles', 3.0))

    def test_own_vehicle_ignored_but_others_block(self):
        """A car at the antenna is the ego; a truck further away blocks."""
        self.assertEqual(gnss_skyplot.classify_hits([('Car', 1.0)])[0], gnss_skyplot.LOS)
        self.assertEqual(gnss_skyplot.classify_hits([('Truck', 12.0)])[0], gnss_skyplot.BLOCKED)


# ---------------------------------------------------------------------------
# Fake CARLA world
# ---------------------------------------------------------------------------

class _Location:
    """Enough of carla.Location for the scene code."""
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = float(x), float(y), float(z)

    def __add__(self, other):
        """Vector sum."""
        return _Location(self.x + other.x, self.y + other.y, self.z + other.z)

    def distance(self, other):
        """Euclidean distance [m]."""
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2)


_Geo = namedtuple('_Geo', ['latitude', 'longitude', 'altitude'])
_Hit = namedtuple('_Hit', ['location', 'label'])


class _FakeMap:  # pylint: disable=too-few-public-methods
    """Locally flat projection like CARLA's: +x east, +y south, +z up."""
    LAT0, LON0 = 35.681, 139.767
    METERS_PER_DEG = 111319.49

    def transform_to_geolocation(self, location):
        """Flat-earth projection around (LAT0, LON0)."""
        lat = self.LAT0 - location.y / self.METERS_PER_DEG
        lon = self.LON0 + location.x / (self.METERS_PER_DEG * math.cos(math.radians(self.LAT0)))
        return _Geo(lat, lon, location.z)


class _FakeWorld:
    """One 30 m tall building occupying x in [5, 25], y in [-100, 100]: a
    wall running north-south, 5 m east of the origin."""
    BOX = ((5.0, 25.0), (-100.0, 100.0), (0.0, 30.0))

    def __init__(self):
        self.rays = 0

    def get_map(self):
        """The fake map."""
        return _FakeMap()

    def cast_ray(self, start, end):
        """Slab test of the segment against the box: one hit at entry."""
        self.rays += 1
        origin = np.array([start.x, start.y, start.z])
        delta = np.array([end.x, end.y, end.z]) - origin
        t_near, t_far = 0.0, 1.0
        for axis, (low, high) in enumerate(self.BOX):
            if abs(delta[axis]) < 1e-12:
                if not low <= origin[axis] <= high:
                    return []
                continue
            t0, t1 = sorted(((low - origin[axis]) / delta[axis],
                             (high - origin[axis]) / delta[axis]))
            t_near, t_far = max(t_near, t0), min(t_far, t1)
            if t_near > t_far:
                return []
        point = origin + t_near * delta
        return [_Hit(_Location(*point), 'Buildings')]


class TestSceneOcclusion(unittest.TestCase):
    """Ray geometry against the fake world."""

    def setUp(self):
        fake = types.ModuleType('carla')
        fake.Location = _Location
        self._saved = gnss_skyplot.carla
        gnss_skyplot.carla = fake

    def tearDown(self):
        gnss_skyplot.carla = self._saved

    def test_basis_matches_carla_axes(self):
        """The numerically measured basis is +x east, +y south."""
        scene = gnss_skyplot.SceneOcclusion(_FakeWorld())
        basis = scene.basis(_Location(0.0, 0.0, 1.5))
        # +x is east, +y is south, up to the flat-earth approximation.
        np.testing.assert_allclose(basis, [[1.0, 0.0], [0.0, -1.0]], atol=5e-3)
        north = gnss_skyplot.carla_direction(basis, gnss_orbit.enu_unit(0.0, 0.0))
        np.testing.assert_allclose(north, [0.0, -1.0, 0.0], atol=5e-3)

    def test_wall_to_the_east_blocks_low_eastern_satellites(self):
        """Each satellite is blocked exactly when its ray meets the wall."""
        world = _FakeWorld()
        scene = gnss_skyplot.SceneOcclusion(world)
        antenna = _Location(0.0, 0.0, 1.5)
        nav = gnss_orbit.NavData.from_files(NAV)
        sky = gnss_orbit.visible_satellites(nav, scene.geodetic(antenna), EPOCH, 5.0)
        results = scene.check(antenna, sky)
        self.assertEqual(world.rays, len(sky))
        # The wall top is 28.5 m above the antenna and 5 m away: anything in
        # the east half (az 0..180) below atan(28.5/(5/sin(az))) is blocked.
        for sat in sky:
            status = results[sat.sat][0]
            sin_az = math.sin(math.radians(sat.azimuth))
            if sin_az <= 0.05:
                self.assertEqual(status, gnss_skyplot.LOS, sat.sat)
                continue
            # Distance along the horizontal to the wall plane x=5, then the
            # height the ray reaches there.
            horizontal = 5.0 / sin_az
            if abs(horizontal * math.cos(math.radians(sat.azimuth))) > 95.0:
                continue  # passes the end of the 200 m wall: skip edge cases
            height = horizontal * math.tan(math.radians(sat.elevation))
            expected = gnss_skyplot.BLOCKED if height < 28.4 else gnss_skyplot.LOS
            if abs(height - 28.5) > 0.5:
                self.assertEqual(status, expected, (sat.sat, sat.azimuth, sat.elevation))
        statuses = [r[0] for r in results.values()]
        self.assertIn(gnss_skyplot.BLOCKED, statuses)
        self.assertIn(gnss_skyplot.LOS, statuses)

    def test_hand_placed_satellites_against_the_wall(self):
        """Low east: hits the wall 5 m away; west or overhead: clear."""
        scene = gnss_skyplot.SceneOcclusion(_FakeWorld())
        antenna = _Location(0.0, 0.0, 1.5)
        reference = scene.geodetic(_Location())
        satellites = gnss_orbit.SatelliteSet()
        satellites.set_azimuth_elevation('EAST_LOW', 90.0, 10.0, reference)
        satellites.set_azimuth_elevation('EAST_HIGH', 90.0, 85.0, reference)
        satellites.set_azimuth_elevation('WEST_LOW', 270.0, 10.0, reference)
        sky = gnss_orbit.visible_satellites(satellites, scene.geodetic(antenna), EPOCH)
        results = scene.check(antenna, sky)
        status, first = results['EAST_LOW']
        self.assertEqual(status, gnss_skyplot.BLOCKED)
        self.assertEqual(first[0], 'Buildings')
        self.assertAlmostEqual(first[1], 5.0 / math.cos(math.radians(10.0)), delta=0.05)
        self.assertEqual(results['EAST_HIGH'][0], gnss_skyplot.LOS)
        self.assertEqual(results['WEST_LOW'][0], gnss_skyplot.LOS)


if __name__ == '__main__':
    unittest.main()
