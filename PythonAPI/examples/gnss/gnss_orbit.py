# Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
# de Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""GNSS satellite positions from RINEX broadcast ephemerides.

Pure numpy, no CARLA dependency. Reads RINEX 3.0x / 4.0x navigation files and
computes Earth-fixed satellite positions for GPS, Galileo, QZSS and BeiDou
(Keplerian broadcast orbits, IS-GPS-200 / Galileo OS SIS ICD / BDS ICD,
including BeiDou GEO satellites), plus geodetic <-> ECEF <-> local ENU helpers
and azimuth / elevation.

Satellites come from one or more *sources*, all with the same interface
(``satellite_positions(utc, **options) -> {id: ECEF}``):

* NavData       every satellite of a RINEX navigation file at once, moving
                along its broadcast orbit;
* SatelliteSet  satellites placed directly, by ECEF position or by azimuth /
                elevation, for test scenarios and what-if constellations.

GLONASS and SBAS broadcast a state vector instead of Keplerian elements and
are not handled yet; their records are read and skipped.
"""

import datetime as _dt
import json
import math
import re
from collections import namedtuple

import numpy as np

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)

SECONDS_IN_WEEK = 604800.0
GPS_EPOCH = _dt.datetime(1980, 1, 6)
BDT_MINUS_GPST = -14.0     # BeiDou time is 14 s behind GPS time.
BDT_WEEK_OFFSET = 1356     # BDT week 0 (2006-01-01) is GPS week 1356.
DEFAULT_LEAP_SECONDS = 18  # GPST - UTC since 2017-01-01.

# (mu [m^3/s^2], Earth rotation rate [rad/s]) per constellation, as fixed by
# each system's interface control document.
_GM_OMEGA = {
    'G': (3.986005e14, 7.2921151467e-5),
    'J': (3.986005e14, 7.2921151467e-5),
    'E': (3.986004418e14, 7.2921151467e-5),
    'C': (3.986004418e14, 7.292115e-5),
}

CONSTELLATION_NAMES = {
    'G': 'GPS', 'E': 'Galileo', 'J': 'QZSS', 'C': 'BeiDou',
    'R': 'GLONASS', 'S': 'SBAS', 'I': 'NavIC',
}

SUPPORTED_SYSTEMS = tuple(_GM_OMEGA)

# RINEX 4 message types that carry the legacy Keplerian record layout.
_KEPLER_MESSAGES = {
    'G': {'LNAV'}, 'J': {'LNAV'}, 'E': {'INAV', 'FNAV'}, 'C': {'D1', 'D2'},
}

_SAT_ID = re.compile(r'^[A-Z][ 0-9]\d')

_BDS_GEO_TILT = math.radians(-5.0)

# Broadcast orbit lines 1..5 of a Keplerian record, in RINEX order.
KeplerElements = namedtuple('KeplerElements', [
    'iode', 'crs', 'delta_n', 'm0',
    'cuc', 'ecc', 'cus', 'sqrt_a',
    'toe', 'cic', 'omega0', 'cis',
    'i0', 'crc', 'omega', 'omega_dot',
    'idot'])


# ---------------------------------------------------------------------------
# Time
# ---------------------------------------------------------------------------

def parse_utc(text):
    """Parse an ISO-8601 UTC string ('2026-03-31T14:00:00Z') to a naive datetime."""
    value = _dt.datetime.fromisoformat(text.strip().rstrip('Zz'))
    if value.tzinfo is not None:
        value = value.astimezone(_dt.timezone.utc).replace(tzinfo=None)
    return value


def gps_seconds(utc, leap_seconds=DEFAULT_LEAP_SECONDS):
    """Continuous GPS seconds since the GPS epoch for a UTC datetime."""
    return (utc - GPS_EPOCH).total_seconds() + leap_seconds


def _system_seconds(gpst, system):
    """GPS seconds expressed in the constellation's own time scale."""
    return gpst + BDT_MINUS_GPST if system == 'C' else gpst


def _wrap_week(seconds):
    """Wrap a time difference into [-half week, +half week)."""
    half = SECONDS_IN_WEEK / 2.0
    return (seconds + half) % SECONDS_IN_WEEK - half


# ---------------------------------------------------------------------------
# RINEX navigation parsing
# ---------------------------------------------------------------------------

def _rinex_float(field):
    field = field.strip().replace('D', 'E').replace('d', 'E')
    return float(field) if field else float('nan')


class Ephemeris:
    """One Keplerian broadcast ephemeris record."""

    __slots__ = ('sat', 'toc', 'message', 'clock', 'orbit')

    def __init__(self, sat, toc, message, clock, orbit):
        self.sat = sat          # 'G01', 'C59'...
        self.toc = toc          # epoch of clock, in the system time scale
        self.message = message  # RINEX 4 message type, or None for RINEX 3
        self.clock = clock      # af0, af1, af2
        self.orbit = orbit      # broadcast orbit lines 1..7, flattened

    @property
    def system(self):
        """Constellation letter: G, E, J or C."""
        return self.sat[0]

    @property
    def elements(self):
        """Keplerian elements of the record."""
        return KeplerElements(*self.orbit[:17])

    @property
    def toe(self):
        """Time of ephemeris, seconds of the system week."""
        return self.orbit[8]

    @property
    def week(self):
        """Week number of toe as written in the file."""
        return self.orbit[18]

    @property
    def health(self):
        """Raw health word (SV health / SatH1)."""
        return self.orbit[21]

    def is_healthy(self):
        """Whether the satellite is usable on the open-service signal.

        The health word is a bit field whose meaning differs per system. QZSS
        sets bits for signals other than L1 C/A (health=1 is routine), so only
        the L1 C/A bit (MSB of the 6-bit word) counts there. GPS LNAV, Galileo
        and BeiDou SatH1 report a problem with any non-zero value.
        """
        health = self.health
        if math.isnan(health):
            return True
        if self.system == 'J':
            return not int(health) & 0x20
        return health == 0

    def toe_seconds(self):
        """Continuous seconds of toe in the system's own time scale.

        The Galileo week in RINEX is already aligned with the GPS week."""
        week = int(self.week)
        if self.system == 'C':
            week += BDT_WEEK_OFFSET
        return week * SECONDS_IN_WEEK + self.toe

    @classmethod
    def parse(cls, first, message, continuation):
        """Build from the record's lines, or None if it is not usable."""
        sat = first[0] + first[1:3].replace(' ', '0')
        if len(continuation) < 7:
            return None
        try:
            stamp = [int(float(part)) for part in first[4:23].split()]
            toc = _dt.datetime(*stamp)
            clock = [_rinex_float(first[23 + 19 * i:42 + 19 * i]) for i in range(3)]
            orbit = [_rinex_float(line[4 + 19 * i:23 + 19 * i])
                     for line in continuation[:7] for i in range(4)]
        except (TypeError, ValueError):
            return None
        return cls(sat, toc, message, clock, orbit)


def _read_header(lines):
    """(version, leap seconds or None, index of the first body line)."""
    version, leap = 3.0, None
    for index, line in enumerate(lines):
        label = line[60:].strip()
        if label == 'RINEX VERSION / TYPE':
            version = float(line[:9])
        elif label == 'LEAP SECONDS':
            fields = line[:60].split()
            leap = int(fields[0]) if fields else leap
        elif label == 'END OF HEADER':
            return version, leap, index + 1
    raise ValueError('no END OF HEADER: not a RINEX file')


def _split_records(body):
    """Yield (first line, RINEX 4 message type or None, continuation lines)
    for each ephemeris record; RINEX 4 STO/EOP/ION records are dropped."""
    record, message, skip = None, None, False
    for line in body:
        if not line.strip():
            continue
        if line.startswith('>'):
            if record:
                yield record[0], message, record[1:]
            record = None
            fields = line[1:].split()
            skip = len(fields) < 3 or fields[0] != 'EPH'
            message = None if skip else fields[2]
        elif _SAT_ID.match(line) and not skip:
            if record:
                yield record[0], message, record[1:]
            record = [line]
        elif record is not None and line.startswith('    '):
            record.append(line)
    if record:
        yield record[0], message, record[1:]


class NavData:
    """Ephemerides read from one or more RINEX navigation files."""

    def __init__(self):
        self.ephemerides = {}   # sat id -> list of Ephemeris
        self.leap_seconds = None
        self.skipped_systems = set()

    @classmethod
    def from_files(cls, *paths):
        """Read RINEX 3/4 navigation files."""
        nav = cls()
        for path in paths:
            with open(path, 'r', encoding='ascii', errors='replace') as stream:
                nav.read(stream.read().splitlines())
        return nav

    def read(self, lines):
        """Add the records of one file, given as a list of lines."""
        version, leap, body = _read_header(lines)
        if version < 3.0:
            raise ValueError(f'RINEX {version:.2f} navigation files are not '
                             'supported; use RINEX 3 or 4')
        if leap is not None:
            self.leap_seconds = leap
        for first, message, continuation in _split_records(lines[body:]):
            system = first[0]
            if system not in SUPPORTED_SYSTEMS:
                self.skipped_systems.add(system)
                continue
            if message is not None and message not in _KEPLER_MESSAGES[system]:
                continue  # CNAV, CNV1... carry a different record layout.
            eph = Ephemeris.parse(first, message, continuation)
            if eph is not None:
                self.ephemerides.setdefault(eph.sat, []).append(eph)

    def satellites(self, systems=SUPPORTED_SYSTEMS):
        """Sorted ids of the satellites with at least one record."""
        return sorted(sat for sat in self.ephemerides if sat[0] in systems)

    def select(self, sat, gpst, max_age=4 * 3600.0, healthy_only=True):
        """Ephemeris of `sat` with toe closest to `gpst`, or None."""
        best, best_age = None, max_age
        for eph in self.ephemerides.get(sat, ()):
            if healthy_only and not eph.is_healthy():
                continue
            age = abs(_system_seconds(gpst, eph.system) - eph.toe_seconds())
            if age <= best_age:
                best, best_age = eph, age
        return best

    def time_span(self):
        """(first, last) toc over all ephemerides, as naive datetimes."""
        tocs = [eph.toc for records in self.ephemerides.values() for eph in records]
        return (min(tocs), max(tocs)) if tocs else (None, None)

    def satellite_positions(self, utc, **options):
        """{sat: ECEF [m]} of every satellite with a usable ephemeris at `utc`.

        Options: systems (letters, default all supported), healthy_only
        (default True), leap_seconds (default: from the file, else 18)."""
        leap = options.get('leap_seconds')
        if leap is None:
            leap = self.leap_seconds if self.leap_seconds is not None else DEFAULT_LEAP_SECONDS
        gpst = gps_seconds(utc, leap)
        positions = {}
        for sat in self.satellites(tuple(options.get('systems', SUPPORTED_SYSTEMS))):
            eph = self.select(sat, gpst, healthy_only=options.get('healthy_only', True))
            if eph is not None:
                positions[sat] = satellite_position(eph, gpst)
        return positions


# ---------------------------------------------------------------------------
# Satellite position
# ---------------------------------------------------------------------------

def _eccentric_anomaly(mean_anomaly, ecc):
    anomaly = mean_anomaly
    for _ in range(30):
        step = (anomaly - ecc * math.sin(anomaly) - mean_anomaly) / (1.0 - ecc * math.cos(anomaly))
        anomaly -= step
        if abs(step) < 1e-14:
            break
    return anomaly


def _orbital_plane(k, gm, tk):
    """(x, y) in the orbital plane [m] and corrected inclination [rad]."""
    a = k.sqrt_a * k.sqrt_a
    mean_motion = math.sqrt(gm / a ** 3) + k.delta_n
    ek = _eccentric_anomaly(k.m0 + mean_motion * tk, k.ecc)
    vk = math.atan2(math.sqrt(1.0 - k.ecc * k.ecc) * math.sin(ek), math.cos(ek) - k.ecc)
    phi = vk + k.omega
    sin2, cos2 = math.sin(2.0 * phi), math.cos(2.0 * phi)
    uk = phi + k.cus * sin2 + k.cuc * cos2
    rk = a * (1.0 - k.ecc * math.cos(ek)) + k.crs * sin2 + k.crc * cos2
    ik = k.i0 + k.idot * tk + k.cis * sin2 + k.cic * cos2
    return rk * math.cos(uk), rk * math.sin(uk), ik


def _rotate_to_frame(xp, yp, inclination, node):
    """Orbital plane -> frame whose x axis is at longitude `node` from the
    ascending node."""
    sin_o, cos_o = math.sin(node), math.cos(node)
    cos_i = math.cos(inclination)
    return np.array([xp * cos_o - yp * cos_i * sin_o,
                     xp * sin_o + yp * cos_i * cos_o,
                     yp * math.sin(inclination)])


def _bds_geo_to_ecef(position, omega_e, tk):
    """BeiDou GEO: rotate the -5 deg tilted user frame into ECEF (BDS ICD)."""
    xg, yg, zg = position
    cos_t, sin_t = math.cos(_BDS_GEO_TILT), math.sin(_BDS_GEO_TILT)
    cos_r, sin_r = math.cos(omega_e * tk), math.sin(omega_e * tk)
    y1 = yg * cos_t + zg * sin_t
    z1 = -yg * sin_t + zg * cos_t
    return np.array([xg * cos_r + y1 * sin_r, -xg * sin_r + y1 * cos_r, z1])


def is_bds_geo(eph):
    """BeiDou GEO satellites have near-zero inclination (IGSO/MEO ~55 deg)."""
    return eph.system == 'C' and abs(eph.elements.i0) < math.radians(30.0)


def satellite_position(eph, gpst):
    """ECEF position [m] of the satellite at continuous GPS seconds `gpst`."""
    gm, omega_e = _GM_OMEGA[eph.system]
    k = eph.elements
    tk = _wrap_week(_system_seconds(gpst, eph.system) - eph.toe_seconds())
    xp, yp, inclination = _orbital_plane(k, gm, tk)
    if is_bds_geo(eph):
        node = k.omega0 + k.omega_dot * tk - omega_e * k.toe
        return _bds_geo_to_ecef(_rotate_to_frame(xp, yp, inclination, node), omega_e, tk)
    node = k.omega0 + (k.omega_dot - omega_e) * tk - omega_e * k.toe
    return _rotate_to_frame(xp, yp, inclination, node)


# ---------------------------------------------------------------------------
# Geodesy
# ---------------------------------------------------------------------------

def geodetic_to_ecef(lat_deg, lon_deg, alt_m):
    """WGS-84 geodetic (deg, deg, m) -> ECEF [m]."""
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    sin_lat = math.sin(lat)
    normal = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    return np.array([
        (normal + alt_m) * math.cos(lat) * math.cos(lon),
        (normal + alt_m) * math.cos(lat) * math.sin(lon),
        (normal * (1.0 - WGS84_E2) + alt_m) * sin_lat])


def ecef_to_geodetic(xyz):
    """ECEF [m] -> WGS-84 geodetic (deg, deg, m)."""
    x, y, z = xyz
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - WGS84_E2))
    alt = 0.0
    for _ in range(10):
        sin_lat = math.sin(lat)
        normal = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
        alt = p / math.cos(lat) - normal
        lat = math.atan2(z, p * (1.0 - WGS84_E2 * normal / (normal + alt)))
    return math.degrees(lat), math.degrees(math.atan2(y, x)), alt


def ecef_to_enu_matrix(lat_deg, lon_deg):
    """Rotation matrix taking an ECEF difference vector to local ENU."""
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_lon, cos_lon = math.sin(lon), math.cos(lon)
    return np.array([
        [-sin_lon, cos_lon, 0.0],
        [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
        [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat]])


def azimuth_elevation(enu):
    """(azimuth from north clockwise, elevation) in degrees."""
    east, north, up = enu
    azimuth = math.degrees(math.atan2(east, north)) % 360.0
    elevation = math.degrees(math.atan2(up, math.hypot(east, north)))
    return azimuth, elevation


def enu_unit(azimuth_deg, elevation_deg):
    """ENU unit vector towards an azimuth / elevation in degrees."""
    az, el = math.radians(azimuth_deg), math.radians(elevation_deg)
    return np.array([math.cos(el) * math.sin(az), math.cos(el) * math.cos(az), math.sin(el)])


# ---------------------------------------------------------------------------
# Directly placed satellites
# ---------------------------------------------------------------------------

# Default distance of a satellite placed by direction: a GPS-like orbit seen
# from the ground. Only the direction matters for occlusion.
DIRECT_SATELLITE_RANGE = 20200e3


class SatelliteSet:
    """Satellites at positions set directly instead of from an ephemeris.

    Ids are free-form strings. An id starting with a constellation letter
    (G, E, J, C...) is drawn as that constellation. The positions do not move
    with time: set them again to move them."""

    def __init__(self):
        self.positions = {}

    def set_ecef(self, sat, ecef):
        """Place `sat` at an ECEF position [m]."""
        self.positions[sat] = np.asarray(ecef, dtype=float)

    def set_azimuth_elevation(self, sat, azimuth, elevation, reference,
                              range_m=DIRECT_SATELLITE_RANGE):
        """Place `sat` so that it is seen at `azimuth` / `elevation` [deg] from
        `reference` (lat [deg], lon [deg], height [m]), `range_m` away. Seen
        from anywhere in a map around `reference`, the direction differs by
        less than map size / range (5e-5 rad for 1 km)."""
        origin = geodetic_to_ecef(*reference)
        rotation = ecef_to_enu_matrix(reference[0], reference[1])
        self.set_ecef(sat, origin + rotation.T.dot(enu_unit(azimuth, elevation)) * range_m)

    def remove(self, sat):
        """Remove `sat` if present."""
        self.positions.pop(sat, None)

    def satellite_positions(self, utc=None, **_options):
        """{sat: ECEF [m]}; time and the RINEX options do not apply."""
        del utc
        return dict(self.positions)

    @classmethod
    def from_json(cls, path, reference=None):
        """Read ``{"satellites": [{"id": "X01", "ecef": [x, y, z]} |
        {"id": "X02", "azimuth": 90, "elevation": 20}]}``. Azimuth /
        elevation entries need `reference` (lat, lon, height)."""
        with open(path, 'r', encoding='utf-8') as stream:
            entries = json.load(stream)['satellites']
        satellites = cls()
        for entry in entries:
            if 'ecef' in entry:
                satellites.set_ecef(entry['id'], entry['ecef'])
            elif reference is None:
                raise ValueError(f"satellite {entry['id']}: azimuth/elevation needs a reference")
            else:
                satellites.set_azimuth_elevation(
                    entry['id'], entry['azimuth'], entry['elevation'], reference,
                    entry.get('range', DIRECT_SATELLITE_RANGE))
        return satellites


# ---------------------------------------------------------------------------
# Sky
# ---------------------------------------------------------------------------

class SkySatellite(namedtuple('SkySatellite', ['sat', 'azimuth', 'elevation', 'ecef', 'range'])):
    """A satellite above the horizon: id, azimuth and elevation [deg], ECEF
    position [m] and geometric range [m]."""
    __slots__ = ()

    @property
    def system(self):
        """Constellation letter: G, E, J or C."""
        return self.sat[0]



def visible_satellites(sources, receiver_geodetic, utc, elevation_mask=0.0, **options):
    """Satellites above `elevation_mask` [deg] seen from `receiver_geodetic`
    (lat [deg], lon [deg], ellipsoidal height [m]) at `utc`.

    `sources` is one satellite source (NavData, SatelliteSet) or a list of
    them; with the same id in several, the later source wins. `options` are
    passed to each source (see NavData.satellite_positions)."""
    if hasattr(sources, 'satellite_positions'):
        sources = [sources]
    positions = {}
    for source in sources:
        positions.update(source.satellite_positions(utc, **options))
    origin = geodetic_to_ecef(*receiver_geodetic)
    rotation = ecef_to_enu_matrix(receiver_geodetic[0], receiver_geodetic[1])
    sky = []
    for sat in sorted(positions):
        enu = rotation.dot(positions[sat] - origin)
        azimuth, elevation = azimuth_elevation(enu)
        if elevation >= elevation_mask:
            sky.append(SkySatellite(sat, azimuth, elevation, positions[sat],
                                    float(np.linalg.norm(enu))))
    return sky
