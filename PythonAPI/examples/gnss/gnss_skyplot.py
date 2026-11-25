#!/usr/bin/env python

# Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
# de Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""GNSS sky plot with satellite occlusion from the CARLA scene (prototype).

Computes where the GPS / Galileo / QZSS / BeiDou satellites are in the sky
above a point of a CARLA map, from a RINEX broadcast ephemeris and the map's
OpenDRIVE geo-reference, then casts one ray per satellite through the scene
with World.cast_ray and classifies each satellite as:

  LOS         nothing between the antenna and the satellite
  ATTENUATED  only vegetation or thin objects (poles, signs) in the way
  BLOCKED     a building, wall, terrain, vehicle... in the way (NLOS)

Results are written as a polar sky plot (PNG), optionally as JSON, and the
rays can be drawn in the simulator with world.debug.

Examples:

  # Sky plot at the hero vehicle's antenna, rays drawn in the world
  python gnss_skyplot.py --role-name hero --draw-rays --output sky.png

  # A fixed map location, at a chosen UTC time
  python gnss_skyplot.py --location 10,-20,2 --time 2026-03-31T13:30:00Z

  # Follow the vehicle and refresh the plot every simulated second
  python gnss_skyplot.py --role-name hero --watch --period 1.0

  # Orbit part only, no CARLA server: sky above a latitude/longitude
  python gnss_skyplot.py --offline 35.681,139.767,40 --output tokyo.png

  # Satellites placed by hand (see satellites_example.json), with or
  # without the RINEX constellation
  python gnss_skyplot.py --role-name hero --satellites my_sats.json --no-nav
"""

import argparse
import datetime
import json
import math
import os

import numpy as np

import gnss_orbit

try:
    import carla
except ImportError:  # --offline and the unit tests run without it.
    carla = None

try:
    import matplotlib
except ImportError:
    matplotlib = None

DEFAULT_NAV = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           'data', 'BRDC00WRD_S_20260901200_01H_MN.rnx')

LOS, ATTENUATED, BLOCKED, UNCHECKED = 'LOS', 'ATTENUATED', 'BLOCKED', 'UNCHECKED'

# Labels that never count as an obstruction (the sky dome, decals).
IGNORED_LABELS = frozenset(['Sky', 'RoadLines'])
# Labels that attenuate the signal rather than block it.
ATTENUATING_LABELS = frozenset(['Vegetation', 'Poles', 'TrafficSigns', 'TrafficLight'])
# Vehicle labels; hits close to the antenna are assumed to be the ego vehicle.
VEHICLE_LABELS = frozenset(['Car', 'Truck', 'Bus', 'Motorcycle', 'Bicycle', 'Rider', 'Train'])

STATUS_COLORS = {
    LOS: '#1b9e4b',
    ATTENUATED: '#e69f00',
    BLOCKED: '#d62728',
    UNCHECKED: '#4c72b0',
}
SYSTEM_MARKERS = {'G': 'o', 'E': 's', 'J': 'D', 'C': '^'}


# ---------------------------------------------------------------------------
# Pure helpers (no CARLA objects): unit-tested in test_gnss_skyplot.py
# ---------------------------------------------------------------------------

def local_enu_basis(geo_origin, geo_dx, geo_dy, step=1.0):
    """2x2 matrix mapping a horizontal CARLA displacement (x, y) [m] to (east,
    north) [m], estimated from the geolocations of the origin and of the
    points `step` metres along +x and +y. Handles any map projection, its grid
    convergence and CARLA's left-handed axes."""
    rotation = gnss_orbit.ecef_to_enu_matrix(geo_origin[0], geo_origin[1])
    origin = gnss_orbit.geodetic_to_ecef(*geo_origin)
    ex = rotation.dot(gnss_orbit.geodetic_to_ecef(*geo_dx) - origin) / step
    ey = rotation.dot(gnss_orbit.geodetic_to_ecef(*geo_dy) - origin) / step
    return np.array([[ex[0], ey[0]], [ex[1], ey[1]]])


def carla_direction(basis, enu):
    """Unit vector in CARLA world axes pointing along the ENU unit vector
    `enu`, given the basis from local_enu_basis (z is up in both frames)."""
    horizontal = np.linalg.solve(basis, np.asarray(enu[:2], dtype=float))
    vector = np.array([horizontal[0], horizontal[1], enu[2]])
    return vector / np.linalg.norm(vector)


def classify_hits(hits, self_clearance=3.0):
    """Classify one ray from its hits, a list of (label_name, distance [m]).

    Returns (status, first obstructing (label, distance) or None)."""
    status, first = LOS, None
    for label, distance in sorted(hits, key=lambda hit: hit[1]):
        if label in IGNORED_LABELS:
            continue
        if label in VEHICLE_LABELS and distance < self_clearance:
            continue  # the vehicle carrying the antenna
        if label in ATTENUATING_LABELS:
            if status == LOS:
                status, first = ATTENUATED, (label, distance)
            continue
        return BLOCKED, (label, distance)
    return status, first


def summarize(sky, results):
    """Rows of per-satellite results, highest elevation first."""
    rows = []
    for sat in sorted(sky, key=lambda s: (-s.elevation, s.sat)):
        status, first = results.get(sat.sat, (UNCHECKED, None))
        rows.append({
            'sat': sat.sat,
            'system': gnss_orbit.CONSTELLATION_NAMES.get(sat.system, 'Custom'),
            'azimuth_deg': round(sat.azimuth, 2),
            'elevation_deg': round(sat.elevation, 2),
            'status': status,
            'obstruction': first[0] if first else None,
            'obstruction_distance_m': round(first[1], 2) if first else None,
        })
    return rows


# ---------------------------------------------------------------------------
# CARLA side
# ---------------------------------------------------------------------------

def _offset(location, direction, distance):
    return location + carla.Location(x=float(direction[0] * distance),
                                     y=float(direction[1] * distance),
                                     z=float(direction[2] * distance))


class SceneOcclusion:
    """Casts satellite rays through a CARLA world."""

    def __init__(self, world, ray_length=1000.0, self_clearance=3.0):
        self.world = world
        self.map = world.get_map()
        self.ray_length = ray_length
        self.self_clearance = self_clearance

    def geodetic(self, location):
        """(latitude, longitude, altitude) of a CARLA location."""
        geo = self.map.transform_to_geolocation(location)
        return (geo.latitude, geo.longitude, geo.altitude)

    def basis(self, location):
        """ENU basis at `location`, see local_enu_basis."""
        return local_enu_basis(self.geodetic(location),
                               self.geodetic(location + carla.Location(x=1.0)),
                               self.geodetic(location + carla.Location(y=1.0)))

    def check(self, location, sky, draw_life=None):
        """{sat: (status, first obstruction)} for a list of SkySatellite.
        With `draw_life` set, the rays are drawn for that many seconds."""
        basis = self.basis(location)
        results = {}
        for sat in sky:
            direction = carla_direction(basis, gnss_orbit.enu_unit(sat.azimuth, sat.elevation))
            end = _offset(location, direction, self.ray_length)
            hits = [(str(hit.label), hit.location.distance(location))
                    for hit in self.world.cast_ray(location, end)]
            results[sat.sat] = classify_hits(hits, self.self_clearance)
            if draw_life is not None:
                self.draw(location, direction, sat.sat, results[sat.sat], draw_life)
        return results

    def draw(self, start, direction, name, result, life):
        """Draw one satellite ray, cut at the obstruction when blocked."""
        status, first = result
        rgb = STATUS_COLORS[status].lstrip('#')
        color = carla.Color(int(rgb[0:2], 16), int(rgb[2:4], 16), int(rgb[4:6], 16))
        shown = min(self.ray_length, 200.0)
        if first is not None and status == BLOCKED:
            shown = min(shown, first[1])
        tip = _offset(start, direction, shown)
        self.world.debug.draw_line(start, tip, thickness=0.05, color=color, life_time=life)
        self.world.debug.draw_string(tip, name, draw_shadow=False, color=color, life_time=life)


def antenna_location(actor, height=0.05):
    """A point `height` metres above the top centre of the actor's bounding box."""
    box = actor.bounding_box
    local = carla.Location(x=box.location.x, y=box.location.y,
                           z=box.location.z + box.extent.z + height)
    return actor.get_transform().transform(local)


def find_actor(world, actor_id=None, role_name=None):
    """The actor with `actor_id`, else the vehicle with `role_name`."""
    actors = world.get_actors()
    if actor_id is not None:
        actor = actors.find(actor_id)
        if actor is None:
            raise RuntimeError(f'no actor with id {actor_id}')
        return actor
    for actor in actors.filter('vehicle.*'):
        if actor.attributes.get('role_name') == role_name:
            return actor
    raise RuntimeError(f"no vehicle with role_name '{role_name}'")


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------

def print_table(header, rows):
    """Print the per-satellite results."""
    print(header)
    print(f"  {'sat':<4} {'system':<8} {'az':>7} {'el':>7}  {'status':<10} first hit")
    for row in rows:
        hit = ''
        if row['obstruction']:
            hit = f"{row['obstruction']} @ {row['obstruction_distance_m']:.1f} m"
        print(f"  {row['sat']:<4} {row['system']:<8} {row['azimuth_deg']:7.1f} "
              f"{row['elevation_deg']:7.1f}  {row['status']:<10} {hit}")
    counts = {}
    for row in rows:
        counts[row['status']] = counts.get(row['status'], 0) + 1
    print('  ' + ', '.join(f'{status} {count}' for status, count in sorted(counts.items())))


class SkyPlot:
    """Polar sky plot: north up, azimuth clockwise, zenith at the centre."""

    def __init__(self, show=False):
        if not show:
            matplotlib.use('Agg')
        # pyplot must be imported after the backend is chosen.
        import matplotlib.pyplot as plt  # pylint: disable=import-outside-toplevel
        from matplotlib.lines import Line2D  # pylint: disable=import-outside-toplevel
        self.plt = plt
        self.show = show
        self.figure = plt.figure(figsize=(7, 8.2))
        self.axes = self.figure.add_subplot(111, projection='polar')
        self.legend = (
            [Line2D([], [], marker='o', ls='', color=STATUS_COLORS[s], label=s)
             for s in (LOS, ATTENUATED, BLOCKED, UNCHECKED)] +
            [Line2D([], [], marker=m, ls='', color='0.4', mfc='none',
                    label=gnss_orbit.CONSTELLATION_NAMES[s])
             for s, m in SYSTEM_MARKERS.items()])
        if show:
            plt.ion()

    def _draw_grid(self, elevation_mask):
        axes = self.axes
        axes.clear()
        axes.set_theta_zero_location('N')
        axes.set_theta_direction(-1)
        axes.set_rlim(0, 90)
        axes.set_rticks([0, 30, 60, 90])
        axes.set_yticklabels(['90', '60', '30', '0'], fontsize=8, color='0.4')
        axes.set_xticks(np.radians([0, 90, 180, 270]))
        axes.set_xticklabels(['N', 'E', 'S', 'W'])
        if elevation_mask > 0:
            theta = np.linspace(0, 2 * np.pi, 361)
            axes.plot(theta, np.full_like(theta, 90 - elevation_mask), ls='--', lw=0.8, color='0.5')

    def update(self, rows, title, elevation_mask, path=None):
        """Redraw with `rows` from summarize(); save to `path` if given."""
        self._draw_grid(elevation_mask)
        for row in rows:
            theta = math.radians(row['azimuth_deg'])
            radius = 90.0 - row['elevation_deg']
            self.axes.plot(theta, radius, marker=SYSTEM_MARKERS.get(row['sat'][0], '*'), ms=11, ls='',
                           color=STATUS_COLORS[row['status']], mec='white', mew=0.8)
            self.axes.annotate(row['sat'], (theta, radius), textcoords='offset points',
                               xytext=(7, 5), fontsize=7.5, color='0.15')
        statuses = [row['status'] for row in rows]
        counts = ', '.join(f'{s} {statuses.count(s)}'
                           for s in (LOS, ATTENUATED, BLOCKED, UNCHECKED) if s in statuses)
        self.axes.set_title(title + '\n' + counts, fontsize=10, pad=22)
        self.axes.legend(handles=self.legend, loc='upper center', bbox_to_anchor=(0.5, -0.05),
                         ncol=4, fontsize=8, frameon=False)
        self.figure.subplots_adjust(left=0.08, right=0.92, top=0.83, bottom=0.12)
        if path:
            self.figure.savefig(path, dpi=130)
        if self.show:
            self.plt.pause(0.001)

    def close(self):
        """Release the figure."""
        self.plt.close(self.figure)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def parse_triplet(text):
    """'a,b,c' -> [a, b, c] as floats."""
    values = [float(v) for v in text.split(',')]
    if len(values) != 3:
        raise argparse.ArgumentTypeError('expected three comma-separated numbers')
    return values


def build_parser():
    """Command-line options."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n', maxsplit=1)[0])
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--timeout', type=float, default=10.0)
    where = parser.add_mutually_exclusive_group()
    where.add_argument('--role-name', default=None,
                       help="put the antenna on the vehicle with this role_name (default: 'hero')")
    where.add_argument('--actor-id', type=int, help='put the antenna on this actor')
    where.add_argument('--location', type=parse_triplet,
                       help='antenna at this CARLA world location x,y,z [m]')
    where.add_argument('--offline', type=parse_triplet, metavar='LAT,LON,ALT',
                       help='no CARLA server: sky above this geodetic position, no occlusion')
    parser.add_argument('--antenna-height', type=float, default=0.05,
                        help='antenna height above the top of the actor bounding box [m]')
    parser.add_argument('--nav', nargs='+', default=[DEFAULT_NAV],
                        help='RINEX 3/4 navigation file(s) (default: bundled sample)')
    parser.add_argument('--no-nav', action='store_true',
                        help='no RINEX constellation: only the --satellites ones')
    parser.add_argument('--satellites',
                        help='JSON file of satellites placed by ECEF position or azimuth/elevation; '
                             'azimuth/elevation is relative to the map geo-reference '
                             '(or the --offline position)')
    parser.add_argument('--time', type=gnss_orbit.parse_utc,
                        help='UTC epoch, e.g. 2026-03-31T14:00:00Z (default: last ephemeris epoch)')
    parser.add_argument('--systems', default='GEJC',
                        help='constellations to use: G=GPS E=Galileo J=QZSS C=BeiDou')
    parser.add_argument('--elevation-mask', type=float, default=5.0, help='[deg]')
    parser.add_argument('--include-unhealthy', action='store_true')
    parser.add_argument('--ray-length', type=float, default=1000.0, help='[m]')
    parser.add_argument('--self-clearance', type=float, default=3.0,
                        help='ignore vehicle hits closer than this to the antenna [m]')
    parser.add_argument('--draw-rays', action='store_true', help='draw the rays with world.debug')
    parser.add_argument('--output', default='gnss_skyplot.png', help="PNG path ('' to skip)")
    parser.add_argument('--json', help='also write the per-satellite results to this file')
    parser.add_argument('--show', action='store_true', help='show the plot in a window')
    parser.add_argument('--watch', action='store_true',
                        help='keep updating, advancing GNSS time with the simulation clock')
    parser.add_argument('--period', type=float, default=1.0,
                        help='simulated seconds between updates with --watch')
    return parser


class SkyReport:
    """Computes the sky for an epoch and position and publishes it."""

    def __init__(self, args):
        self.args = args
        self.systems = tuple(s for s in args.systems.upper() if s in gnss_orbit.SUPPORTED_SYSTEMS)
        self.sources = []
        self.start = args.time
        if not args.no_nav:
            nav = gnss_orbit.NavData.from_files(*args.nav)
            self.sources.append(nav)
            self.start = self.start or nav.time_span()[1]
            if self.start is None:
                raise RuntimeError('no usable ephemeris in the navigation file')
            if nav.skipped_systems:
                names = sorted(gnss_orbit.CONSTELLATION_NAMES.get(s, s) for s in nav.skipped_systems)
                print('note: not modelled yet, skipped: ' + ', '.join(names))
        if self.start is None:  # directly placed satellites only: time is free
            self.start = datetime.datetime.now(datetime.timezone.utc).replace(tzinfo=None)
        self.plot = None
        if args.output or args.show:
            if matplotlib is None:
                print('matplotlib is not installed: printing the table only')
            else:
                self.plot = SkyPlot(show=args.show)

    def add_direct_satellites(self, reference):
        """Load --satellites; azimuth/elevation entries are relative to
        `reference` (lat, lon, height). They override RINEX ones by id."""
        if self.args.satellites:
            self.sources.append(gnss_orbit.SatelliteSet.from_json(self.args.satellites, reference))
        if not self.sources:
            raise RuntimeError('no satellites: drop --no-nav or give --satellites')

    def sky(self, geodetic, utc):
        """Satellites above the elevation mask."""
        return gnss_orbit.visible_satellites(
            self.sources, geodetic, utc, self.args.elevation_mask, systems=self.systems,
            healthy_only=not self.args.include_unhealthy)

    def publish(self, utc, geodetic, sky, results):
        """Print, plot and optionally dump the results as JSON."""
        rows = summarize(sky, results)
        header = (f"{utc:%Y-%m-%d %H:%M:%S} UTC  lat {geodetic[0]:.6f} "
                  f"lon {geodetic[1]:.6f} h {geodetic[2]:.1f} m")
        print_table(header, rows)
        if self.plot is not None:
            self.plot.update(rows, header, self.args.elevation_mask, self.args.output or None)
        if self.args.json:
            with open(self.args.json, 'w', encoding='utf-8') as stream:
                json.dump({'utc': utc.isoformat() + 'Z',
                           'receiver': dict(zip(('latitude', 'longitude', 'altitude'), geodetic)),
                           'satellites': rows}, stream, indent=2)


def run_carla(args, report):
    """Connect to the server and check occlusion once, or keep watching."""
    if carla is None:
        raise RuntimeError('the carla package is not installed (use --offline without a server)')
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    scene = SceneOcclusion(world, args.ray_length, args.self_clearance)
    report.add_direct_satellites(scene.geodetic(carla.Location()))
    actor = None if args.location else find_actor(world, args.actor_id, args.role_name)

    def update(utc, draw_life):
        if actor is None:
            location = carla.Location(*args.location)
        else:
            location = antenna_location(actor, args.antenna_height)
        geodetic = scene.geodetic(location)
        sky = report.sky(geodetic, utc)
        results = scene.check(location, sky, draw_life if args.draw_rays else None)
        report.publish(utc, geodetic, sky, results)

    if not args.watch:
        update(report.start, 30.0)
        return
    first, next_update = None, 0.0
    while True:
        elapsed = world.wait_for_tick(seconds=args.timeout).timestamp.elapsed_seconds
        first = elapsed if first is None else first
        if elapsed - first >= next_update:
            next_update = elapsed - first + args.period
            update(report.start + datetime.timedelta(seconds=elapsed - first), args.period)


def main():
    """Entry point."""
    args = build_parser().parse_args()
    if not args.offline and not args.location and args.actor_id is None:
        args.role_name = args.role_name or 'hero'
    report = SkyReport(args)
    if args.offline:
        geodetic = tuple(args.offline)
        report.add_direct_satellites(geodetic)
        report.publish(report.start, geodetic, report.sky(geodetic, report.start), {})
        if args.show:
            report.plot.plt.show(block=True)
        return
    try:
        run_carla(args, report)
    except KeyboardInterrupt:
        pass
    finally:
        if report.plot is not None:
            report.plot.close()


if __name__ == '__main__':
    main()
