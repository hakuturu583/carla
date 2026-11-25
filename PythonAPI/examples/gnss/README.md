# GNSS sky plot and satellite occlusion (prototype)

`sensor.other.gnss` reports the true position plus Gaussian noise: it has no
satellites, so it cannot lose them behind a building, in a tunnel or under a
bridge. This prototype is the first step towards a measurement-level GNSS
model. It works out which satellites the scene would hide, entirely from the
Python client.

1. **Satellite positions.** These come from one or more sources, which can be
   combined (see [Satellite sources](#satellite-sources)):
   * a **RINEX 3/4 navigation file**, loaded in one go. `gnss_orbit.py`
     computes ECEF positions for GPS, Galileo, QZSS and BeiDou (BeiDou GEO
     included) as they move along their broadcast orbits;
   * **satellites placed directly**, by ECEF position or by azimuth and
     elevation, for deterministic test cases ("one satellite low in the east,
     behind that building") and what-if constellations.
2. **Where the antenna is on Earth.** The map's OpenDRIVE geo-reference gives
   the antenna's latitude, longitude and height through
   `Map.transform_to_geolocation`. The local east/north axes are measured
   numerically from that projection, so any projection and its grid
   convergence are handled.
3. **Occlusion.** For each satellite, one `World.cast_ray` runs from the
   antenna towards the satellite. The labels of the hits classify it:

   | Status | What the ray hit |
   |---|---|
   | `LOS` | nothing, or only the sky dome or road decals |
   | `ATTENUATED` | only vegetation, poles, signs or traffic lights |
   | `BLOCKED` | a building, wall, terrain, bridge or another vehicle (NLOS) |

   Vehicle hits within `--self-clearance` (3 m) of the antenna are taken to be
   the vehicle carrying it.

## Run

```bash
pip install numpy matplotlib
```

Put the antenna on the vehicle with `role_name=hero` and draw the rays in the
simulator:

```bash
python gnss_skyplot.py --role-name hero --draw-rays --output sky.png --json sky.json
```

Follow the vehicle, with GNSS time advancing with the simulation clock:

```bash
python gnss_skyplot.py --role-name hero --watch --period 1.0 --draw-rays
```

Use a fixed map location and a chosen UTC time:

```bash
python gnss_skyplot.py --location 10,-20,2 --time 2026-03-31T13:30:00Z
```

Run the orbit part only, with no server and no occlusion:

```bash
python gnss_skyplot.py --offline 35.681,139.767,40 --output tokyo.png
```

`--watch` follows the server clock with `wait_for_tick`. In synchronous mode,
another client has to tick the world.

## Satellite sources

From Python, both sources have the same interface. `visible_satellites` takes
one source or a list. When two sources hold the same id, the later one wins, so
a hand-placed `G24` replaces the broadcast one:

```python
import gnss_orbit

nav = gnss_orbit.NavData.from_files('data/BRDC00WRD_S_20260901200_01H_MN.rnx')

custom = gnss_orbit.SatelliteSet()
reference = (35.681, 139.767, 40.0)            # lat, lon, height
custom.set_azimuth_elevation('X01', azimuth=90.0, elevation=15.0, reference=reference)
custom.set_ecef('X02', [-14657587.6, 18534937.6, 11217467.7])

utc = gnss_orbit.parse_utc('2026-03-31T14:00:00Z')
for sat in gnss_orbit.visible_satellites([nav, custom], reference, utc, elevation_mask=5.0):
    print(sat.sat, round(sat.azimuth, 1), round(sat.elevation, 1))
```

From the command line, `--satellites` reads the same thing from JSON
([`satellites_example.json`](satellites_example.json)). Add `--no-nav` to use
only those satellites:

```json
{"satellites": [
  {"id": "X01", "azimuth": 90.0, "elevation": 15.0},
  {"id": "G99", "ecef": [-14657587.6, 18534937.6, 11217467.7]}
]}
```

With a server, azimuth and elevation are relative to the map's geo-reference,
the geolocation of `(0, 0, 0)`. With `--offline`, they are relative to the
given position. The satellite is placed 20,200 km away in that direction, so 1
km across a map changes the direction by less than 0.01°. Directly placed
satellites do not move with time.

## Tests

The tests need no CARLA server. The occlusion test replaces `carla` with a fake
world that holds one box-shaped building:

```bash
python -m unittest discover -s PythonAPI/examples/gnss -p 'test_*.py'
```

The satellite positions were checked against `gnss_lib_py` 1.0.3
(`find_sv_states`) on the same records. GPS agrees to 5 mm and QZSS to 8 cm.
Galileo drifts about 1 m per hour, which is exactly what `gnss_lib_py` using
the GPS value of GM for Galileo produces. BeiDou GEO satellites, which
`gnss_lib_py` does not model, land on their nominal orbital slots.

## Ephemeris data

`data/BRDC00WRD_S_20260901200_01H_MN.rnx` is a one-hour, multi-GNSS broadcast
ephemeris from 2026-03-31, with epochs from 11:40 to 14:00. It is merged by BKG from the
IGS real-time streams (`BCEP00BKG0`). IGS data is distributed under CC BY 4.0.
With no `--time`, the sky is computed at the last epoch in the file.

For another date, download a daily merged file such as
`BRDC00WRD_R_<yyyy><ddd>0000_01D_MN.rnx` from an IGS data centre (CDDIS, BKG
or IGN) and pass it with `--nav`. CDDIS requires a free Earthdata login.

## Limitations

These are deliberate in a prototype and are the next steps:

* Directly placed satellites are static. Moving them means setting them
  again, which a script can do every tick.
* GLONASS and SBAS records are skipped, because they broadcast state vectors
  that need numerical integration.
* Occlusion uses the scene's **simple collision**, which `cast_ray` traces
  against. Building collision in the shipped towns is often a coarse hull, so
  check it on the map you use.
* A blocked satellite is only flagged. Its reflected path (the NLOS range
  delay), diffraction at building edges and C/N0 are not computed yet.
* The map ends at its edges, so satellites low over the edge of a town see
  open sky. Use an elevation mask.
* One `cast_ray` RPC per satellite per update, about 40, is fine for a
  prototype. A sensor would do this on the server.
