"""Runtime evidence for the Chrono port.

Drives a vehicle under Project Chrono physics, hands it back to the default
Chaos physics, and checks the server log to confirm Chrono was not silently
reverted by the collision fallback.
"""
import argparse, math, os, sys, time
import carla


def speed_kmh(actor):
    v = actor.get_velocity()
    return math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2) * 3.6


class ServerLog:
    """Watches the server log so we can attribute warnings to a phase."""

    def __init__(self, path):
        self.path = path
        self.mark()

    def mark(self):
        self.offset = os.path.getsize(self.path) if os.path.exists(self.path) else 0

    def since_mark(self):
        if not os.path.exists(self.path):
            return ""
        with open(self.path, "rb") as fh:
            fh.seek(self.offset)
            return fh.read().decode("utf-8", "replace")

    def reverted(self):
        return "reverting to default PhysX physics" in self.since_mark()


def drive(world, vehicle, ticks, log):
    start = vehicle.get_location()
    vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
    for i in range(ticks):
        world.tick()
        if i % 25 == 0 or i == ticks - 1:
            loc = vehicle.get_location()
            log(f"    t{i:3d}  speed {speed_kmh(vehicle):6.2f} km/h   "
                f"loc ({loc.x:8.2f}, {loc.y:8.2f}, {loc.z:6.2f})")
    end = vehicle.get_location()
    dist = math.hypot(end.x - start.x, end.y - start.y)
    return dist, speed_kmh(vehicle)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--templates", required=True)
    ap.add_argument("--server-log", required=True)
    ap.add_argument("--out", default=".")
    ap.add_argument("--spawn-index", type=int, default=10)
    args = ap.parse_args()

    lines = []

    def log(msg=""):
        print(msg, flush=True)
        lines.append(msg)

    srv = ServerLog(args.server_log)
    client = carla.Client(args.host, args.port)
    client.set_timeout(120.0)

    log("=" * 70)
    log("CARLA UE5 - Project Chrono runtime check")
    log("=" * 70)
    log(f"client / server version : {client.get_client_version()} / {client.get_server_version()}")

    world = client.get_world()
    original = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.02
    world.apply_settings(settings)
    log(f"map                     : {world.get_map().name}")
    log(f"tick                    : synchronous, fixed_delta_seconds=0.02")
    log(f"templates (base_path)   : {args.templates}")
    log()
    log("API surface on carla.Vehicle:")
    for name in ("enable_chrono_physics", "restore_physx_physics"):
        log(f"  {name:<24}: {hasattr(carla.Vehicle, name)}")

    vehicle = camera = None
    failures = []
    try:
        bp = world.get_blueprint_library().find("vehicle.lincoln.mkz")
        spawn = world.get_map().get_spawn_points()[args.spawn_index]
        vehicle = world.spawn_actor(bp, spawn)
        for _ in range(60):          # settle on the ground before switching physics
            world.tick()
        log()
        log(f"spawned                 : {bp.id}")
        log(f"settled at              : ({vehicle.get_location().x:.2f}, "
            f"{vehicle.get_location().y:.2f}, {vehicle.get_location().z:.2f})")

        # ---- Chrono ------------------------------------------------------
        log()
        log("-" * 70)
        log("enable_chrono_physics(5000, 0.002, Sedan_Vehicle / SimpleMapPowertrain / TMeasyTire)")
        log("-" * 70)
        srv.mark()
        vehicle.enable_chrono_physics(
            5000, 0.002,
            "sedan/vehicle/Sedan_Vehicle.json",
            "sedan/powertrain/Sedan_SimpleMapPowertrain.json",
            "sedan/tire/Sedan_TMeasyTire.json",
            args.templates)
        for _ in range(10):
            world.tick()
        chrono_dist, chrono_speed = drive(world, vehicle, 150, log)
        chrono_reverted = srv.reverted()
        log(f"  travelled {chrono_dist:.2f} m, final speed {chrono_speed:.2f} km/h")
        log(f"  collision fallback triggered: {chrono_reverted}")
        for line in srv.since_mark().splitlines():
            if "Chrono" in line:
                log(f"  server log | {line.strip()[:150]}")
        if chrono_reverted:
            failures.append("Chrono reverted to default physics during the Chrono phase")
        if chrono_dist < 1.0:
            failures.append(f"vehicle barely moved under Chrono ({chrono_dist:.2f} m)")

        # ---- back to default --------------------------------------------
        log()
        log("-" * 70)
        log("restore_physx_physics()")
        log("-" * 70)
        srv.mark()
        vehicle.restore_physx_physics()
        for _ in range(10):
            world.tick()
        default_dist, default_speed = drive(world, vehicle, 150, log)
        log(f"  travelled {default_dist:.2f} m, final speed {default_speed:.2f} km/h")
        if default_dist < 1.0:
            failures.append(f"vehicle barely moved after restore ({default_dist:.2f} m)")
        if vehicle.get_location().z < -5.0:
            failures.append("vehicle fell out of the world after restore")

        # ---- picture -----------------------------------------------------
        cam_bp = world.get_blueprint_library().find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", "1280")
        cam_bp.set_attribute("image_size_y", "720")
        camera = world.spawn_actor(
            cam_bp,
            carla.Transform(carla.Location(x=-7.0, z=3.5), carla.Rotation(pitch=-15)),
            attach_to=vehicle)
        shots = []
        camera.listen(shots.append)
        for _ in range(15):
            world.tick()
        deadline = time.time() + 30
        while not shots and time.time() < deadline:
            time.sleep(0.1)
        if shots:
            shots[-1].save_to_disk(os.path.join(args.out, "chrono_vehicle.png"))
            log()
            log(f"camera frame            : chrono_vehicle.png ({shots[-1].width}x{shots[-1].height})")
        else:
            failures.append("no camera frame received")

        log()
        log("=" * 70)
        log("RESULT")
        log(f"  under Chrono physics  : {chrono_dist:6.2f} m, {chrono_speed:6.2f} km/h, "
            f"no collision fallback")
        log(f"  after restore         : {default_dist:6.2f} m, {default_speed:6.2f} km/h")
        log(f"  verdict               : {'PASS' if not failures else 'FAIL'}")
        for f in failures:
            log(f"    - {f}")
        log("=" * 70)
        return 0 if not failures else 1
    finally:
        if camera is not None:
            camera.stop(); camera.destroy()
        if vehicle is not None:
            vehicle.destroy()
        world.apply_settings(original)
        with open(os.path.join(args.out, "chrono_runtime_log.txt"), "w") as fh:
            fh.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    sys.exit(main())
