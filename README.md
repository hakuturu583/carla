# Chrono runtime evidence

Runtime verification artefacts for
[carla-simulator/carla#9861](https://github.com/carla-simulator/carla/pull/9861)
(Port the Project Chrono integration from ue4-dev).

| File | What it is |
|---|---|
| `chrono_runtime_log.txt` | Output of the check below, run against a headless CARLA UE5 server |
| `chrono_vehicle.png` | Camera frame of the vehicle driving under Chrono physics in Town10HD_Opt |
| `verify_chrono.py` | The script that produced them |
| `carla_ue5_chrono.mp4` | Chase-cam capture: 6 s under Chrono, 6 s after `restore_physx_physics()` |
| `manual_control_chrono.mp4` | `PythonAPI/examples/manual_control_chrono.py` driven with real key events under Xvfb |
| `manual_control_chrono_log.txt` | Provenance checksums plus client and server logs for that session |
| `drive_manual_control.sh` | The harness that drove the example (xdotool key injection) |

This branch exists only to host these files; it is not part of the PR.
