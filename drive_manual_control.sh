#!/bin/bash
# Runs PythonAPI/examples/manual_control_chrono.py unmodified under Xvfb and
# drives it with synthetic key events, recording the screen.
#
# The example spawns at a random point, and at many of them enabling Chrono
# trips the pre-existing collision fallback immediately (carla-simulator/carla#9862).
# So: start a session, try CTRL+O, and check the server log. If Chrono was
# reverted, restart for a different spawn point. Record only a session that holds.
set -u
S="$1"
SERVER_LOG="$2"
export DISPLAY=:99
ATTEMPTS="${ATTEMPTS:-8}"

# The example handles keys on KEYUP and reads the modifier with
# pygame.key.get_mods(), which reports the CURRENT state rather than the state at
# event time. A single `xdotool key ctrl+o` releases both keys before the app
# polls, so hold ctrl across the whole tap, the way a person would.
chrono_toggle() {
  xdotool keydown ctrl; sleep 0.3; xdotool key o; sleep 0.5; xdotool keyup ctrl
}

find_window() {
  local wid=""
  for _ in $(seq 90); do
    # SDL2 sets _NET_WM_NAME but not the legacy WM_NAME that
    # `xdotool search --name` reads, so match on getwindowname instead.
    for id in $(xdotool search --onlyvisible "" 2>/dev/null); do
      case "$(xdotool getwindowname "$id" 2>/dev/null)" in
        *pygame*) wid="$id"; break ;;
      esac
    done
    [ -n "$wid" ] && { echo "$wid"; return 0; }
    kill -0 "$MC_PID" 2>/dev/null || return 1
    sleep 1
  done
  return 1
}

reverted_since() {   # $1 = byte offset into the server log
  tail -c +$(($1 + 1)) "$SERVER_LOG" | grep -qi "reverting to default"
}
enabled_since() {
  tail -c +$(($1 + 1)) "$SERVER_LOG" | grep -qi "Loading Chrono files"
}

for attempt in $(seq "$ATTEMPTS"); do
  echo "===== attempt $attempt/$ATTEMPTS ====="
  CLIENT_LOG="$S/mc_client.log"
  /mnt/sata_ssd/carla-venv/bin/python \
    /mnt/sata_ssd/carla/PythonAPI/examples/manual_control_chrono.py \
    --res 1280x720 --filter "${FILTER:-vehicle.lincoln.mkz}" \
    > "$CLIENT_LOG" 2>&1 &
  MC_PID=$!

  WID=$(find_window) || { echo "  no window"; kill -9 $MC_PID 2>/dev/null; sleep 2; continue; }
  echo "  window $WID"
  xdotool windowfocus "$WID"
  sleep 6                       # let the vehicle settle before switching physics

  OFF=$(wc -c < "$SERVER_LOG")
  chrono_toggle
  sleep 3

  if ! enabled_since "$OFF"; then
    echo "  CTRL+O did not reach the server"
    kill -9 $MC_PID 2>/dev/null; sleep 3; continue
  fi
  if reverted_since "$OFF"; then
    echo "  chrono enabled but immediately reverted at this spawn point; retrying"
    kill -9 $MC_PID 2>/dev/null; sleep 3; continue
  fi

  echo "  CHRONO HELD - recording this session"
  ffmpeg -y -loglevel error -f x11grab -framerate 30 -video_size 1280x720 -i :99.0 \
    -t 20 -c:v libx264 -preset veryfast -crf 23 -pix_fmt yuv420p \
    "$S/manual_control_chrono_raw.mp4" &
  FF_PID=$!
  sleep 2

  echo "  >>> throttle 6s under Chrono"
  xdotool keydown w; sleep 6; xdotool keyup w
  sleep 1
  if reverted_since "$OFF"; then
    echo "  hit something while driving under Chrono; retrying"
    kill -9 $FF_PID 2>/dev/null; wait $FF_PID 2>/dev/null
    kill -9 $MC_PID 2>/dev/null; sleep 3; continue
  fi
  echo "  still under Chrono after the drive"

  echo "  >>> CTRL+O : restore default physics"
  chrono_toggle
  sleep 2
  echo "  >>> throttle 6s under default"
  xdotool keydown w; sleep 6; xdotool keyup w
  sleep 1

  import -window "$WID" "$S/manual_control_chrono.png" 2>/dev/null && echo "  screenshot ok"
  wait $FF_PID 2>/dev/null
  xdotool key Escape
  sleep 4
  kill $MC_PID 2>/dev/null
  wait $MC_PID 2>/dev/null
  echo "SUCCESS_ATTEMPT=$attempt"
  exit 0
done

echo "ALL_ATTEMPTS_REVERTED"
exit 1
