#!/usr/bin/env bash
set -eo pipefail
cd /home/developer/ardurover_navigation

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

cleanup() {
  local pids
  pids=$(ps -eo pid,args | awk '
    /gz sim / || /mavros_node/ || /sim_vehicle\.py/ || /bin\/ardurover/ ||
    /\/rviz2 / || /parameter_bridge/ || /trajectory_controller_node/ || /path_scorer_node/ ||
    /ros2 launch ardurover_nav/ {
      if ($0 !~ /awk/ && $0 !~ /run_one_path/) print $1
    }')
  if [[ -n "${pids}" ]]; then
    # shellcheck disable=SC2086
    kill ${pids} 2>/dev/null || true
  fi
  sleep 2
  if [[ -n "${pids}" ]]; then
    # shellcheck disable=SC2086
    kill -9 ${pids} 2>/dev/null || true
  fi
  sleep 3
}

NAME="${1:?name}"
PATH_FILE="${2:?path}"
TIMEOUT_S="${3:-180.0}"
shift 3 || true
EXTRA_PARAMS=("$@")

echo "======== RUNNING ${NAME} ========"
cleanup
rm -f paths/score.txt

ros2 launch ardurover_nav control.launch.py gz_gui:=false \
  path_file:="${PATH_FILE}" \
  "${EXTRA_PARAMS[@]}" \
  >"/tmp/control_${NAME}.log" 2>&1 &
LAUNCH_PID=$!
echo "launch_pid=${LAUNCH_PID}"

GOT=0
LOOPS=$(( ${TIMEOUT_S%.*} / 2 + 40 ))
for i in $(seq 1 "${LOOPS}"); do
  if [[ -f paths/score.txt ]]; then
    echo "--- score ${NAME} ---"
    cat paths/score.txt
    cp paths/score.txt "paths/score_${NAME}.txt"
    GOT=1
    break
  fi
  if (( i % 10 == 0 )); then
    grep -E "Controller running|Requesting|v=" "/tmp/control_${NAME}.log" | tail -3 || true
  fi
  sleep 2
done

if [[ "${GOT}" -ne 1 ]]; then
  echo "TIMEOUT/FAIL waiting for score on ${NAME}"
  grep -E "Controller running|Requesting|v=|Goal|Score|ERROR" "/tmp/control_${NAME}.log" | tail -40 || true
fi

kill "${LAUNCH_PID}" 2>/dev/null || true
cleanup
echo "done ${NAME} got=${GOT}"
exit $(( 1 - GOT ))
