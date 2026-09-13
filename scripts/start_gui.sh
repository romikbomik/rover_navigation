#!/usr/bin/env bash
set -eo pipefail
cd /home/developer/ardurover_navigation

ps -eo pid,args | awk '
  $0 ~ / gz sim / || $0 ~ /mavros_node/ || $0 ~ /sim_vehicle\.py/ ||
  $0 ~ /bin\/ardurover/ || $0 ~ /\/rviz2 / || $0 ~ /parameter_bridge/ ||
  $0 ~ /trajectory_controller_node/ || $0 ~ /path_scorer_node/ ||
  $0 ~ /ros2 launch ardurover_nav/ {
    if ($0 !~ /awk/ && $0 !~ /start_gui/) print $1
  }' | while read -r pid; do
  kill "$pid" 2>/dev/null || true
done
sleep 3

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

exec ros2 launch ardurover_nav control.launch.py gz_gui:=true \
  path_file:=/home/developer/ardurover_navigation/paths/1-drive-with-turns.path
