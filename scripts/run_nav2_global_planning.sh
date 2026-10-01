#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
stepit_dir="$(cd -- "${script_dir}/.." && pwd -P)"
ros_setup="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
planner_profile="navfn_xy_legacy"
launch_args=()

while (($#)); do
  case "$1" in
    --profile)
      if (($# < 2)); then
        echo "Missing value for --profile" >&2
        exit 2
      fi
      planner_profile="$2"
      shift 2
      ;;
    --profile=*)
      planner_profile="${1#--profile=}"
      shift
      ;;
    *)
      launch_args+=("$1")
      shift
      ;;
  esac
done

case "${planner_profile}" in
  navfn_xy_legacy|smac_hybrid_xy_forward|smac_terminal_yaw|smac_lattice_full_se2)
    ;;
  *)
    echo "Invalid --profile '${planner_profile}'" >&2
    echo "Allowed profiles: navfn_xy_legacy smac_hybrid_xy_forward smac_terminal_yaw smac_lattice_full_se2" >&2
    exit 2
    ;;
esac

if [[ ! -r "${ros_setup}" ]]; then
  echo "ROS setup file not found: ${ros_setup}" >&2
  exit 1
fi

if [[ -n "${CONDA_PREFIX:-}" ]]; then
  echo "INFO: Conda is active, but Nav2 and the waypoint node will use system ROS and /usr/bin/python3." >&2
fi

# This is a bash script; source setup.bash here even when the caller's login
# shell is zsh.  The Python process below is pinned to the system interpreter.
source "${ros_setup}"
set -u

for command in ros2 /usr/bin/python3; do
  command -v "${command}" >/dev/null || {
    echo "Required command not found: ${command}" >&2
    exit 1
  }
done

exec /usr/bin/python3 \
  "${stepit_dir}/nav2/launch/sim_global_planning.launch.py" \
  "planner_profile:=${planner_profile}" \
  "${launch_args[@]}"
