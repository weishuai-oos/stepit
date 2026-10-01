#!/usr/bin/env bash
set -euo pipefail

if [[ -t 1 ]]; then
	GREEN=$'\033[0;32m'
	YELLOW=$'\033[0;33m'
	RED=$'\033[0;31m'
	CLEAR=$'\033[0m'
else
	GREEN=""
	YELLOW=""
	RED=""
	CLEAR=""
fi

log() {
	printf "%b\n" "$*"
}

die() {
	log "${RED}ERROR:${CLEAR} $*" >&2
	exit 1
}

quote_cmd() {
	printf '%q ' "$@"
	printf '\n'
}

usage() {
	cat <<'EOF'
Usage:
  ./scripts/run_global_goal_waypoints.sh [smooth|arc|line] [options]

Modes:
  smooth                 Smooth Hermite planner. Default and recommended first test.
  arc                    Guarded curvature-limited planner.
  line                   Original straight-line planner.

Common options:
  --mode MODE            Same as positional mode: smooth, arc, or line.
  --v-des VALUE          Desired path speed in m/s. Default: 0.50
  --min-speed VALUE      Minimum speed before reaching the goal. Default: 0.08
  --publish-rate VALUE   Publish rate in Hz. Default: 50.0
  --target-tolerance M   Position tolerance for reached goal. Default: 0.08
  --heading-tolerance R  Final heading tolerance in rad. Default: 0.10
  --yaw-rate VALUE       Final in-place heading alignment rate in rad/s. Default: 0.80
  --tracking-yaw-rate R  Navigation-stage yaw limit in rad/s. Default: 0.45
                         Applies to smooth and arc, not line.
  --final-heading-distance M
                         Distance where final RViz yaw starts blending in. Default: 0.80
  --use-goal-heading     Use RViz goal orientation. Default.
  --no-goal-heading      Ignore RViz goal orientation while navigating.
  --hold-goal            Keep holding the last goal after reaching it. Default.
  --clear-on-reached     Clear the goal after reaching it.
  --odom-topic TOPIC     Default: /odom
  --goal-topic TOPIC     Default: /goal_pose
  --waypoints-topic TOPIC
                         Default: /waypoints_b
  --remain-time-topic TOPIC
                         Default: /remain_time
  --debug-path-topic TOPIC
                         Default depends on mode.
  --remain-time LIST     Comma list, e.g. 0.5,1.0,1.5,2.0,2.5
  --ros-setup FILE       ROS setup.bash to source. Default: /opt/ros/humble/setup.bash
  --no-source            Do not source ROS setup before launching.
  --extra-param KEY:=VAL Add a raw ROS parameter override. Repeatable.
  --print                Print the command instead of executing.
  -h, --help             Show this help.

Planner safety options:
  --slowdown-distance M  Smooth/line slowdown distance. Default: 1.00
  --max-first-distance M First waypoint distance cap through speed limit. Default: 0.60
  --max-segment-speed V  Inter-waypoint speed cap. Default: 1.20
  --max-segment-accel A  Inter-waypoint acceleration cap. Default: 4.00
  --max-waypoint-distance M
                         Hard waypoint distance guard. Default: 1.35
  --max-heading-error R  Heading-to-chord guard. Default: 1.20

Smooth-only options:
  --max-departure-heading R
                         Start tangent heading cap. Default: 0.80

Arc-only options:
  --min-turn-radius M    Arc minimum turn radius. Default: 1.00
  --max-path-length-ratio R
                         Fall back to guarded local arc if Dubins path is too long. Default: 2.50
  --max-guarded-turn-angle R
                         Guarded local arc angle cap. Default: 1.20
EOF
}

mode="smooth"
v_des="0.50"
min_speed="0.08"
publish_rate="50.0"
target_tolerance="0.08"
heading_tolerance="0.10"
yaw_rate_des="0.80"
max_tracking_yaw_rate="0.45"
slowdown_distance="1.00"
final_heading_distance="0.80"
max_first_distance="0.60"
max_segment_speed="1.20"
max_segment_acceleration="4.00"
max_waypoint_distance="1.35"
max_heading_error="1.20"
max_departure_heading="0.80"
min_turn_radius="1.00"
max_path_length_ratio="2.50"
max_guarded_turn_angle="1.20"
remain_time="0.5,1.0,1.5,2.0,2.5"
odom_topic="/odom"
goal_topic="/goal_pose"
waypoints_topic="/waypoints_b"
remain_time_topic="/remain_time"
debug_path_topic=""
use_goal_heading="true"
hold_goal="true"
ros_setup="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
source_ros=true
print_only=false
extra_params=()

while [[ $# -gt 0 ]]; do
	case "$1" in
		smooth|arc|line)
			mode="$1"
			shift
			;;
		--mode)
			[[ $# -ge 2 ]] || die "--mode requires a value"
			mode="$2"
			shift 2
			;;
		--v-des)
			[[ $# -ge 2 ]] || die "--v-des requires a value"
			v_des="$2"
			shift 2
			;;
		--min-speed)
			[[ $# -ge 2 ]] || die "--min-speed requires a value"
			min_speed="$2"
			shift 2
			;;
		--publish-rate)
			[[ $# -ge 2 ]] || die "--publish-rate requires a value"
			publish_rate="$2"
			shift 2
			;;
		--target-tolerance)
			[[ $# -ge 2 ]] || die "--target-tolerance requires a value"
			target_tolerance="$2"
			shift 2
			;;
		--heading-tolerance)
			[[ $# -ge 2 ]] || die "--heading-tolerance requires a value"
			heading_tolerance="$2"
			shift 2
			;;
		--yaw-rate)
			[[ $# -ge 2 ]] || die "--yaw-rate requires a value"
			yaw_rate_des="$2"
			shift 2
			;;
		--tracking-yaw-rate)
			[[ $# -ge 2 ]] || die "--tracking-yaw-rate requires a value"
			max_tracking_yaw_rate="$2"
			shift 2
			;;
		--slowdown-distance)
			[[ $# -ge 2 ]] || die "--slowdown-distance requires a value"
			slowdown_distance="$2"
			shift 2
			;;
		--final-heading-distance)
			[[ $# -ge 2 ]] || die "--final-heading-distance requires a value"
			final_heading_distance="$2"
			shift 2
			;;
		--max-first-distance)
			[[ $# -ge 2 ]] || die "--max-first-distance requires a value"
			max_first_distance="$2"
			shift 2
			;;
		--max-segment-speed)
			[[ $# -ge 2 ]] || die "--max-segment-speed requires a value"
			max_segment_speed="$2"
			shift 2
			;;
		--max-segment-accel|--max-segment-acceleration)
			[[ $# -ge 2 ]] || die "$1 requires a value"
			max_segment_acceleration="$2"
			shift 2
			;;
		--max-waypoint-distance)
			[[ $# -ge 2 ]] || die "--max-waypoint-distance requires a value"
			max_waypoint_distance="$2"
			shift 2
			;;
		--max-heading-error)
			[[ $# -ge 2 ]] || die "--max-heading-error requires a value"
			max_heading_error="$2"
			shift 2
			;;
		--max-departure-heading)
			[[ $# -ge 2 ]] || die "--max-departure-heading requires a value"
			max_departure_heading="$2"
			shift 2
			;;
		--min-turn-radius)
			[[ $# -ge 2 ]] || die "--min-turn-radius requires a value"
			min_turn_radius="$2"
			shift 2
			;;
		--max-path-length-ratio)
			[[ $# -ge 2 ]] || die "--max-path-length-ratio requires a value"
			max_path_length_ratio="$2"
			shift 2
			;;
		--max-guarded-turn-angle)
			[[ $# -ge 2 ]] || die "--max-guarded-turn-angle requires a value"
			max_guarded_turn_angle="$2"
			shift 2
			;;
		--remain-time)
			[[ $# -ge 2 ]] || die "--remain-time requires a value"
			remain_time="$2"
			shift 2
			;;
		--odom-topic)
			[[ $# -ge 2 ]] || die "--odom-topic requires a value"
			odom_topic="$2"
			shift 2
			;;
		--goal-topic)
			[[ $# -ge 2 ]] || die "--goal-topic requires a value"
			goal_topic="$2"
			shift 2
			;;
		--waypoints-topic)
			[[ $# -ge 2 ]] || die "--waypoints-topic requires a value"
			waypoints_topic="$2"
			shift 2
			;;
		--remain-time-topic)
			[[ $# -ge 2 ]] || die "--remain-time-topic requires a value"
			remain_time_topic="$2"
			shift 2
			;;
		--debug-path-topic)
			[[ $# -ge 2 ]] || die "--debug-path-topic requires a value"
			debug_path_topic="$2"
			shift 2
			;;
		--use-goal-heading)
			use_goal_heading="true"
			shift
			;;
		--no-goal-heading)
			use_goal_heading="false"
			shift
			;;
		--hold-goal)
			hold_goal="true"
			shift
			;;
		--clear-on-reached)
			hold_goal="false"
			shift
			;;
		--ros-setup)
			[[ $# -ge 2 ]] || die "--ros-setup requires a value"
			ros_setup="$2"
			shift 2
			;;
		--no-source)
			source_ros=false
			shift
			;;
		--extra-param)
			[[ $# -ge 2 ]] || die "--extra-param requires KEY:=VALUE"
			extra_params+=("$2")
			shift 2
			;;
		--print)
			print_only=true
			shift
			;;
		-h|--help)
			usage
			exit 0
			;;
		*)
			die "Unknown argument: $1 (try --help)"
			;;
	esac
done

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
case "${mode}" in
	smooth)
		planner_script="${script_dir}/smooth_global_goal_to_waypoints.py"
		[[ -n "${debug_path_topic}" ]] || debug_path_topic="/smooth_global_goal_waypoints_path"
		;;
	arc)
		planner_script="${script_dir}/arc_global_goal_to_waypoints.py"
		[[ -n "${debug_path_topic}" ]] || debug_path_topic="/arc_global_goal_waypoints_path"
		;;
	line)
		planner_script="${script_dir}/global_goal_to_waypoints.py"
		[[ -n "${debug_path_topic}" ]] || debug_path_topic="/global_goal_waypoints_path"
		;;
	*)
		die "--mode must be smooth, arc, or line"
		;;
esac

[[ -x "${planner_script}" ]] || die "Planner script is not executable: ${planner_script}"

ros_params=(
	-p "odom_topic:=${odom_topic}"
	-p "goal_topic:=${goal_topic}"
	-p "waypoints_topic:=${waypoints_topic}"
	-p "remain_time_topic:=${remain_time_topic}"
	-p "debug_path_topic:=${debug_path_topic}"
	-p "v_des:=${v_des}"
	-p "min_speed:=${min_speed}"
	-p "publish_rate:=${publish_rate}"
	-p "target_tolerance:=${target_tolerance}"
	-p "heading_tolerance:=${heading_tolerance}"
	-p "yaw_rate_des:=${yaw_rate_des}"
	-p "max_segment_speed:=${max_segment_speed}"
	-p "max_segment_acceleration:=${max_segment_acceleration}"
	-p "max_waypoint_distance:=${max_waypoint_distance}"
	-p "remain_time:=[${remain_time}]"
	-p "use_goal_heading:=${use_goal_heading}"
	-p "hold_goal:=${hold_goal}"
)

case "${mode}" in
	smooth)
		ros_params+=(
			-p "slowdown_distance:=${slowdown_distance}"
			-p "final_heading_distance:=${final_heading_distance}"
			-p "max_tracking_yaw_rate:=${max_tracking_yaw_rate}"
			-p "max_first_distance:=${max_first_distance}"
			-p "max_heading_error:=${max_heading_error}"
			-p "max_departure_heading:=${max_departure_heading}"
		)
		;;
	arc)
		ros_params+=(
			-p "final_heading_distance:=${final_heading_distance}"
			-p "max_tracking_yaw_rate:=${max_tracking_yaw_rate}"
			-p "min_turn_radius:=${min_turn_radius}"
			-p "max_path_length_ratio:=${max_path_length_ratio}"
			-p "max_guarded_turn_angle:=${max_guarded_turn_angle}"
			-p "max_first_distance:=${max_first_distance}"
			-p "max_heading_error:=${max_heading_error}"
		)
		;;
	line)
		ros_params+=(
			-p "slowdown_distance:=${slowdown_distance}"
			-p "final_heading_distance:=${final_heading_distance}"
		)
		;;
esac

for param in "${extra_params[@]}"; do
	ros_params+=(-p "${param}")
done

cmd=("${planner_script}" --ros-args "${ros_params[@]}")

log "${GREEN}Planner:${CLEAR} ${mode}"
log "${GREEN}Script:${CLEAR}  ${planner_script}"
log "${GREEN}Topics:${CLEAR}  ${goal_topic} + ${odom_topic} -> ${waypoints_topic}, ${remain_time_topic}"
log "${GREEN}Debug:${CLEAR}   ${debug_path_topic}"
log "${GREEN}Tuning:${CLEAR}  v_des=${v_des}, tracking_yaw_rate=${max_tracking_yaw_rate}, yaw_rate=${yaw_rate_des}, final_heading_distance=${final_heading_distance}"

if [[ "${print_only}" == true ]]; then
	quote_cmd "${cmd[@]}"
	exit 0
fi

if [[ "${source_ros}" == true ]]; then
	[[ -f "${ros_setup}" ]] || die "ROS setup not found: ${ros_setup}. Pass --no-source if ROS is already sourced."
	set +u
	# shellcheck disable=SC1090
	source "${ros_setup}"
	set -u
fi

exec "${cmd[@]}"
