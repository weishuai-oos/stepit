#include <cmath>

#include <stepit/policy_neuro/waypoint_source.h>

namespace stepit {
namespace neuro_policy {
namespace {
constexpr float kPi = 3.14159265358979323846F;

float wrapToPi(float angle) {
  constexpr float kTwoPi = 2.0F * kPi;
  angle                  = std::fmod(angle + kPi, kTwoPi);
  if (angle < 0.0F) angle += kTwoPi;
  return angle - kPi;
}

float limitHeading(float heading, float limit) {
  heading = wrapToPi(heading);
  if (limit > 0.0F) {
    heading = clamp(heading, -limit, limit);
  }
  return heading;
}
}  // namespace

WaypointSource::WaypointSource(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec)
    : Module(policy_spec, ModuleSpec(module_spec, "waypoint_source")) {
  if (config_.hasValue()) {
    config_["remain_time"].to(remain_time_, true);
    config_["time_points"].to(remain_time_, true);
    config_["unicycle_integration"].to(unicycle_integration_, true);
    config_["simple_heading"].to(simple_heading_, true);
    config_["yaw_rate_deadzone"].to(yaw_rate_deadzone_, true);
    config_["heading_deadzone"].to(heading_deadzone_, true);
    config_["heading_limit"].to(heading_limit_, true);
  }

  STEPIT_ASSERT(not remain_time_.empty(), "WaypointSource requires at least one waypoint time.");

  cmd_vel_id_     = registerRequirement("cmd_vel", 3);
  waypoints_id_   = registerProvision("waypoints_b", static_cast<FieldSize>(remain_time_.size() * 3));
  remain_time_id_ = registerProvision("remain_time", static_cast<FieldSize>(remain_time_.size()));

  waypoints_.setZero(static_cast<Eigen::Index>(remain_time_.size() * 3));
  remain_time_field_.resize(static_cast<Eigen::Index>(remain_time_.size()));
  for (std::size_t i{}; i < remain_time_.size(); ++i) {
    remain_time_field_[static_cast<Eigen::Index>(i)] = remain_time_[i];
  }
}

bool WaypointSource::update(const LowState &, ControlRequests &, FieldMap &context) {
  const auto &cmd_vel = context.at(cmd_vel_id_);
  STEPIT_ASSERT(cmd_vel.size() == 3, "Field 'cmd_vel' must have size 3.");

  const float vx = cmd_vel[0];
  const float vy = cmd_vel[1];
  const float wz = cmd_vel[2];

  for (std::size_t i{}; i < remain_time_.size(); ++i) {
    const float t     = remain_time_[i];
    float x           = vx * t;
    float y           = vy * t;
    float heading     = wz * t;
    const float wt    = wz * t;
    const bool use_arc = unicycle_integration_ and std::abs(wz) > yaw_rate_deadzone_;

    if (use_arc) {
      const float sin_wt = std::sin(wt);
      const float cos_wt = std::cos(wt);
      x                  = (vx * sin_wt + vy * (cos_wt - 1.0F)) / wz;
      y                  = (vx * (1.0F - cos_wt) + vy * sin_wt) / wz;
    }

    if (heading_limit_ > 0.0F) {
      heading = clamp(heading, -heading_limit_, heading_limit_);
    }

    const Eigen::Index offset = static_cast<Eigen::Index>(i * 3);
    waypoints_[offset + 0]    = x;
    waypoints_[offset + 1]    = y;
    waypoints_[offset + 2]    = heading;
  }

  // Match MIRLab simple_heading: translational commands face along the waypoint path.
  if (simple_heading_ and std::hypot(vx, vy) > heading_deadzone_) {
    float previous_heading{};
    bool previous_heading_valid = false;

    for (std::size_t i{}; i < remain_time_.size(); ++i) {
      const Eigen::Index offset = static_cast<Eigen::Index>(i * 3);
      float dx{};
      float dy{};

      if (i == 0) {
        dx = waypoints_[offset + 0];
        dy = waypoints_[offset + 1];
      } else if (i + 1 < remain_time_.size()) {
        const Eigen::Index next_offset = static_cast<Eigen::Index>((i + 1) * 3);
        dx                             = waypoints_[next_offset + 0] - waypoints_[offset + 0];
        dy                             = waypoints_[next_offset + 1] - waypoints_[offset + 1];
      } else if (previous_heading_valid) {
        waypoints_[offset + 2] = previous_heading;
        continue;
      } else {
        const Eigen::Index prev_offset = static_cast<Eigen::Index>((i - 1) * 3);
        dx                             = waypoints_[offset + 0] - waypoints_[prev_offset + 0];
        dy                             = waypoints_[offset + 1] - waypoints_[prev_offset + 1];
      }

      if (std::hypot(dx, dy) > heading_deadzone_) {
        const float heading = limitHeading(std::atan2(dy, dx), heading_limit_);
        waypoints_[offset + 2] = heading;
        previous_heading       = heading;
        previous_heading_valid = true;
      }
    }
  }

  context[waypoints_id_]   = waypoints_;
  context[remain_time_id_] = remain_time_field_;
  return true;
}

STEPIT_REGISTER_MODULE(waypoint_source, kDefPriority, Module::make<WaypointSource>);
STEPIT_REGISTER_FIELD_SOURCE(waypoints_b, kDefPriority, Module::make<WaypointSource>);
STEPIT_REGISTER_FIELD_SOURCE(remain_time, kDefPriority, Module::make<WaypointSource>);
}  // namespace neuro_policy
}  // namespace stepit
