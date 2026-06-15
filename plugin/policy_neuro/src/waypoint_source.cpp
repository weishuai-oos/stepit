#include <stepit/policy_neuro/waypoint_source.h>

namespace stepit {
namespace neuro_policy {
WaypointSource::WaypointSource(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec)
    : Module(policy_spec, ModuleSpec(module_spec, "waypoint_source")) {
  if (config_.hasValue()) {
    config_["remain_time"].to(remain_time_, true);
    config_["time_points"].to(remain_time_, true);
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
    const float t = remain_time_[i];
    float heading = wz * t;

    if (heading_limit_ > 0.0F) {
      heading = clamp(heading, -heading_limit_, heading_limit_);
    }

    const Eigen::Index offset = static_cast<Eigen::Index>(i * 3);
    waypoints_[offset + 0]    = vx * t;
    waypoints_[offset + 1]    = vy * t;
    waypoints_[offset + 2]    = heading;
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
