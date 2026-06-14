#ifndef STEPIT_NEURO_POLICY_WAYPOINT_SOURCE_H_
#define STEPIT_NEURO_POLICY_WAYPOINT_SOURCE_H_

#include <vector>

#include <stepit/policy_neuro/module.h>

namespace stepit {
namespace neuro_policy {
class WaypointSource : public Module {
 public:
  WaypointSource(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec);
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;

 private:
  FieldId cmd_vel_id_{};
  FieldId waypoints_id_{};
  FieldId remain_time_id_{};

  std::vector<float> remain_time_{0.5F, 1.0F, 1.5F, 2.0F, 2.5F};
  bool unicycle_integration_{true};
  bool simple_heading_{false};
  float yaw_rate_deadzone_{1e-4F};
  float heading_deadzone_{1e-4F};
  float heading_limit_{3.1415926F};

  ArrXf waypoints_;
  ArrXf remain_time_field_;
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_WAYPOINT_SOURCE_H_
