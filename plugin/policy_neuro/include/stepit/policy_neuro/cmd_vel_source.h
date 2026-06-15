#ifndef STEPIT_NEURO_POLICY_CMD_VEL_SOURCE_H_
#define STEPIT_NEURO_POLICY_CMD_VEL_SOURCE_H_

#include <map>
#include <string>

#include <stepit/joystick/joystick.h>
#include <stepit/policy_neuro/module.h>

namespace stepit {
namespace neuro_policy {
class CmdVelSource : public Module {
 public:
  CmdVelSource(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;
  void exit() override;

  enum class Action : std::uint8_t {
    kSetVelocity,
    kSetVelocityUnscaled,
    kSetTurboRatio,
    kSelectMode,
    kCycleMode,
    kEnableSmoothing,
    kDisableSmoothing,
    kSetMaxAccel,
    kEnableJoystick,
    kDisableJoystick,
    kInvalid = 255,
  };

 protected:
  static const std::map<std::string, Action> kActionMap;
  virtual void handleControlRequest(ControlRequest request);

  FieldId cmd_vel_id_{};
  FieldId cmd_stall_id_{};
  std::vector<JoystickRule> joystick_rules_;

  std::vector<Arr2f> velocity_scale_factor_{{1.0, 1.0}, {0.5, 0.5}, {1.0, 1.0}};
  Arr3f velocity_turbo_factor_{Arr3f::Ones()};
  float velocity_deadzone_{0.1};
  bool smoothing_{false};
  bool clamp_unscaled_velocity_norm_{true};
  float timestep_{0.01};
  Arr3f max_acceleration_{5., 2.5, 10.};
  bool joystick_enabled_{true};
  float joystick_direction_deadzone_{0.0F};
  float joystick_direction_release_deadzone_{0.03F};
  float joystick_direction_switch_margin_{0.2F};
  Arr2f joystick_linear_scale_{0.7F, 0.35F};
  float joystick_yaw_deadzone_{0.0F};
  float joystick_yaw_scale_{1.0F};

  enum class JoystickLinearDirection : std::uint8_t { kNone, kForward, kBackward, kLeft, kRight };
  JoystickLinearDirection joystick_linear_direction_{JoystickLinearDirection::kNone};
  JoystickLinearDirection selectJoystickLinearDirection(float forward, float lateral);

  enum Mode { kAuto, kStall, kMove, kNumModes } mode_{kAuto};
  static constexpr std::array<const char *, kNumModes> kModeName{"auto", "stall", "move"};
  std::array<bool, kNumModes> mode_enabled_{true, true, true};

  Arr3f cmd_vel_{Arr3f::Zero()};
  Arr3f target_cmd_vel_{Arr3f::Zero()};
  bool cmd_stall_{false};
  double velocity_turbo_ratio_{0.0};
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_CMD_VEL_SOURCE_H_
