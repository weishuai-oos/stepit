#include <algorithm>
#include <cmath>

#include <stepit/agent.h>
#include <stepit/policy_neuro/cmd_vel_source.h>

namespace stepit {
namespace neuro_policy {
namespace {
float applyDeadzone(float value, float deadzone) {
  const float magnitude = std::abs(value);
  if (magnitude <= deadzone) return 0.0F;

  const float scale = std::max(1.0F - deadzone, static_cast<float>(kEPS));
  const float normalized = clamp((magnitude - deadzone) / scale, 0.0F, 1.0F);
  return std::copysign(normalized, value);
}

void enforceCardinalLinearCommand(Arr3f &cmd_vel, const Arr3f &target_cmd_vel) {
  const bool has_linear_command = target_cmd_vel.template head<2>().abs().maxCoeff() > kEPS;
  if (not has_linear_command) {
    cmd_vel[0] = 0.0F;
    cmd_vel[1] = 0.0F;
    return;
  }

  if (std::abs(target_cmd_vel[0]) >= std::abs(target_cmd_vel[1])) {
    cmd_vel[1] = 0.0F;
  } else {
    cmd_vel[0] = 0.0F;
  }
}
}  // namespace

constexpr std::array<const char *, CmdVelSource::kNumModes> CmdVelSource::kModeName;

// clang-format off
const std::map<std::string, CmdVelSource::Action> CmdVelSource::kActionMap = {
    {"SetVelocity",         Action::kSetVelocity},
    {"SetVelocityUnscaled", Action::kSetVelocityUnscaled},
    {"SetTurboRatio",       Action::kSetTurboRatio},
    {"SelectMode",          Action::kSelectMode},
    {"CycleMode",           Action::kCycleMode},
    {"EnableSmoothing",     Action::kEnableSmoothing},
    {"DisableSmoothing",    Action::kDisableSmoothing},
    {"SetMaxAccel",         Action::kSetMaxAccel},
    {"EnableJoystick",      Action::kEnableJoystick},
    {"DisableJoystick",     Action::kDisableJoystick},
};
// clang-format on

CmdVelSource::CmdVelSource(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec)
    : Module(policy_spec, ModuleSpec(module_spec, "cmd_vel")) {
  cmd_vel_id_   = registerProvision("cmd_vel", 3);
  cmd_stall_id_ = registerProvision("cmd_stall", 1);
  timestep_     = 1.0F / static_cast<float>(policy_spec.control_freq);

  if (config_.hasValue()) {
    config_["velocity_scale_factor"].to(velocity_scale_factor_, true);
    config_["velocity_turbo_factor"].to(velocity_turbo_factor_, true);
    config_["velocity_deadzone"].to(velocity_deadzone_, true);
    config_["smoothing"].to(smoothing_, true);
    config_["clamp_unscaled_velocity_norm"].to(clamp_unscaled_velocity_norm_, true);
    config_["max_acceleration"].to(max_acceleration_, true);
    config_["stall_mode_enabled"].to(mode_enabled_[kStall], true);
    config_["move_mode_enabled"].to(mode_enabled_[kMove], true);
    config_["joystick_enabled"].to(joystick_enabled_, true);
    config_["joystick_direction_deadzone"].to(joystick_direction_deadzone_, true);
    config_["joystick_direction_release_deadzone"].to(joystick_direction_release_deadzone_, true);
    config_["joystick_direction_switch_margin"].to(joystick_direction_switch_margin_, true);
    config_["joystick_linear_scale"].to(joystick_linear_scale_, true);
    config_["joystick_yaw_deadzone"].to(joystick_yaw_deadzone_, true);
    config_["joystick_yaw_scale"].to(joystick_yaw_scale_, true);
  }
}

bool CmdVelSource::reset() {
  mode_ = kAuto;
  cmd_vel_.setZero();
  target_cmd_vel_.setZero();
  cmd_stall_            = true;
  velocity_turbo_ratio_ = 0.0;
  joystick_linear_direction_ = JoystickLinearDirection::kNone;

  joystick_rules_.clear();
  joystick_rules_.emplace_back([this](const joystick::State &js) -> std::string {
    if (not joystick_enabled_) return "";
    const float forward = applyDeadzone(-js.las_y(), joystick_direction_deadzone_);
    const float lateral = applyDeadzone(-js.las_x(), joystick_direction_deadzone_);
    float wz = applyDeadzone(-js.ras_x(), joystick_yaw_deadzone_) * joystick_yaw_scale_;

    float vx = 0.0F;
    float vy = 0.0F;
    switch (selectJoystickLinearDirection(forward, lateral)) {
      case JoystickLinearDirection::kForward:
      case JoystickLinearDirection::kBackward:
        vx = forward * joystick_linear_scale_[0];
        break;
      case JoystickLinearDirection::kLeft:
      case JoystickLinearDirection::kRight:
        vy = lateral * joystick_linear_scale_[1];
        break;
      case JoystickLinearDirection::kNone:
        break;
    }

    return fmt::format("Policy/CmdVel/SetVelocity:{},{},{}", vx, vy, wz);
  });
  return true;
}

auto CmdVelSource::selectJoystickLinearDirection(float forward, float lateral) -> JoystickLinearDirection {
  const float forward_mag = std::abs(forward);
  const float lateral_mag = std::abs(lateral);
  const float max_mag     = std::max(forward_mag, lateral_mag);

  if (max_mag <= joystick_direction_release_deadzone_) {
    joystick_linear_direction_ = JoystickLinearDirection::kNone;
    return joystick_linear_direction_;
  }

  auto forward_direction = forward >= 0.0F ? JoystickLinearDirection::kForward : JoystickLinearDirection::kBackward;
  auto lateral_direction = lateral >= 0.0F ? JoystickLinearDirection::kLeft : JoystickLinearDirection::kRight;

  if (joystick_linear_direction_ == JoystickLinearDirection::kNone) {
    joystick_linear_direction_ = forward_mag >= lateral_mag ? forward_direction : lateral_direction;
    return joystick_linear_direction_;
  }

  const bool holding_forward = joystick_linear_direction_ == JoystickLinearDirection::kForward or
                               joystick_linear_direction_ == JoystickLinearDirection::kBackward;
  const float held_mag      = holding_forward ? forward_mag : lateral_mag;
  const float contender_mag = holding_forward ? lateral_mag : forward_mag;

  if (held_mag <= joystick_direction_release_deadzone_ or
      contender_mag > held_mag + joystick_direction_switch_margin_) {
    joystick_linear_direction_ = holding_forward ? lateral_direction : forward_direction;
  } else if (holding_forward) {
    joystick_linear_direction_ = forward_direction;
  } else {
    joystick_linear_direction_ = lateral_direction;
  }

  return joystick_linear_direction_;
}

bool CmdVelSource::update(const LowState &low_state, ControlRequests &requests, FieldMap &context) {
  for (auto &&request : requests.filterByChannel("Policy/CmdVel")) {
    handleControlRequest(std::move(request));
  }

  if (smoothing_) {
    cmd_vel_ += (target_cmd_vel_ - cmd_vel_)
                    .cwiseMin(max_acceleration_ * timestep_)
                    .cwiseMax(-max_acceleration_ * timestep_);
  } else {
    cmd_vel_ = target_cmd_vel_;
  }
  enforceCardinalLinearCommand(cmd_vel_, target_cmd_vel_);

  switch (mode_) {
    case kAuto:
      cmd_stall_ = cmd_vel_.abs().maxCoeff() < velocity_deadzone_ and target_cmd_vel_.abs()
                                                                              .maxCoeff() < velocity_deadzone_;
      if (cmd_stall_) cmd_vel_.setZero();
      break;
    case kStall:
      cmd_stall_ = true;
      cmd_vel_.setZero();
      break;
    case kMove:
      cmd_stall_ = false;
      break;
    default:
      STEPIT_UNREACHABLE();
      break;
  }

  context[cmd_vel_id_]   = cmd_vel_;
  context[cmd_stall_id_] = Arr1f{cmd_stall_};
  return true;
}

void CmdVelSource::exit() { joystick_rules_.clear(); }

void CmdVelSource::handleControlRequest(ControlRequest request) {
  auto action = lookupAction(request.action(), kActionMap);
  switch (action) {
    case Action::kSetVelocity:
    case Action::kSetVelocityUnscaled: {
      float vx, vy, wz;
      if (not request.parseArgument("%f,%f,%f", vx, vy, wz) or
          (not std::isfinite(vx) or not std::isfinite(vy) or not std::isfinite(wz))) {
        request.response(kIncorrectArgument);
        break;
      }
      if (action == Action::kSetVelocity) {
        target_cmd_vel_ = Arr3f{vx, vy, wz};
      } else {
        Vec3f unscaled_cmd_vel{vx, vy, wz};
        Arr3f velocity_scale_factor;
        if (clamp_unscaled_velocity_norm_) {
          unscaled_cmd_vel = unscaled_cmd_vel / std::max(1.0F, unscaled_cmd_vel.norm());
        } else {
          unscaled_cmd_vel = unscaled_cmd_vel.array().cwiseMin(1.0F).cwiseMax(-1.0F).matrix();
        }
        for (Eigen::Index i{}; i < velocity_scale_factor.size(); ++i) {
          velocity_scale_factor[i] = unscaled_cmd_vel[i] >= 0.0F ? velocity_scale_factor_[i][0]
                                                                 : velocity_scale_factor_[i][1];
        }
        target_cmd_vel_ = unscaled_cmd_vel.array()
                              .cwiseProduct(velocity_scale_factor)
                              .cwiseProduct(1 + velocity_turbo_factor_ * velocity_turbo_ratio_);
      }
      request.response(kSuccess);
      break;
    }
    case Action::kSetTurboRatio: {
      float turbo_ratio;
      if (not request.parseArgument("%f", turbo_ratio) or
          (not std::isfinite(turbo_ratio) or turbo_ratio < -kEPS or turbo_ratio > 1.0F + kEPS)) {
        request.response(kIncorrectArgument);
        break;
      }
      turbo_ratio = clamp(turbo_ratio, 0.0F, 1.0F);
      // Apply new turbo ratio
      target_cmd_vel_ = target_cmd_vel_.cwiseQuotient(1 + velocity_turbo_factor_ * velocity_turbo_ratio_)
                            .cwiseProduct(1 + velocity_turbo_factor_ * turbo_ratio);
      velocity_turbo_ratio_ = turbo_ratio;
      request.response(kSuccess);
      break;
    }
    case Action::kSelectMode: {
      auto it = std::find_if(kModeName.begin(), kModeName.end(),
                             [&request](const char *mode) { return request.argument() == mode; });
      if (it == kModeName.end()) {
        request.response(kIncorrectArgument, fmt::format("Unknown mode: '{}'.", request.argument()));
      } else {
        mode_ = static_cast<Mode>(std::distance(kModeName.begin(), it));
        request.response(kSuccess, fmt::format("Switched to '{}' mode.", kModeName[mode_]));
      }
      break;
    }
    case Action::kCycleMode: {
      do {
        mode_ = static_cast<Mode>((static_cast<int>(mode_) + 1) % kNumModes);
      } while (not mode_enabled_[mode_]);
      request.response(kSuccess, fmt::format("Switched to '{}' mode.", kModeName[mode_]));
      break;
    }
    case Action::kEnableSmoothing: {
      smoothing_ = true;
      request.response(kSuccess);
      break;
    }
    case Action::kDisableSmoothing: {
      smoothing_ = false;
      request.response(kSuccess);
      break;
    }
    case Action::kSetMaxAccel: {
      float ax, ay, az;
      if (not request.parseArgument("%f,%f,%f", ax, ay, az) or
          (not std::isfinite(ax) or not std::isfinite(ay) or not std::isfinite(az)) or (ax < 0 or ay < 0 or az < 0)) {
        request.response(kIncorrectArgument);
        break;
      }
      max_acceleration_ = Arr3f{ax, ay, az};
      request.response(kSuccess);
      break;
    }
    case Action::kEnableJoystick: {
      joystick_enabled_ = true;
      request.response(kSuccess);
      break;
    }
    case Action::kDisableJoystick: {
      joystick_enabled_ = false;
      request.response(kSuccess);
      break;
    }
    default: {
      request.response(kUnrecognizedRequest);
      break;
    }
  }
}

STEPIT_REGISTER_MODULE(cmd_vel_source, kDefPriority - 1, Module::make<CmdVelSource>);
STEPIT_REGISTER_FIELD_SOURCE(cmd_vel, kDefPriority - 1, Module::make<CmdVelSource>);
STEPIT_REGISTER_FIELD_SOURCE(cmd_stall, kDefPriority - 1, Module::make<CmdVelSource>);
}  // namespace neuro_policy
}  // namespace stepit
