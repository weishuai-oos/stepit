#ifndef STEPIT_NEURO_POLICY_MOTION_TRAJECTORY_H_
#define STEPIT_NEURO_POLICY_MOTION_TRAJECTORY_H_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <stepit/field/data_loader.h>
#include <stepit/policy_neuro/module.h>

namespace stepit {
namespace neuro_policy {
class MotionTrajectory : public Module {
 public:
  MotionTrajectory(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec);

  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;

 private:
  std::string path_;
  std::unique_ptr<field::DataLoader> data_;
  std::size_t num_frames_{};
  std::vector<std::string> key_names_;
  std::vector<std::size_t> frame_sizes_;
  std::vector<std::vector<std::int64_t>> frame_offsets_;
  FieldIdVec field_ids_;
  std::vector<std::string> field_names_;
  std::vector<std::size_t> field_sizes_;
  std::vector<ArrXf> field_buffers_;

  std::size_t frame_idx_{};
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_MOTION_TRAJECTORY_H_
