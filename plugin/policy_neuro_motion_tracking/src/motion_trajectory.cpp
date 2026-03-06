#include <cmath>
#include <cstdint>
#include <functional>
#include <numeric>

#include <stepit/policy_neuro/motion_trajectory.h>

namespace stepit {
namespace neuro_policy {
MotionTrajectory::MotionTrajectory(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec)
    : Module(policy_spec, ModuleSpec(module_spec, "motion_trajectory")) {
  npz_filename_ = yml::readIf<std::string>(config_, "npz_filename", name_ + ".npz");
  STEPIT_ASSERT(not npz_filename_.empty(), "'npz_filename' cannot be empty.");
  npz_.readFile((npz_filename_[0] == '/') ? npz_filename_ : joinPaths(policy_spec.home_dir, npz_filename_));

  auto default_offsets = yml::readIf(config_, "offsets", std::vector<std::int64_t>{0});
  STEPIT_ASSERT(not default_offsets.empty(), "'offsets' cannot be empty.");

  if (npz_.hasKey("fps")) {
    const auto &fps = npz_["fps"];
    STEPIT_ASSERT(fps.shape.size() == 0 or (fps.shape.size() == 1 and fps.shape[0] == 1),
                  "Expected 'fps' in '{}' to be a scalar.", npz_filename_);
    std::size_t fps_value = 0;
    if (fps.dtype == "float64") {
      fps_value = static_cast<std::size_t>(std::round(*fps.data<double>()));
    } else if (fps.dtype == "float32") {
      fps_value = static_cast<std::size_t>(std::round(*fps.data<float>()));
    } else if (fps.dtype == "int32") {
      fps_value = static_cast<std::size_t>(*fps.data<int32_t>());
    } else if (fps.dtype == "int64") {
      fps_value = static_cast<std::size_t>(*fps.data<int64_t>());
    } else {
      STEPIT_THROW("Expected 'fps' in '{}' to have dtype 'float32', 'float64', 'int32', or 'int64', but got '{}'.",
                   npz_filename_, fps.dtype);
    }
    STEPIT_ASSERT(fps_value == policy_spec.control_freq,
                  "FPS in '{}' ({}) does not match control frequency in policy spec ({}).", npz_filename_, fps_value,
                  policy_spec.control_freq);
  }

  STEPIT_ASSERT(config_["field"].IsDefined(), "Missing 'field' in '{}'.", config_filename_);
  STEPIT_ASSERT(config_["field"].IsSequence(), "Expected 'field' to be a sequence.");
  for (const auto &node : config_["field"]) {
    STEPIT_ASSERT(node.IsMap(), "Expected each field to be a map.");
    auto field_name = yml::readAs<std::string>(node, "name");
    auto key_name   = yml::readAs<std::string>(node, "key");
    auto offsets    = yml::readIf(node, "offsets", default_offsets);
    STEPIT_ASSERT(not offsets.empty(), "Offsets for field '{}' cannot be empty.", field_name);
    STEPIT_ASSERT(npz_.hasKey(key_name), "Key '{}' not found in '{}'.", key_name, npz_filename_);

    const auto &array = npz_[key_name];
    const auto &shape = array.shape;
    STEPIT_ASSERT(shape.size() >= 1, "Expected array for key '{}' to have at least 1 dimension.", key_name);
    STEPIT_ASSERT(array.dtype == "float32" or array.dtype == "float64",
                  "Expected array '{}' to have dtype 'float32' or 'float64', but got '{}'.", key_name, array.dtype);
    if (num_frames_ == 0) {
      num_frames_ = shape[0];
    } else if (num_frames_ != shape[0]) {
      STEPIT_THROW("Arrays in '{}' have different frame counts.", npz_filename_);
    }

    std::size_t frame_size = std::accumulate(shape.begin() + 1, shape.end(), 1UL, std::multiplies<std::size_t>());
    std::size_t field_size = frame_size * offsets.size();
    if (yml::hasValue(node, "size")) {
      auto specified_size = yml::readAs<std::size_t>(node, "size");
      STEPIT_ASSERT(specified_size == field_size,
                    "Field size specified ({}) does not match the stacked size ({}) of array '{}'.", specified_size,
                    field_size, key_name);
    }
    key_names_.push_back(key_name);
    frame_sizes_.push_back(frame_size);
    frame_offsets_.push_back(std::move(offsets));

    field_ids_.push_back(registerProvision(field_name, static_cast<FieldSize>(field_size)));
    field_names_.push_back(field_name);
    field_sizes_.push_back(field_size);
    field_buffers_.emplace_back(static_cast<Eigen::Index>(field_size));
  }

  STEPIT_ASSERT(num_frames_ > 0, "Loaded trajectory '{}' with 0 frames.", npz_filename_);
  STEPIT_DBUG("Loaded motion trajectory with {} frames and {} fields from '{}'.", num_frames_, field_names_.size(),
              npz_filename_);
}

bool MotionTrajectory::reset() {
  frame_idx_ = 0;
  return true;
}

bool MotionTrajectory::update(const LowState &low_state, ControlRequests &requests, FieldMap &context) {
  for (std::size_t i{}; i < field_ids_.size(); ++i) {
    const auto &key          = key_names_[i];
    const auto &offsets      = frame_offsets_[i];
    const auto field_id      = field_ids_[i];
    const auto field_size    = field_sizes_[i];
    const auto frame_size    = frame_sizes_[i];
    const auto frame_idx     = static_cast<std::int64_t>(frame_idx_);
    const auto max_frame_idx = static_cast<std::int64_t>(num_frames_ - 1);

    const auto &array = npz_[key];
    auto &buffer      = field_buffers_[i];

    Eigen::Index buffer_offset = 0;
    for (auto frame_offset : offsets) {
      auto sample_frame = static_cast<std::size_t>(clamp(frame_idx + frame_offset, std::int64_t{0}, max_frame_idx));
      std::size_t data_offset = sample_frame * frame_size;
      rArrXf buffer_segment   = buffer.segment(buffer_offset, static_cast<Eigen::Index>(frame_size));
      if (array.dtype == "float64") {
        buffer_segment = array.segment<double>(data_offset, frame_size).cast<float>();
      } else {
        buffer_segment = array.segment<float>(data_offset, frame_size);
      }
      buffer_offset += static_cast<Eigen::Index>(frame_size);
    }
    STEPIT_ASSERT(buffer_offset == buffer.size(), "Stacked field size ({}) does not match the buffer size ({}).",
                  buffer_offset, buffer.size());

    STEPIT_ASSERT(field_size == static_cast<std::size_t>(buffer.size()),
                  "Registered field size ({}) does not match the buffer size ({}).", field_size, buffer.size());
    context[field_id] = buffer;
  }

  if (frame_idx_ < num_frames_ - 1) ++frame_idx_;
  return true;
}

STEPIT_REGISTER_MODULE(motion_trajectory, kDefPriority, Module::make<MotionTrajectory>);
}  // namespace neuro_policy
}  // namespace stepit
