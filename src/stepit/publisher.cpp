#include <stepit/publisher.h>

namespace stepit {
Publisher &Publisher::instance() {
  static Publisher::Ptr instance_{Publisher::make("")};
  return *instance_;
}

bool Publisher::hasStatus(const std::string &name) const {
  std::lock_guard<std::mutex> lock(status_mutex_);
  return named_status_.find(name) != named_status_.end();
}

void Publisher::registerStatus(const std::string &name) {
  std::lock_guard<std::mutex> lock(status_mutex_);
  auto result = named_status_.emplace(name, std::string{});
  STEPIT_ASSERT(result.second, "Status '{}' is already registered.", name);
}

void Publisher::updateStatus(const std::string &name, const std::string &value) {
  std::lock_guard<std::mutex> lock(status_mutex_);
  named_status_[name] = value;
}

void Publisher::removeStatus(const std::string &name) {
  std::lock_guard<std::mutex> lock(status_mutex_);
  named_status_.erase(name);
}

std::map<std::string, std::string> Publisher::getStatusSnapshot() const {
  std::lock_guard<std::mutex> lock(status_mutex_);
  return named_status_;
}

namespace publisher {
Filter::Filter() {
  getenv("STEPIT_PUBLISH_STATUS", publish_status);
  getenv("STEPIT_PUBLISH_LOW_LEVEL", publish_low_level);
  getenv("STEPIT_PUBLISH_ARRAY", publish_array);
}

const Filter g_filter;

StatusRegistration::StatusRegistration(const std::string &name) : name_(name) { publisher().registerStatus(name_); }

StatusRegistration::~StatusRegistration() {
  if (not name_.empty()) removeStatus(name_);
}

StatusRegistration::StatusRegistration(StatusRegistration &&other) noexcept : name_(std::move(other.name_)) {
  other.name_.clear();
}

StatusRegistration &StatusRegistration::operator=(StatusRegistration &&other) noexcept {
  if (this != &other) {
    if (not name_.empty()) removeStatus(name_);
    name_ = std::move(other.name_);
    other.name_.clear();
  }
  return *this;
}
}  // namespace publisher
}  // namespace stepit
