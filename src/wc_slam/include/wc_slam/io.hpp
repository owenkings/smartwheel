#pragma once

#include <filesystem>
#include <string>

namespace wc_slam {
std::filesystem::path safe_absolute(const std::filesystem::path &path);
std::string file_sha256(const std::filesystem::path &path);
std::string text_sha256(const std::string &text);
std::string json_string(const std::string &text);
void atomic_json_file(const std::filesystem::path &path, const std::string &text, bool immutable);

class ProcessLock {
 public:
  explicit ProcessLock(const std::filesystem::path &path);
  ~ProcessLock();
  ProcessLock(const ProcessLock &) = delete;
  ProcessLock &operator=(const ProcessLock &) = delete;
 private:
  int descriptor_ = -1;
};
}  // namespace wc_slam
