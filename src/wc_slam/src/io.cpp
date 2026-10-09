#include "wc_slam/io.hpp"

#include <openssl/evp.h>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

#include <vector>
#include <cerrno>
#include <cstring>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>

namespace wc_slam {
namespace fs = std::filesystem;
namespace {
void fail(const std::string &action) { throw std::runtime_error(action + ": " + std::strerror(errno)); }
using Digest = std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)>;
class FileDescriptor {
 public:
  explicit FileDescriptor(int value) noexcept : value_(value) {}
  ~FileDescriptor() noexcept { if (value_ >= 0) ::close(value_); }
  FileDescriptor(const FileDescriptor &) = delete;
  FileDescriptor &operator=(const FileDescriptor &) = delete;
  int get() const noexcept { return value_; }
 private:
  int value_;
};
Digest digest() {
  Digest context(EVP_MD_CTX_new(), &EVP_MD_CTX_free);
  if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1)
    throw std::runtime_error("cannot initialize SHA256");
  return context;
}
std::string finish(Digest &context) {
  unsigned char result[EVP_MAX_MD_SIZE];
  unsigned int length = 0;
  if (EVP_DigestFinal_ex(context.get(), result, &length) != 1)
    throw std::runtime_error("cannot finalize SHA256");
  std::ostringstream text;
  for (unsigned int index = 0; index < length; ++index)
    text << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned int>(result[index]);
  return text.str();
}
void sync_parent(const fs::path &path) {
  int descriptor = ::open(path.parent_path().c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW);
  if (descriptor < 0) fail("open output directory");
  const int outcome = ::fsync(descriptor);
  ::close(descriptor);
  if (outcome != 0) fail("fsync output directory");
}
}  // namespace

fs::path safe_absolute(const fs::path &path) {
  if (!path.is_absolute()) throw std::runtime_error("runtime paths must be explicit absolute paths");
  for (const auto &part : path)
    if (part == "..") throw std::runtime_error("parent traversal is forbidden");
  fs::path current;
  for (const auto &part : path.lexically_normal()) {
    current /= part;
    std::error_code error;
    const auto state = fs::symlink_status(current, error);
    if (fs::is_symlink(state)) throw std::runtime_error("symbolic links are forbidden: " + current.string());
    if (error && error != std::errc::no_such_file_or_directory)
      throw std::runtime_error("cannot inspect runtime path: " + current.string());
  }
  return path.lexically_normal();
}

std::string file_sha256(const fs::path &path) {
  const auto checked = safe_absolute(path);
  FileDescriptor descriptor(::open(checked.c_str(), O_RDONLY | O_NOFOLLOW | O_NONBLOCK));
  if (descriptor.get() < 0) fail("open raw observation for hashing");
  struct stat state;
  if (::fstat(descriptor.get(), &state) != 0 || !S_ISREG(state.st_mode)) {
    throw std::runtime_error("hash input must be a regular file");
  }
  auto context = digest();
  // Bound memory without a 1 MiB stack frame. This also keeps unwinding across
  // the early safe_absolute rejection independent of a large stack adjustment.
  std::vector<char> buffer(1024 * 1024);
  for (;;) {
    const auto count = ::read(descriptor.get(), buffer.data(), buffer.size());
    if (count < 0 && errno == EINTR) continue;
    if (count < 0) fail("read hash input");
    if (count == 0) break;
    if (EVP_DigestUpdate(context.get(), buffer.data(), static_cast<size_t>(count)) != 1) {
      throw std::runtime_error("cannot update SHA256");
    }
  }
  return finish(context);
}

std::string text_sha256(const std::string &text) {
  auto context = digest();
  if (EVP_DigestUpdate(context.get(), text.data(), text.size()) != 1)
    throw std::runtime_error("cannot hash raw index");
  return finish(context);
}

std::string json_string(const std::string &text) {
  std::ostringstream output;
  output << '"';
  for (unsigned char byte : text) {
    switch (byte) {
      case '"': output << "\\\""; break;
      case '\\': output << "\\\\"; break;
      case '\b': output << "\\b"; break;
      case '\f': output << "\\f"; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default:
        if (byte < 0x20) output << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(byte);
        else output << byte;
    }
  }
  output << '"';
  return output.str();
}

void atomic_json_file(const fs::path &path, const std::string &text, bool immutable) {
  const auto checked = safe_absolute(path);
  fs::create_directories(checked.parent_path());
  const auto temporary = checked.string() + ".partial-" + std::to_string(::getpid());
  int descriptor = ::open(temporary.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW, 0600);
  if (descriptor < 0) fail("create new JSON temporary");
  try {
    size_t written = 0;
    while (written < text.size()) {
      const auto count = ::write(descriptor, text.data() + written, text.size() - written);
      if (count <= 0) fail("write JSON snapshot");
      written += static_cast<size_t>(count);
    }
    if (::fsync(descriptor) != 0) fail("fsync JSON snapshot");
    ::close(descriptor);
    descriptor = -1;
    if (immutable) {
      if (::link(temporary.c_str(), checked.c_str()) != 0) fail("publish immutable JSON snapshot");
      if (::unlink(temporary.c_str()) != 0) fail("unlink committed temporary");
    } else {
      safe_absolute(checked);
      if (::rename(temporary.c_str(), checked.c_str()) != 0) fail("publish current JSON snapshot");
    }
    sync_parent(checked);
  } catch (...) {
    if (descriptor >= 0) ::close(descriptor);
    ::unlink(temporary.c_str());
    throw;
  }
}

ProcessLock::ProcessLock(const fs::path &path) {
  const auto checked = safe_absolute(path);
  fs::create_directories(checked.parent_path());
  descriptor_ = ::open(checked.c_str(), O_RDWR | O_CREAT | O_NOFOLLOW | O_NONBLOCK, 0600);
  if (descriptor_ < 0) fail("open shared graph lock");
  struct stat state;
  if (::fstat(descriptor_, &state) != 0 || !S_ISREG(state.st_mode) ||
      ::flock(descriptor_, LOCK_EX | LOCK_NB) != 0) {
    ::close(descriptor_);
    descriptor_ = -1;
    throw std::runtime_error("shared graph resource is already owned or lock is invalid");
  }
}
ProcessLock::~ProcessLock() { if (descriptor_ >= 0) ::close(descriptor_); }
}  // namespace wc_slam
