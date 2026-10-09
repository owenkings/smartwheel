#pragma once
// Same UID-wide namespace as wc_runtime.project_paths.shared_lock_root().
// No ROS/vendor dependency: native direct launches retain the same exclusion.
#include <cerrno>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

namespace wc_xt_driver {
inline std::filesystem::path shared_lock_root() {
  const auto root = std::filesystem::path("/tmp")/("wc-locks-"+std::to_string(::geteuid()));
  if (::mkdir(root.c_str(), 0700) != 0 && errno != EEXIST)
    throw std::runtime_error("cannot create shared resource lock directory");
  struct stat info{};
  if (::lstat(root.c_str(), &info) != 0 || !S_ISDIR(info.st_mode) ||
      info.st_uid != ::geteuid() || (info.st_mode & 0777) != 0700)
    throw std::runtime_error("shared resource lock directory must be ordinary, owned and mode 0700");
  return root;
}

class HostResourceLock {
 public:
  explicit HostResourceLock(const std::string& name) {
    if (name.empty() || name.size()>200 || name=="." || name==".." ||
        name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.-") != std::string::npos)
      throw std::invalid_argument("invalid shared resource lock name");
    const auto path = shared_lock_root()/name;
    fd_ = ::open(path.c_str(), O_RDWR|O_CREAT|O_CLOEXEC|O_NOFOLLOW, 0600);
    struct stat info{};
    if (fd_<0 || ::fstat(fd_, &info) != 0 || !S_ISREG(info.st_mode) ||
        info.st_uid != ::geteuid() || (info.st_mode & 0022) != 0 || ::flock(fd_, LOCK_EX|LOCK_NB)) {
      if (fd_>=0) ::close(fd_);
      fd_=-1;
      throw std::runtime_error("resource lock busy or invalid: "+path.string());
    }
    const auto pid=std::to_string(::getpid())+"\n";
    if (::ftruncate(fd_,0) || ::write(fd_,pid.data(),pid.size()) != static_cast<ssize_t>(pid.size())) {
      ::close(fd_); fd_=-1;
      throw std::runtime_error("cannot write lock ownership");
    }
  }
  ~HostResourceLock() {if(fd_>=0) {::flock(fd_,LOCK_UN);::close(fd_);}}
  HostResourceLock(const HostResourceLock&)=delete;
  HostResourceLock& operator=(const HostResourceLock&)=delete;
 private:
  int fd_=-1;
};
}  // namespace wc_xt_driver
