#pragma once
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <utility>

namespace wc_xt_driver {
// Capacity includes the item currently processed and any copy being prepared.
// Neither the producer's copy nor the consumer's work runs under this mutex.
// Diagnostics read atomics and never wait for disk, publisher, or queue locks.
template<class T> class BoundedWorkQueue {
 public:
  struct Entry { T value; size_t bytes; };
  struct Snapshot {
    uint64_t received, accepted, rejected, rejected_bytes, discarded;
    size_t capacity_frames, capacity_bytes, highwater_frames, highwater_bytes;
    size_t pending_frames, pending_bytes, active_frames;
  };
  explicit BoundedWorkQueue(size_t frames=64, size_t bytes=64*1024*1024)
      : capacity_frames_(frames), capacity_bytes_(bytes) {
    if (!frames || !bytes) throw std::invalid_argument("positive queue capacities required");
  }
  template<class Factory> bool try_copy(size_t bytes, Factory copy) {
    ++received_;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (closed_ || !bytes || bytes>capacity_bytes_ ||
          pending_frames_>=capacity_frames_ || bytes>capacity_bytes_-pending_bytes_) {
        ++rejected_; rejected_bytes_+=bytes; return false;
      }
      ++pending_frames_; pending_bytes_+=bytes;
      update_max(highwater_frames_, pending_frames_.load());
      update_max(highwater_bytes_, pending_bytes_.load());
    }
    try {
      auto value=copy(); // deep-copy outside lock, after a bounded reservation
      {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push_back(Entry{std::move(value),bytes});
        ++accepted_;
      }
      cv_.notify_one(); return true;
    } catch (...) {
      std::lock_guard<std::mutex> lock(mutex_);
      --pending_frames_; pending_bytes_-=bytes; ++rejected_; rejected_bytes_+=bytes;
      cv_.notify_all(); throw;
    }
  }
  void reject(size_t bytes) noexcept {
    ++received_;++rejected_;rejected_bytes_+=bytes;
  }
  bool push(T value, size_t bytes) {
    return try_copy(bytes,[&](){return std::move(value);});
  }
  bool take(Entry& entry, bool latest=false) {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock,[this](){return !queue_.empty() || (closed_ && pending_frames_==active_frames_);});
    if (queue_.empty()) return false;
    // Preview may explicitly retain only the latest frame; every discarded
    // accepted frame remains visible in counters. Strict recording uses FIFO.
    if(latest) while(queue_.size()>1) {
      --pending_frames_; pending_bytes_-=queue_.front().bytes;
      ++discarded_; queue_.pop_front();
    }
    entry=std::move(queue_.front()); queue_.pop_front(); ++active_frames_;
    return true;
  }
  void complete(size_t bytes) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if(!active_frames_ || !pending_frames_ || bytes>pending_bytes_)
        throw std::logic_error("queue completion without matching active item");
      --active_frames_; --pending_frames_; pending_bytes_-=bytes;
    }
    cv_.notify_all();
  }
  void discard_active(size_t bytes) {++discarded_;complete(bytes);}
  void close() { {std::lock_guard<std::mutex> lock(mutex_);closed_=true;} cv_.notify_all(); }
  void reopen() {
    std::lock_guard<std::mutex> lock(mutex_);
    if(pending_frames_)throw std::logic_error("cannot reopen undrained queue");
    closed_=false;
  }
  Snapshot snapshot() const noexcept {
    return {received_.load(),accepted_.load(),rejected_.load(),rejected_bytes_.load(),discarded_.load(),
      capacity_frames_,capacity_bytes_,highwater_frames_.load(),highwater_bytes_.load(),
      pending_frames_.load(),pending_bytes_.load(),active_frames_.load()};
  }
 private:
  static void update_max(std::atomic<size_t>& dest,size_t value) {
    auto old=dest.load();while(old<value && !dest.compare_exchange_weak(old,value)){}
  }
  const size_t capacity_frames_,capacity_bytes_;
  std::mutex mutex_;std::condition_variable cv_;std::deque<Entry> queue_;bool closed_=false;
  std::atomic<uint64_t> received_{0},accepted_{0},rejected_{0},rejected_bytes_{0},discarded_{0};
  std::atomic<size_t> pending_frames_{0},pending_bytes_{0},active_frames_{0},highwater_frames_{0},highwater_bytes_{0};
};
}
