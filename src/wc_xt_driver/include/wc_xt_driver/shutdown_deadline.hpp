#pragma once
#include <chrono>
#include <cstdlib>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
namespace wc_xt_driver {
// Must not touch any output, filesystem, mutex, allocator or destructor: even
// stderr can be a blocked pipe or filesystem. Parent observes exit code 74 and
// missing successful source summary; no logger is part of the escape path.
[[noreturn]] inline void hard_shutdown_expired() noexcept {std::_Exit(74);}

// A separate watchdog covers SDK join, worker publish, journal flush and fsync.
// The expiry action must terminate the process, never detach a live worker.
class ShutdownDeadline {
 public:
  ShutdownDeadline(std::chrono::milliseconds timeout,std::function<void()> expired)
      : watchdog_([this,timeout,expired=std::move(expired)](){
        std::unique_lock<std::mutex> lock(mutex_);
        if(!cv_.wait_for(lock,timeout,[this](){return done_;})){
          lock.unlock();expired();
        }
      }){}
  ~ShutdownDeadline(){
    {std::lock_guard<std::mutex> lock(mutex_);done_=true;}
    cv_.notify_all();watchdog_.join();
  }
  ShutdownDeadline(const ShutdownDeadline&)=delete;
 private:
  std::mutex mutex_;std::condition_variable cv_;bool done_=false;std::thread watchdog_;
};
// Starts before ROS init and lives until the process runner returns. Signal
// handlers only store a lock-free flag; this ordinary thread polls it without
// asking the executor, SDK, publisher or journal to cooperate. Once armed,
// repeated signals or observation resets cannot extend the deadline.
class ProcessStopDeadline {
 public:
  ProcessStopDeadline(std::chrono::milliseconds timeout,std::function<bool()> requested)
      : watchdog_([this,timeout,requested=std::move(requested)](){
        bool armed=false;std::chrono::steady_clock::time_point first_request;
        std::unique_lock<std::mutex> lock(mutex_);
        while(!done_) {
          if(!armed && requested()){armed=true;first_request=std::chrono::steady_clock::now();}
          if(armed && std::chrono::steady_clock::now()-first_request>=timeout){
            lock.unlock();hard_shutdown_expired();
          }
          cv_.wait_for(lock,std::chrono::milliseconds(10),[this](){return done_;});
        }
      }){}
  ~ProcessStopDeadline(){
    {std::lock_guard<std::mutex> lock(mutex_);done_=true;}
    cv_.notify_all();watchdog_.join();
  }
  ProcessStopDeadline(const ProcessStopDeadline&)=delete;
 private:
  std::mutex mutex_;std::condition_variable cv_;bool done_=false;std::thread watchdog_;
};

}
