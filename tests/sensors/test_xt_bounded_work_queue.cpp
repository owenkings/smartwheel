#ifdef NDEBUG
#undef NDEBUG
#endif
#include "wc_xt_driver/bounded_work_queue.hpp"
#include "wc_xt_driver/shutdown_deadline.hpp"
#include <atomic>
#include <cassert>
#include <chrono>
#include <future>
#include <thread>
#include <vector>
#if defined(__linux__)
#include <sys/wait.h>
#include <fcntl.h>
#include <cerrno>
#include <csignal>
#include <unistd.h>
#include <cstdlib>
#endif
using namespace std::chrono_literals;
using wc_xt_driver::BoundedWorkQueue;
int main(){
  // Exact byte/frame boundaries; active work remains charged to capacity.
  BoundedWorkQueue<int> q(2,10);
  assert(q.push(1,6));assert(q.push(2,4));assert(!q.push(3,1));
  auto s=q.snapshot();assert(s.received==3 && s.accepted==2 && s.rejected==1);
  assert(s.highwater_frames==2 && s.highwater_bytes==10);
  BoundedWorkQueue<int>::Entry entry;
  assert(q.take(entry));assert(entry.value==1);assert(!q.push(3,1));
  q.complete(entry.bytes);assert(q.push(3,1));q.close();
  assert(q.take(entry) && entry.value==2);q.complete(entry.bytes);
  assert(q.take(entry) && entry.value==3);q.complete(entry.bytes);assert(!q.take(entry));
  assert(q.snapshot().pending_frames==0 && q.snapshot().pending_bytes==0);
  assert(!q.push(4,1));q.reopen();assert(q.push(5,10));q.close();
  assert(q.take(entry));q.complete(entry.bytes);

  // Default policy really enforces 64 frames and 64MiB, with no silent drop.
  BoundedWorkQueue<int> full;
  for(int i=0;i<64;++i)assert(full.push(i,1024*1024));
  assert(!full.push(65,1));full.close();
  for(int i=0;i<64;++i){assert(full.take(entry));assert(entry.value==i);full.complete(entry.bytes);}
  assert(!full.take(entry));assert(full.snapshot().discarded==0);
  BoundedWorkQueue<int> oversized;assert(!oversized.push(1,64*1024*1024+1));
  assert(oversized.snapshot().rejected_bytes==64*1024*1024+1);

  // Work deliberately blocks. Callback enqueue and atomic diagnostics return
  // promptly, and the full queue rejects rather than waiting for the worker.
  BoundedWorkQueue<int> blocked(2,10);std::promise<void> working,release;
  auto released=release.get_future();assert(blocked.push(1,5));
  std::thread worker([&](){BoundedWorkQueue<int>::Entry e;assert(blocked.take(e));
    working.set_value();released.wait();blocked.complete(e.bytes);
    while(blocked.take(e))blocked.complete(e.bytes);});
  working.get_future().wait();
  auto producer=std::async(std::launch::async,[&](){assert(blocked.push(2,5));
    assert(!blocked.push(3,1));return blocked.snapshot();});
  assert(producer.wait_for(1s)==std::future_status::ready);
  assert(producer.get().pending_frames==2);blocked.close();release.set_value();worker.join();

  // Copy reservation happens before allocation, outside the queue mutex.
  BoundedWorkQueue<std::vector<int>> copying(1,100);std::promise<void> copying_started,copy_release;
  auto allow_copy=copy_release.get_future();
  auto pending=std::async(std::launch::async,[&](){return copying.try_copy(50,[&](){
    copying_started.set_value();allow_copy.wait();return std::vector<int>{1,2,3};});});
  copying_started.get_future().wait();assert(copying.snapshot().pending_frames==1);
  bool called=false;assert(!copying.try_copy(1,[&](){called=true;return std::vector<int>{};}));assert(!called);
  copying.close();copy_release.set_value();assert(pending.get());
  BoundedWorkQueue<std::vector<int>>::Entry vector_entry;
  assert(copying.take(vector_entry));copying.complete(vector_entry.bytes);assert(!copying.take(vector_entry));
  BoundedWorkQueue<int> throwing;
  try{throwing.try_copy(20,[]()->int{throw std::runtime_error("copy failed");});assert(false);}catch(const std::runtime_error&){}
  assert(throwing.snapshot().pending_frames==0 && throwing.snapshot().rejected==1);

  // Preview latest-frame dropping is explicit, strict FIFO dropping is zero.
  BoundedWorkQueue<int> preview;for(int i=0;i<4;++i)assert(preview.push(i,1));
  preview.close();assert(preview.take(entry,true) && entry.value==3);preview.complete(entry.bytes);
  assert(preview.snapshot().discarded==3);

  // Watchdog can fire while user work is blocked; normal completion cancels it.
  std::atomic<bool> expired{false};{
    wc_xt_driver::ShutdownDeadline deadline(30ms,[&](){expired=true;});
    const auto until=std::chrono::steady_clock::now()+1s;
    while(!expired && std::chrono::steady_clock::now()<until)std::this_thread::sleep_for(1ms);
    assert(expired);
  }
  expired=false;{wc_xt_driver::ShutdownDeadline deadline(1s,[&](){expired=true;});}
  assert(!expired);
#if defined(__linux__)
  // A permanently blocked publication/IO worker cannot be detached and treated
  // as a successful close. The production deadline action is a nonzero exit.
  const pid_t pid=fork();assert(pid>=0);
  if(pid==0){
    wc_xt_driver::ShutdownDeadline deadline(30ms,wc_xt_driver::hard_shutdown_expired);
    std::promise<void> never;never.get_future().wait();std::_Exit(0);
  }
  int status=0;assert(waitpid(pid,&status,0)==pid);
  assert(WIFEXITED(status) && WEXITSTATUS(status)==74);

  // Actual production expiry helper with a full, blocking stderr pipe. Keep
  // the read end open but never drain it: attempting even one diagnostic write
  // before hard exit would hang. Parent bounds this test and reaps its own PID.
  int pipes[2];assert(pipe(pipes)==0);
  const int flags=fcntl(pipes[1],F_GETFL);assert(flags>=0);
  assert(fcntl(pipes[1],F_SETFL,flags|O_NONBLOCK)==0);
  char buffer[4096]={};
  while(true){
    const auto n=write(pipes[1],buffer,sizeof(buffer));
    if(n<0){if(errno==EINTR)continue;assert(errno==EAGAIN || errno==EWOULDBLOCK);break;}
    assert(n>0);
  }
  // Fill any residual space smaller than PIPE_BUF as well, so even a
  // one-byte write would block (not merely the previous 4096-byte writes).
  while(true){
    const auto n=write(pipes[1],buffer,1);
    if(n<0){if(errno==EINTR)continue;assert(errno==EAGAIN || errno==EWOULDBLOCK);break;}
    assert(n==1);
  }
  const pid_t blocked_stderr=fork();assert(blocked_stderr>=0);
  if(blocked_stderr==0){
    assert(dup2(pipes[1],STDERR_FILENO)==STDERR_FILENO);
    close(pipes[0]);if(pipes[1]!=STDERR_FILENO)close(pipes[1]);
    assert(fcntl(STDERR_FILENO,F_SETFL,flags & ~O_NONBLOCK)==0);
    wc_xt_driver::ShutdownDeadline deadline(30ms,wc_xt_driver::hard_shutdown_expired);
    std::promise<void> never;never.get_future().wait();std::_Exit(0);
  }
  close(pipes[1]);
  const auto test_deadline=std::chrono::steady_clock::now()+1s;
  bool exited=false;
  while(std::chrono::steady_clock::now()<test_deadline){
    const auto waited=waitpid(blocked_stderr,&status,WNOHANG);
    if(waited==blocked_stderr){exited=true;break;}
    assert(waited==0 || (waited<0 && errno==EINTR));
    std::this_thread::sleep_for(1ms);
  }
  if(!exited){kill(blocked_stderr,SIGKILL);while(waitpid(blocked_stderr,&status,0)<0 && errno==EINTR){}}
  close(pipes[0]);
  assert(exited && WIFEXITED(status) && WEXITSTATUS(status)==74);
#endif
}
