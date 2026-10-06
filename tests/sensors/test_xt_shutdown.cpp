// SOFTWARE-ONLY lifecycle race test. The production process template is reused;
// the hardware node and all XT vendor includes are excluded. No SDK is created.
#define WC_XT_SHUTDOWN_TEST_ONLY
#include "../../src/wc_xt_driver/src/xt_source_node.cpp"
#include <poll.h>
#include <sys/wait.h>
#include <unistd.h>
#include <string>
#include <stdexcept>
#include <future>

namespace {
using namespace std::chrono_literals;

void require(bool value, const std::string& reason) {
  if (!value) throw std::runtime_error(reason);
}
void send_byte(int fd, char value) {
  ssize_t count;
  do {count=::write(fd,&value,1);} while(count<0 && errno==EINTR);
  require(count==1,"test control pipe write failed");
}
char read_byte(int fd) {
  const auto deadline=std::chrono::steady_clock::now()+6s;
  while(std::chrono::steady_clock::now()<deadline) {
    pollfd item{fd,POLLIN,0};
    const int ready=::poll(&item,1,100);
    if(ready<0 && errno==EINTR)continue;
    require(ready>=0,"test control pipe poll failed");
    if(ready>0) {
      char value=0;
      const auto count=::read(fd,&value,1);
      if(count<0 && errno==EINTR)continue;
      require(count==1,"child exited before completing lifecycle handshake");
      return value;
    }
  }
  throw std::runtime_error("lifecycle handshake exceeded 6 seconds");
}
void wait_for_signal(int expected) {
  const auto deadline=std::chrono::steady_clock::now()+6s;
  while(wc_xt_process::received_signal!=expected && std::chrono::steady_clock::now()<deadline)
    std::this_thread::sleep_for(1ms);
  require(wc_xt_process::received_signal==expected,"process signal handler did not survive cleanup phase");
}
void arm_signal_stage(int events, char ready_marker) {
  // Test-only observation reset: the production process never clears its stop
  // flag. The parent must wait for this marker before sending this stage's
  // signal. Without this ordering, sigterm_success could reuse the initial
  // SIGTERM as the cleanup acknowledgement and let SIGTERM race with SIGINT.
  wc_xt_process::received_signal=0;
  send_byte(events,ready_marker);
}

class SoftwareNode : public rclcpp::Node {
 public:
  SoftwareNode(std::string mode,int events,int release)
      :Node("wc_xt_software_shutdown_probe"),mode_(std::move(mode)),events_(events),release_(release) {
    timer_=create_wall_timer(25ms,[this](){
      if(mode_=="blocked_timer"){
        send_byte(events_,'T');
        // A surrogate for a permanently blocked diagnostics publish/SDK call.
        // This callback never returns, so run cannot enter node->close().
        std::promise<void> never;never.get_future().wait();
      }
      if(mode_.find("timer")==0)done_=true;
    });
    send_byte(events_,'R');
  }
  ~SoftwareNode() override {
    // This marker is after close's deliberately held SDK-cleanup surrogate.
    const char marker='D'; (void)::write(events_,&marker,1);
  }
  bool finished() const {return done_;}
  bool failed() const {return mode_=="timer_failure";}
  void close() {
    if(closed_)return;
    closed_=true;
    require(rclcpp::ok(),"ROS context was shut down before owned node cleanup");
    timer_->cancel();
    arm_signal_stage(events_,'C');
    // Hold cleanup until the parent demonstrates both dispositions still work.
    // All control I/O occurs in ordinary code, never inside the signal handler.
    wait_for_signal(SIGTERM);arm_signal_stage(events_,'A');
    wait_for_signal(SIGINT);send_byte(events_,'B');
    require(read_byte(release_)=='X',"cleanup release marker missing");
    std::this_thread::sleep_for(20ms);
    require(rclcpp::ok(),"signal unexpectedly shut the ROS context during cleanup");
    send_byte(events_,'K');
  }
 private:
  std::string mode_;
  int events_,release_;
  bool done_=false,closed_=false;
  rclcpp::TimerBase::SharedPtr timer_;
};

struct Child {
  pid_t pid=-1;
  int events=-1,release=-1;
  bool reaped=false;
  ~Child() {
    // Only this still-unreaped child can be signalled. Never signal a PID after
    // waitpid reaped it, since that PID could have been reused by another task.
    if(pid>0 && !reaped) {
      (void)::kill(pid,SIGKILL);
      int status=0;
      while(::waitpid(pid,&status,0)<0 && errno==EINTR){}
    }
    if(events>=0)::close(events);
    if(release>=0)::close(release);
  }
  void signal(int number) {require(!reaped && ::kill(pid,number)==0,"cannot signal owned child");}
  int wait(std::chrono::milliseconds timeout=6s) {
    const auto deadline=std::chrono::steady_clock::now()+timeout;
    while(std::chrono::steady_clock::now()<deadline) {
      int status=0;
      const auto result=::waitpid(pid,&status,WNOHANG);
      if(result==pid){reaped=true;return status;}
      if(result<0 && errno==EINTR)continue;
      require(result>=0,"waitpid failed");
      std::this_thread::sleep_for(5ms);
    }
    throw std::runtime_error("owned child did not exit after cleanup");
  }
};

void exercise(const char* executable,const std::string& mode) {
  int events[2],release[2];
  require(::pipe(events)==0,"pipe creation failed");
  if(::pipe(release)!=0){::close(events[0]);::close(events[1]);throw std::runtime_error("pipe creation failed");}
  Child child;
  child.pid=::fork();
  if(child.pid<0) {
    for(int fd:events)::close(fd);
    for(int fd:release)::close(fd);
    throw std::runtime_error("fork failed");
  }
  if(child.pid==0) {
    ::close(events[0]);::close(release[1]);
    const std::string event_fd=std::to_string(events[1]), release_fd=std::to_string(release[0]);
    ::execl(executable,executable,"--child",mode.c_str(),event_fd.c_str(),release_fd.c_str(),static_cast<char*>(nullptr));
    ::_exit(127);
  }
  ::close(events[1]);::close(release[0]);
  child.events=events[0];child.release=release[1];
  require(read_byte(child.events)=='R',mode+": node readiness marker missing");
  if(mode=="blocked_timer"){
    require(read_byte(child.events)=='T',"blocking timer was not entered");
    const auto stopped_at=std::chrono::steady_clock::now();
    child.signal(SIGINT);
    const int status=child.wait(12s);
    const auto elapsed=std::chrono::steady_clock::now()-stopped_at;
    require(WIFEXITED(status) && WEXITSTATUS(status)==74,
      "blocked executor did not leave through production hard deadline");
    require(elapsed>=9s && elapsed<12s,"signal deadline did not respect fixed 10s budget");
    // No close/destructor/ROS-shutdown marker may be produced. The independent
    // watchdog must terminate even though the executor callback never returned.
    char marker=0;require(::read(child.events,&marker,1)==0,
      "close unexpectedly ran while executor callback remained blocked");
    std::cout<<"PASS SOFTWARE_ONLY: blocked timer + SIGINT exited74 without close\n";
    return;
  }
  if(mode=="sigint_success")child.signal(SIGINT);
  if(mode=="sigterm_success")child.signal(SIGTERM);
  require(read_byte(child.events)=='C',mode+": cleanup did not start");
  child.signal(SIGTERM);
  require(read_byte(child.events)=='A',mode+": SIGTERM interrupted cleanup");
  child.signal(SIGINT);
  require(read_byte(child.events)=='B',mode+": SIGINT interrupted cleanup");
  send_byte(child.release,'X');
  require(read_byte(child.events)=='K',mode+": cleanup did not complete");
  require(read_byte(child.events)=='D',mode+": node destructor did not complete");
  require(read_byte(child.events)=='H',mode+": ROS shutdown did not complete");
  child.signal(SIGTERM);
  require(read_byte(child.events)=='I',mode+": handler was restored to default after ROS shutdown");
  const int status=child.wait();
  require(WIFEXITED(status),mode+": child was killed by signal instead of exiting");
  require(WEXITSTATUS(status)==(mode=="timer_failure"?1:0),mode+": incorrect failure/success exit code");
  std::cout<<"PASS SOFTWARE_ONLY: "<<mode<<" with signals during cleanup and after ROS shutdown\n";
}
}

int main(int argc,char** argv) {
  try {
    if(argc==5 && std::string(argv[1])=="--child") {
      const std::string mode=argv[2];
      const int events=std::stoi(argv[3]),release=std::stoi(argv[4]);
      char* ros_argv[]={argv[0],nullptr};
      const int code=wc_xt_process::run(1,ros_argv,[&](){return std::make_shared<SoftwareNode>(mode,events,release);});
      require(!rclcpp::ok(),"context remains alive after runner returned");
      arm_signal_stage(events,'H');
      wait_for_signal(SIGTERM);send_byte(events,'I');
      return code;
    }
    require(argc==1,"unexpected test arguments");
    for(const auto* mode:{"timer_success","sigint_success","sigterm_success","timer_failure","blocked_timer"})exercise(argv[0],mode);
    return 0;
  }catch(const std::exception& error){std::cerr<<"FAIL SOFTWARE_ONLY: "<<error.what()<<"\n";return 1;}
}
