#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <cerrno>
#include <csignal>
#include <chrono>
#include <iostream>
#include <memory>
#include <atomic>
#include "wc_xt_driver/shutdown_deadline.hpp"
#include <system_error>
#include <thread>

namespace wc_xt_process {
// A plain lock-free atomic store is signal-safe; no log, allocation, mutex,
// ROS/SDK operation or device command ever occurs in the signal handler.
static_assert(std::atomic<std::sig_atomic_t>::is_always_lock_free,
              "signal stop flag must always be lock-free");
static std::atomic<std::sig_atomic_t> received_signal{0};
extern "C" void request_stop(int signal_number) noexcept {
  received_signal.store(signal_number,std::memory_order_relaxed);
}
static bool stop_requested() noexcept {return received_signal.load(std::memory_order_relaxed)!=0;}
static void install_stop_handlers() {
  struct sigaction action{};
  action.sa_handler = request_stop;
  ::sigemptyset(&action.sa_mask);
  ::sigaddset(&action.sa_mask, SIGINT);
  ::sigaddset(&action.sa_mask, SIGTERM);
  action.sa_flags = SA_RESTART;
  if (::sigaction(SIGINT, &action, nullptr) != 0 ||
      ::sigaction(SIGTERM, &action, nullptr) != 0)
    throw std::system_error(errno, std::generic_category(), "install sensor process stop handlers");
  // Deliberately never restore the default dispositions in this executable.
  // A second signal during SDK cleanup or ROS shutdown must remain a request,
  // not kill a partially cleaned process. The OS removes handlers at exit.
}

template<class Factory>
int run(int argc, char** argv, Factory make_node) {
  int code = 1;
  std::atomic<bool> teardown_started{false};
  // Function-scope ownership deliberately outlives ROS init, executor/node
  // cleanup, catch handling and global ROS shutdown. A blocked callback cannot
  // prevent this independent thread from observing the stop request.
  std::unique_ptr<wc_xt_driver::ProcessStopDeadline> process_deadline;
  try {
    install_stop_handlers();
    process_deadline=std::make_unique<wc_xt_driver::ProcessStopDeadline>(
      std::chrono::seconds(10),[&](){return stop_requested() || teardown_started.load();});
    // Verified against the installed ROS Humble utilities.hpp. None prevents
    // rclcpp's global shutdown from restoring SIG_DFL during our cleanup.
    rclcpp::init(argc, argv, rclcpp::InitOptions{}, rclcpp::SignalHandlerOptions::None);
    {
      auto node = make_node();
      {
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        while (rclcpp::ok() && !stop_requested() && !node->finished()) {
          // This limits dispatch batches, not time spent inside one callback.
          // Existing SDK request timeouts bound blocking sensor reads.
          executor.spin_some(std::chrono::milliseconds(20));
          if (!stop_requested() && !node->finished())
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        teardown_started=true; // also bounds normal finished/context-failure cleanup
        executor.remove_node(node);
      }
      const bool context_failed = !rclcpp::ok();
      node->close(); // stop our stream and join SDK workers while ROS is alive
      code = node->failed() || context_failed ? 1 : 0;
      node.reset(); // release remaining node resources before global shutdown
    }
  } catch (const std::exception& error) {
    teardown_started=true;
    std::cerr << "xt_source_node: " << error.what() << std::endl;
    code = 1;
  }
  try {
    if (rclcpp::ok()) rclcpp::shutdown();
  } catch (const std::exception& error) {
    std::cerr << "xt_source_node ROS shutdown: " << error.what() << std::endl;
    code = 1;
  }
  return code;
}
} // namespace wc_xt_process

// The software-only race test includes the process lifecycle above, excluding
// all vendor SDK code and the hardware node. Production uses the same template.
#ifndef WC_XT_SHUTDOWN_TEST_ONLY
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <wc_interfaces/msg/source_frame.hpp>
#include <algorithm>
#include "xtsdk.h"
#include "wc_frame_storage.h"
#include "wc_xt_driver/bounded_work_queue.hpp"
#include "wc_xt_driver/owned_stop.hpp"
#include "wc_xt_driver/acquisition_budget.hpp"
#include "wc_xt_driver/xtcfg.hpp"
#include "wc_xt_driver/device_config_policy.hpp"
#include <openssl/sha.h>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <atomic>
#include <cstdlib>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>

namespace fs = std::filesystem;
using XinTan::XtSdk;
using FrameMsg = wc_interfaces::msg::SourceFrame;
static uint64_t steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
static uint64_t system_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}
static std::string sha256(const std::string& data) {
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char*>(data.data()), data.size(), digest);
  std::ostringstream out; out << std::hex << std::setfill('0');
  for (auto byte : digest) out << std::setw(2) << unsigned(byte);
  return out.str();
}
static std::string ipv4(const uint8_t* bytes) {
  return std::to_string(bytes[0])+"."+std::to_string(bytes[1])+"."+std::to_string(bytes[2])+"."+std::to_string(bytes[3]);
}
static void check_ip(const std::string& ip) {
  in_addr value{};
  if (inet_pton(AF_INET, ip.c_str(), &value) != 1 || value.s_addr == INADDR_ANY || IN_MULTICAST(ntohl(value.s_addr)))
    throw std::invalid_argument("explicit unicast IPv4 required");
}
static bool token(const std::string& text) {
  return !text.empty() && text.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.-") == std::string::npos;
}
class Lock {
 public:
  explicit Lock(const fs::path& path) {
    fd_ = ::open(path.c_str(), O_RDWR|O_CREAT|O_CLOEXEC|O_NOFOLLOW, 0600);
    if (fd_<0 || ::flock(fd_, LOCK_EX|LOCK_NB)) {
      if(fd_>=0)::close(fd_); fd_=-1; throw std::runtime_error("resource lock busy: "+path.string());
    }
    const auto pid=std::to_string(::getpid())+"\n";
    if (::ftruncate(fd_,0) || ::write(fd_,pid.data(),pid.size()) != static_cast<ssize_t>(pid.size())) {
      ::close(fd_); fd_=-1;
      throw std::runtime_error("cannot write lock ownership");
    }
  }
  ~Lock(){if(fd_>=0){::flock(fd_,LOCK_UN);::close(fd_);}}
  Lock(const Lock&)=delete;
 private:int fd_=-1;
};

class XtSourceNode : public rclcpp::Node {
 public:
  XtSourceNode() : Node("xt_source") {
    if (!parameter<bool>("allow_hardware",false)) throw std::runtime_error("hardware acquisition disabled; use the root-managed live launch after preflight");
    side_=parameter<std::string>("side","");
    serial_=parameter<std::string>("expected_serial","");
    device_ip_=parameter<std::string>("device_ip","");
    receive_ip_=parameter<std::string>("receive_ip","");
    frame_id_=parameter<std::string>("frame_id","");
    session_id_=parameter<std::string>("session_id","");
    run_root_=parameter<std::string>("run_root","");
    const auto config_path=parameter<std::string>("source_config_path","");
    port_=parameter<int>("receive_port",7687);
    mirror_=parameter<bool>("publish_cloud_mirror",false);
    require_recorder_=parameter<bool>("require_recorder",false);
    read_only_=parameter<bool>("read_only_probe",true);
    device_policy_=wc_xt_driver::parse_device_config_policy(parameter<std::string>("device_config_policy","preserve_current"));
    deadline_seconds_=parameter<int>("connect_timeout_seconds",20);
    max_runtime_seconds_=parameter<int>("max_runtime_seconds",30);
    source_stale_seconds_=parameter<int>("source_stale_seconds",3);
    shutdown_timeout_seconds_=parameter<int>("shutdown_timeout_seconds",10);
    if(shutdown_timeout_seconds_<1 || shutdown_timeout_seconds_>60)
      throw std::invalid_argument("shutdown timeout must be 1..60 seconds");
    if((side_!="left"&&side_!="right")||!token(serial_)||!token(session_id_)||frame_id_.empty()||frame_id_.front()=='/')
      throw std::invalid_argument("explicit side, identity, frame and session required");
    check_ip(device_ip_);check_ip(receive_ip_);
    if(port_<1||port_>=10000||deadline_seconds_<1||source_stale_seconds_<0||!wc_xt_driver::valid_acquisition_runtime(max_runtime_seconds_))
      throw std::invalid_argument("port must meet SDK pre-start range 1..9999; positive connect timeout and nonnegative runtime required (0 means until stopped)");
    if(!fs::path(run_root_).is_absolute()||!fs::is_directory(run_root_))
      throw std::invalid_argument("run_root must be an existing shared absolute directory");
    fs::create_directories(fs::path(run_root_)/"locks");
    identity_lock_=std::make_unique<Lock>(fs::path(run_root_)/"locks"/("xt-"+serial_+".lock"));
    udp_lock_=std::make_unique<Lock>(fs::path(run_root_)/"locks"/("xt-udp-"+receive_ip_+"-"+std::to_string(port_)+".lock"));
    std::ifstream config(config_path,std::ios::binary);
    if(!config)throw std::runtime_error("source_config_path must name the supplied per-device config, retained read-only");
    std::ostringstream data;data<<config.rdbuf(); requested_hash_=sha256(data.str());
    requested_=wc_xt_driver::XtConfig::parse(data.str());
    epoch_=session_id_+"-"+std::to_string(::getpid())+"-"+std::to_string(steady_ns());
    session_dir_=fs::path(run_root_)/"sessions"/session_id_/side_/epoch_;
    fs::create_directories(session_dir_);
    source_journal_.open(session_dir_/"source_frames.jsonl",std::ios::out|std::ios::binary);
    if(!source_journal_)throw std::runtime_error("cannot create source-side frame journal");
    auto source_qos=rclcpp::SensorDataQoS();
    if(require_recorder_)source_qos.keep_last(128).reliable();
    source_=create_publisher<FrameMsg>("source_frame",source_qos);
    cloud_=create_publisher<sensor_msgs::msg::PointCloud2>("points_raw",rclcpp::SensorDataQoS().keep_last(2));
    filtered_source_=create_publisher<FrameMsg>("source_frame_filtered",source_qos);
    filtered_cloud_=create_publisher<sensor_msgs::msg::PointCloud2>("points_filtered",rclcpp::SensorDataQoS().keep_last(2));
    diagnostics_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics",10);
    sdk_=std::make_unique<XtSdk>((session_dir_/"sdk_log").string(),side_);
    sdk_->setStrictFrameDelivery(require_recorder_);
    sdk_->setCallback([this](const auto& event){on_event(event);},[this](const auto& frame){
      try{on_frame(frame);}catch(const std::exception& error){
        latch_pipeline_failure(PipelineFailure::Callback);
        RCLCPP_ERROR(get_logger(),"source callback: %s",error.what());
      }
    });
    if(!sdk_->setConnectIpaddress(device_ip_))throw std::runtime_error("SDK rejected device IP");
    // Existing SDK API before startup only sets host-side values and returns
    // false. The reviewed transport patch disables its connected device write.
    sdk_->setUdpDestIp(receive_ip_,static_cast<uint16_t>(port_));
    sdk_->clearAllSdkFilter(); // host-only flag clear, no device configuration
    sdk_->setPointsCornerCut(false); // retain raw input coverage
    sdk_->setSdkCloudCoordType(XinTan::ClOUDCOORD_CAR);
    started_at_=steady_ns();
    sdk_->startup();
    stop_=create_service<std_srvs::srv::Trigger>("stop",[this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
      ready_=false; paused_=true; fault_stop_attempted_=true;
      response->success=request_capture_stop("stop service");
      response->message=response->success ? "own stream stop acknowledged; explicit process restart required to resume" :
        "own stream stop unconfirmed; failure latched, explicit retry or cleanup will retry";
    });
    timer_=create_wall_timer(std::chrono::milliseconds(200),[this](){tick();});
    frame_worker_=std::thread([this](){frame_worker_loop();});
  }
  ~XtSourceNode() override { close(); }
  void close() noexcept {
    if(cleanup_started_.exchange(true))return;
    // Stop device ingress while SDK callbacks remain accepted. The final ACK
    // can race with a valid last UDP frame; do not silently drop that tail.
    wc_xt_driver::ShutdownDeadline deadline(std::chrono::seconds(shutdown_timeout_seconds_),
      wc_xt_driver::hard_shutdown_expired); // no I/O; never detach live workers
    try {
      if(timer_)timer_->cancel();
      if(sdk_){request_capture_stop("cleanup");sdk_->shutdown();}
      ready_=false;
      frame_queue_.close();
      if(frame_worker_.joinable())frame_worker_.join();
      // Drain reliable DDS while the recorder is still alive. Both publishers
      // share one 3s budget; an unsupported RMW or timeout is a source failure.
      publication_acknowledged_=!require_recorder_;
      if(require_recorder_) {
        const auto until=std::chrono::steady_clock::now()+std::chrono::seconds(3);
        auto remaining=[&](){return std::max(std::chrono::nanoseconds::zero(),
          std::chrono::duration_cast<std::chrono::nanoseconds>(until-std::chrono::steady_clock::now()));};
        try {
          raw_acknowledged_=source_->wait_for_all_acked(remaining());
          filtered_acknowledged_=filtered_source_->wait_for_all_acked(remaining());
          publication_acknowledged_=raw_acknowledged_ && filtered_acknowledged_;
        }catch(const std::exception& error){
          std::cerr<<"xt_source_node: publication ACK failure: "<<error.what()<<std::endl;
        }
        if(!publication_acknowledged_)latch_pipeline_failure(PipelineFailure::Acknowledgement);
      }
      sdk_queue_final_=sdk_?sdk_->getQueueDiagnostics():XinTan::WcQueueDiagnostics{};
      const auto pending=frame_queue_.snapshot();
      if(require_recorder_ && (pending.rejected || pending.discarded || pending.pending_frames ||
          sdk_queue_final_.raw_dropped || sdk_queue_final_.image_dropped ||
          sdk_queue_final_.raw_pending_frames || sdk_queue_final_.image_pending_frames))
        latch_pipeline_failure(PipelineFailure::UndrainedOrDropped);
      // Keep the SDK/logtag referenced by the copied Frame objects alive until
      // the queue is drained. Deleting the SDK never precedes worker join.
      sdk_.reset();
      source_journal_.flush();
      if(!source_journal_)throw std::runtime_error("source journal flush failed");
      source_journal_.close();
      sync_file(session_dir_/"source_frames.jsonl");
      const bool synchronized=!failed_ && !pending.pending_frames &&
        journal_frames_==published_.load() && pending.accepted==worker_processed_ &&
        pending.accepted==journal_frames_;
      std::ostringstream report;
      report<<"{\"schema_version\":2,\"side\":\""<<side_<<"\",\"sensor_id\":\""<<serial_
        <<"\",\"published_frames\":"<<published_.load()<<",\"journal_frames\":"<<journal_frames_
        <<",\"synchronized\":"<<(synchronized?"true":"false")<<",\"closed_normally\":"<<(failed_?"false":"true")
        <<",\"sdk_malformed_frames_rejected\":"<<malformed_frames_.load()
        <<",\"pipeline\":{\"callback_received\":"<<pending.received<<",\"accepted_frames\":"<<pending.accepted
        <<",\"rejected_frames\":"<<pending.rejected<<",\"rejected_bytes\":"<<pending.rejected_bytes
        <<",\"queue_capacity_frames\":"<<pending.capacity_frames<<",\"queue_capacity_bytes\":"<<pending.capacity_bytes
        <<",\"queue_highwater_frames\":"<<pending.highwater_frames<<",\"queue_highwater_bytes\":"<<pending.highwater_bytes
        <<",\"pending_frames\":"<<pending.pending_frames<<",\"pending_bytes\":"<<pending.pending_bytes
        <<",\"worker_processed\":"<<worker_processed_.load()<<",\"worker_journaled\":"<<journal_frames_
        <<",\"worker_published\":"<<published_.load()
        <<",\"publication_acknowledged\":"<<(publication_acknowledged_?"true":"false")
        <<",\"raw_publication_acknowledged\":"<<(raw_acknowledged_?"true":"false")
        <<",\"filtered_publication_acknowledged\":"<<(filtered_acknowledged_?"true":"false")
        <<",\"failure_code\":\""<<pipeline_failure_name()
        <<"\",\"shutdown_deadline_exceeded\":false},\"sdk_queues\":"<<sdk_queue_json(sdk_queue_final_)<<"}\n";
      write_record("source_summary.json",report.str());sync_file(session_dir_/"source_summary.json");
      std::cerr<<"xt_source_node: SDK cleanup completed"<<std::endl;
    }catch(const std::exception& error){
      latch_pipeline_failure(PipelineFailure::Cleanup);
      // In particular, never destruct a joinable live thread on an error path.
      frame_queue_.close();if(frame_worker_.joinable())frame_worker_.join();
      std::cerr<<"xt_source_node SDK cleanup: "<<error.what()<<std::endl;
    }
  }
  bool finished() const {return finished_.load();}
  bool failed() const {return failed_.load();}
 private:
  enum class PipelineFailure {None,Callback,QueueFull,Worker,Malformed,UndrainedOrDropped,Cleanup,Acknowledgement};
  void latch_pipeline_failure(PipelineFailure code) noexcept {
    auto expected=PipelineFailure::None;pipeline_failure_.compare_exchange_strong(expected,code);
    ready_=false;failed_=true;paused_=true;
  }
  const char* pipeline_failure_name() const noexcept {
    switch(pipeline_failure_.load()) {
      case PipelineFailure::None:return "";case PipelineFailure::Callback:return "CALLBACK_FRAME_INVALID";
      case PipelineFailure::QueueFull:return "SOURCE_QUEUE_CAPACITY_EXCEEDED";
      case PipelineFailure::Worker:return "SOURCE_WORKER_FAILED";
      case PipelineFailure::Malformed:return "SDK_FRAME_MALFORMED";
      case PipelineFailure::UndrainedOrDropped:return "SOURCE_OR_SDK_DROPPED_OR_PENDING";
      case PipelineFailure::Cleanup:return "SOURCE_CLEANUP_FAILED";
      case PipelineFailure::Acknowledgement:return "SOURCE_PUBLICATION_ACK_UNCONFIRMED";
    }return "UNKNOWN";
  }
  static void sync_file(const fs::path& path) {
    const int fd=::open(path.c_str(),O_RDONLY|O_CLOEXEC);
    if(fd<0)throw std::runtime_error("cannot reopen artifact for sync");
    const int status=::fsync(fd);::close(fd);
    if(status)throw std::runtime_error("artifact fsync failed");
  }
  static std::string sdk_queue_json(const XinTan::WcQueueDiagnostics& q) {
    std::ostringstream out;
    out<<"{\"raw_received\":"<<q.raw_received<<",\"raw_dropped\":"<<q.raw_dropped
      <<",\"raw_highwater_frames\":"<<q.raw_highwater_frames<<",\"raw_highwater_bytes\":"<<q.raw_highwater_bytes
      <<",\"raw_pending_frames\":"<<q.raw_pending_frames<<",\"raw_pending_bytes\":"<<q.raw_pending_bytes
      <<",\"image_received\":"<<q.image_received<<",\"image_dropped\":"<<q.image_dropped
      <<",\"image_highwater_frames\":"<<q.image_highwater_frames<<",\"image_highwater_bytes\":"<<q.image_highwater_bytes
      <<",\"image_pending_frames\":"<<q.image_pending_frames<<",\"image_pending_bytes\":"<<q.image_pending_bytes
      <<",\"queue_capacity_frames\":"<<q.queue_capacity_frames<<",\"queue_capacity_bytes\":"<<q.queue_capacity_bytes
      <<",\"strict_delivery\":"<<(q.strict_delivery?"true":"false")<<"}";return out.str();
  }
  void frame_worker_loop() noexcept {
    FrameQueue::Entry entry;
    while(frame_queue_.take(entry)) {
      ++worker_processed_;
      try{process_frame(entry.value);}catch(const std::exception& error){
        latch_pipeline_failure(PipelineFailure::Worker);
        RCLCPP_ERROR(get_logger(),"source worker: %s",error.what());
      }catch(...){latch_pipeline_failure(PipelineFailure::Worker);}
      entry.value.reset();frame_queue_.complete(entry.bytes);
    }
  }
  template<class T>T parameter(const char* name,const T& value) {
    rcl_interfaces::msg::ParameterDescriptor d;d.read_only=true;
    return declare_parameter<T>(name,value,d);
  }
  void on_event(const std::shared_ptr<XinTan::CBEventData>& event) {
    if(event->eventstr=="sdkState" && event->cmdid==0xfe) connected_event_=true;
    if(event->eventstr=="wc_frame_rejected"){
      ++malformed_frames_;
      if(require_recorder_)latch_pipeline_failure(PipelineFailure::Malformed);
    }
  }
  bool request_capture_stop(const char* phase) {
    const auto result=wc_xt_driver::request_owned_stop(streaming_,failed_,[this](){return sdk_ && sdk_->stop();});
    if(!result.acknowledged)
      std::cerr<<"xt_source_node "<<phase<<": "<<result.error<<"; own stream stop remains unconfirmed"<<std::endl;
    return result.acknowledged;
  }
  void fail(const std::string& reason) {
    if(failed_.exchange(true))return;
    ready_=false;paused_=true;
    fault_stop_attempted_=true;request_capture_stop("validation failure");
    RCLCPP_ERROR(get_logger(),"%s",reason.c_str());
    diagnostic(reason,2);
  }
  void tick() {
    if(wc_xt_process::stop_requested()){finished_=true;return;}
    if(sdk_ && require_recorder_) {
      const auto q=sdk_->getQueueDiagnostics();
      if(q.raw_dropped || q.image_dropped)latch_pipeline_failure(PipelineFailure::UndrainedOrDropped);
    }
    const double elapsed=(steady_ns()-started_at_)/1e9;
    if(wc_xt_driver::acquisition_runtime_expired(elapsed,max_runtime_seconds_)){
      finished_=true;
      diagnostic("bounded acquisition completed; main process will clean up SDK",0);return;
    }
    if(failed_||paused_){
      // One fault-triggered attempt; do not hammer an unavailable device every
      // timer tick. Explicit service retries and final cleanup remain possible.
      if(!fault_stop_attempted_){fault_stop_attempted_=true;request_capture_stop("asynchronous source failure");}
      if(++tick_count_%5==0)diagnostic("source paused after validation failure; inspect log and source flags",2);
      return;
    }
    if(connected_event_.load() && (!require_recorder_ ||
        (source_->get_subscription_count()>0 && filtered_source_->get_subscription_count()>0))) {
      connected_event_=false;
      if(initialized_){fail("SDK reconnect detected; explicit restart required, no automatic map resume");return;}
      read_identity_and_start();
    }
    if(!initialized_ && elapsed>deadline_seconds_)fail("device connection/readback timeout");
    if(initialized_&&!sdk_->isconnect())fail("device disconnected; mapping must remain paused");
    if(streaming_ && last_frame_ns_ && wc_xt_driver::acquisition_source_silence_expired(
        (steady_ns()-last_frame_ns_)/1e9,source_stale_seconds_))
      fail("no point frame within configured source silence timeout");
    if(++tick_count_%5==0)diagnostic(read_only_?"read-only identity/config probe":"arrival_only: calibration and common-time validation required",1);
  }
  void write_record(const std::string& filename,const std::string& text) {
    std::ofstream out(session_dir_/filename,std::ios::binary|std::ios::out|std::ios::trunc);
    out<<text;out.flush();out.close();
    if(!out)throw std::runtime_error("cannot persist "+filename);
  }
  void persist_identity(const char* name,const XinTan::RespDevInfo& info) {
    std::ostringstream out;
    out<<"expected_serial="<<serial_<<"\nobserved_serial="<<info.sn<<"\nfirmware="<<info.fwVersion
       <<"\nexpected_device_ip="<<device_ip_<<"\nobserved_device_ip="<<ipv4(info.ip)
       <<"\nexpected_receive_ip="<<receive_ip_<<"\nobserved_receive_ip="<<ipv4(info.udpDestIp)
       <<"\nexpected_receive_port="<<port_<<"\nobserved_receive_port="<<info.udpDestPort
       <<"\nsdk_info_time_sync_type="<<unsigned(info.timeSyncType)
       <<"\nsdk_info_time_sync_type_validity=SDK_DECODED_NOT_INDEPENDENTLY_PROVEN\n";
    write_record(name,out.str());
  }
  wc_xt_driver::DeviceConfigSnapshot persist_readback(const char* name,const XinTan::RespDevInfo& info,const XinTan::RespDevConfig& config) {
    std::ostringstream out;
    out<<"serial="<<info.sn<<"\nfirmware="<<info.fwVersion<<"\nhdr_readback="<<unsigned(config.hdrMode)
       <<"\ndevice_config_policy="<<wc_xt_driver::device_config_policy_name(device_policy_)
       <<"\nsdk_config_version="<<unsigned(config.version)
       <<"\nobserved_device_ip="<<ipv4(info.ip)<<"\nobserved_receive_ip="<<ipv4(info.udpDestIp)<<"\nobserved_receive_port="<<info.udpDestPort
       <<"\nsdk_info_time_sync_type="<<unsigned(info.timeSyncType)
       <<"\nmaxfps="<<unsigned(config.maxfps)<<"\nsetmaxfps="<<unsigned(config.setmaxfps)
       <<"\nmin_amplitude="<<config.miniAmp<<"\nintgs="<<config.integrationTimeGs
       <<"\nhbinning="<<unsigned(config.bBinningH)<<"\nvbinning="<<unsigned(config.bBinningV)<<"\n";
    for(int n=0;n<5;++n)out<<"int"<<n+1<<"="<<config.integrationTimes[n]<<"\nfreq_wire"<<n+1<<"="<<unsigned(config.freq[n])<<"\n";
    XinTan::XByteArray raw;
    if(!sdk_->customCmd(5,{},raw)) {
      out<<"raw_response_status=FAILED\n";write_record(name,out.str());
      throw std::runtime_error("raw configuration readback failed; decoded snapshot retained");
    }
    const auto snapshot=wc_xt_driver::device_config_snapshot(raw);
    out<<"raw_response_status=RECEIVED\nraw_response_bytes="<<raw.size()
       <<"\nconfig_response_hex="<<wc_xt_driver::config_bytes_hex(raw)<<"\n"
       <<wc_xt_driver::describe_device_config_snapshot(snapshot)
       <<"decoded_sdk_fields_and_raw_response_are_separate_read_requests=true\n";
    write_record(name,out.str());return snapshot;
  }
  void apply_imaging(const XinTan::RespDevConfig& before) {
    std::ofstream log(session_dir_/"configuration_commands.txt",std::ios::out|std::ios::trunc);
    if(!log)throw std::runtime_error("cannot open configuration command log");
    log<<"device_config_policy="<<wc_xt_driver::device_config_policy_name(device_policy_)<<"\n";
    auto run=[&](const char* name,bool needed,auto operation) {
      log<<name<<"="<<(needed?"REQUESTED":"READBACK_MATCH_NO_WRITE")<<"\n";log.flush();
      if(!log)throw std::runtime_error("cannot log imaging request");
      if(needed){const bool ok=operation();log<<name<<"="<<(ok?"ACK":"FAILED")<<"\n";log.flush();
        if(!ok)throw std::runtime_error(std::string("imaging setter not acknowledged: ")+name);}
    };
    const auto& c=requested_;
    run("HDR_cmd10",int(before.hdrMode)!=c.integer("Setting.HDR"),[&](){return sdk_->setHdrMode(static_cast<XinTan::HDRMode>(c.integer("Setting.HDR")));});
    bool int_changed=before.integrationTimeGs!=c.integer("Setting.intgs"),freq_changed=false;
    for(int n=0;n<5;++n){int_changed|=before.integrationTimes[n]!=c.integer("Setting.int"+std::to_string(n+1));freq_changed|=before.freq[n]!=c.frequency_wire(n+1);}
    run("exposures_cmd8",int_changed,[&](){return sdk_->setIntTimesus(c.integer("Setting.intgs"),c.integer("Setting.int1"),c.integer("Setting.int2"),c.integer("Setting.int3"),c.integer("Setting.int4"),c.integer("Setting.int5"));});
    auto f=[&](int n){return static_cast<XinTan::ModulationFreq>(c.frequency(n));};
    run("multi_frequency_cmd27",freq_changed,[&](){return sdk_->setMultiModFreq(f(1),f(2),f(3),f(4),f(5));});
    run("amplitude_cmd9",before.miniAmp!=c.integer("Setting.minLSB"),[&](){return sdk_->setMinAmplitude(c.integer("Setting.minLSB"));});
    run("maxfps_cmd56",before.setmaxfps!=c.integer("Setting.maxfps"),[&](){return sdk_->setMaxFps(c.integer("Setting.maxfps"));});
    run("vertical_binning_cmd12",before.bBinningV!=c.integer("Setting.vbinning"),[&](){return sdk_->setBinningV(c.integer("Setting.vbinning"));});
    log<<"persistent_save=NOT_REQUESTED\nnetwork_clock_firmware_motor=DENIED\n";
  }
  void read_identity_and_start() {
    XinTan::RespDevInfo info{};XinTan::RespDevConfig config{};XinTan::CamParameterS lens{};
    if(!sdk_->getDevInfo(info)){fail("required identity readback failed");return;}
    try {persist_identity("device_identity.txt",info);}catch(const std::exception& error){fail(error.what());return;}
    if(info.sn!=serial_){fail("device serial mismatch: observed "+info.sn);return;}
    if(!sdk_->getDevConfig(config)){fail("required device configuration readback failed; identity retained");return;}
    try {
      const auto before=persist_readback("device_before.txt",info,config);
      if(ipv4(info.ip)!=device_ip_ || ipv4(info.udpDestIp)!=receive_ip_ || info.udpDestPort!=port_)
        throw std::runtime_error("device IP/UDP readback does not match requested endpoint; actual identity/config archived; no settings changed");
      if(!sdk_->getLensCalidata(lens)||!std::isfinite(lens.fx)||!std::isfinite(lens.fy)||lens.fx<=0||lens.fy<=0)
        throw std::runtime_error("required lens readback unavailable or invalid");
      if(!read_only_ && !before.known_fields_available)
        throw std::runtime_error("device configuration format unavailable for protected readback; raw evidence retained; no imaging writes/start");
      wc_xt_driver::apply_device_config_policy(device_policy_,read_only_,
        [this,&config](){apply_imaging(config);},[this](){requested_.apply_host(*sdk_);});
      if(read_only_ || device_policy_==wc_xt_driver::DeviceConfigPolicy::PreserveCurrent) {
        write_record("configuration_commands.txt",std::string("device_config_policy=")+wc_xt_driver::device_config_policy_name(device_policy_)
          +"\nimaging_setters=NOT_CALLED\nhost_filters="+(read_only_?"NOT_APPLIED_PROBE":"APPLIED")+"\npersistent_save=NOT_REQUESTED\nnetwork_clock_firmware_motor=DENIED\n");
      }
      if(!read_only_ && !sdk_->getDevConfig(config))throw std::runtime_error("post-policy device config read failed");
      const auto after=persist_readback("device_after.txt",info,config);
      const auto baseline=device_policy_==wc_xt_driver::DeviceConfigPolicy::PreserveCurrent ? before : after;
      const auto pre_start_changes=wc_xt_driver::device_config_changes(baseline,after);
      if(!read_only_ && !pre_start_changes.empty()) {
        protection_status_="FAILED_BEFORE_START";
        std::ostringstream changed;changed<<"status="<<protection_status_<<"\n";
        for(const auto& field:pre_start_changes)changed<<"changed_field="<<field<<"\n";
        write_record("device_protection.txt",changed.str());
        throw std::runtime_error("known device configuration changed before start; no stream started");
      }
      if(!read_only_) {
        if(wc_xt_process::stop_requested()){finished_=true;return;}
        streaming_=true; // Lost ACK may still mean the device began streaming.
        if(!sdk_->start(XinTan::IMG_POINTCLOUDAMP))throw std::runtime_error("SDK start stream was not acknowledged");
        // Keep ready_ false until the post-start readback is checked. No frame
        // from a changed/unconfirmed device configuration is labelled retained.
        protection_status_="POST_START_READBACK_PENDING";
        XinTan::RespDevInfo post_info{};XinTan::RespDevConfig post_config{};
        if(!sdk_->getDevInfo(post_info))throw std::runtime_error("post-start identity readback failed");
        persist_identity("device_identity_after_start.txt",post_info);
        if(post_info.sn!=serial_)throw std::runtime_error("post-start serial mismatch");
        if(!sdk_->getDevConfig(post_config))throw std::runtime_error("post-start configuration readback failed");
        const auto post=persist_readback("device_after_start.txt",post_info,post_config);
        auto changes=wc_xt_driver::device_config_changes(baseline,post);
        if(ipv4(post_info.ip)!=device_ip_ || ipv4(post_info.udpDestIp)!=receive_ip_ || post_info.udpDestPort!=port_)
          changes.push_back("device_endpoint");
        protection_status_=changes.empty()?"KNOWN_FIELDS_UNCHANGED_AFTER_START":"FAILED_AFTER_START";
        observed_channel_=post.channel_valid?std::to_string(post.channel):"UNAVAILABLE";
        std::ostringstream checked;checked<<"device_config_policy="<<wc_xt_driver::device_config_policy_name(device_policy_)
          <<"\nstatus="<<protection_status_<<"\n"<<wc_xt_driver::describe_device_config_snapshot(post);
        for(const auto& field:changes)checked<<"changed_field="<<field<<"\n";
        write_record("device_protection.txt",checked.str());
        if(!changes.empty())throw std::runtime_error("known device configuration changed after stream start; capture refused, inspect device_protection.txt");
        config=post_config;info=post_info;
      } else {
        protection_status_="NOT_RUN_PROBE";
        observed_channel_=before.channel_valid?std::to_string(before.channel):"UNAVAILABLE";
      }
      std::ifstream after_file(session_dir_/"device_after.txt",std::ios::binary);
      std::ostringstream after_text;after_text<<after_file.rdbuf();
      std::ostringstream effective;
      effective<<"actual_device_readback_sha256="<<sha256(after_text.str())<<"\n";
      if(!read_only_) {
        std::ifstream post_file(session_dir_/"device_after_start.txt",std::ios::binary);
        std::ostringstream post_text;post_text<<post_file.rdbuf();
        if(!post_file)throw std::runtime_error("cannot hash archived post-start readback");
        effective<<"post_start_device_readback_sha256="<<sha256(post_text.str())<<"\n";
      }
      effective<<"sdk_commit="<<WC_XT_SDK_COMMIT<<"\nsdk_provenance_mode="<<WC_XT_SDK_PROVENANCE_MODE
        <<"\nvendor_patch_sha256="<<WC_XT_PATCH_SHA<<"\nvendor_manifest_sha256="<<WC_XT_MANIFEST_SHA
        <<"\ndriver_sha256="<<WC_XT_DRIVER_SHA<<"\nfilter_origin_commit=3d3db067ae9bdc0528202c3087bc10fd3b706638"
        <<"\nfilter_manifest_sha256="<<WC_XT_FILTER_MANIFEST_SHA
        <<"\nserial="<<info.sn<<"\nfirmware="<<info.fwVersion<<"\nrequested_xtcfg_sha256="<<requested_hash_
        <<"\ndevice_ip="<<device_ip_<<"\nreceive_ip="<<receive_ip_<<"\nreceive_port="<<port_
        <<"\nSDK_filter_flags="<<sdk_->getSdkFilterFlags()<<"\ncoordinate_convention=FLU\ncloud_coord=CAR"
        <<"\ncoordinate_contract=xt_sdk_car_fru_to_flu_yneg_v1\nsdk_native_coordinate_convention=FRU"
        <<"\ncoordinate_conversion_matrix=1,0,0,0,-1,0,0,0,1\ncoordinate_conversion_kind=representation_reflection_not_extrinsic"
        <<"\nraw_branch=before_host_filter_and_host_geometry_cuts\nfiltered_branch=SDK_host_filters_and_geometry_cuts"
        <<"\nconfiguration_status="<<(read_only_?"READ_ONLY_PROBE":"PARTIAL_XTCFG")
        <<"\ndevice_config_policy="<<wc_xt_driver::device_config_policy_name(device_policy_)
        <<"\ndevice_config_protection="<<protection_status_<<"\nobserved_freq_channel="<<observed_channel_
        <<"\ndevice_setting_disposition="<<(read_only_?"REQUESTED_NOT_APPLIED_PROBE":device_policy_==wc_xt_driver::DeviceConfigPolicy::PreserveCurrent?"PRESERVED_CURRENT_NOT_OLD_XTCFG":"XTCFG_DIFF_SETTERS")
        <<"\npost_start_readback_file="<<(read_only_?"NOT_REQUESTED":"device_after_start.txt")
        <<"\nGUI_PointCloudViewer_parity=UNPROVEN"
        <<"\nfilter_timedf_kalman_ms=2000\nfilter_timedf_dust_ms=300\nfilter_timedf_is_not_measured_latency=true\n";
      for(const auto& item:requested_.values) {
        const auto& k=item.first;
        std::string disposition=wc_xt_driver::xtcfg_disposition(device_policy_,read_only_,k);
        if(!read_only_ && device_policy_==wc_xt_driver::DeviceConfigPolicy::ApplyXtConfig && k=="Setting.hbinning")disposition=config.bBinningH==requested_.integer(k)?"READBACK_MATCH":"UNSUPPORTED_HORIZONTAL_BINNING_SETTER";
        if(!read_only_ && k.rfind("Filters.average",0)==0 && !requested_.integer("Filters.averageEnable"))disposition="DISABLED_AS_REQUESTED";
        effective<<k<<"="<<item.second<<";disposition="<<disposition<<"\n";
      }
      source_hash_=sha256(effective.str()+"representation=before_host_filter\n");
      filtered_hash_=sha256(effective.str()+"representation=host_filtered\n");
      write_record("device_readback.txt",effective.str()+"effective_source_hash="+source_hash_+"\nfiltered_source_hash="+filtered_hash_+"\n");
      if(!read_only_)RCLCPP_WARN(get_logger(),"device_config_policy=%s; host filters including spatial applied; PARTIAL_XTCFG: Windows-only pclFilterOn processor has no verified Linux equivalent",wc_xt_driver::device_config_policy_name(device_policy_));
    } catch(const std::exception& error) {fail(error.what());return;}
    initialized_=true;
    if(read_only_){diagnostic("identity/config/lens readback completed; no acquisition requested",0);return;}
    if(wc_xt_process::stop_requested()){finished_=true;return;}
    ready_=true;last_frame_ns_=steady_ns();
    diagnostic("stream active with arrival-only time; formal mapping remains gated",1);
  }
  void on_frame(const std::shared_ptr<XinTan::Frame>& frame) {
    if(!ready_ || !frame || !frame->hasPointcloud)return;
    const uint64_t callback_ns=steady_ns();
    const auto& raw=frame->wc_before_host_filter;
    if(!frame->wc_receive_system_ns || !frame->wc_receive_steady_ns || !raw ||
       !raw->hasPointcloud || frame->points.empty() || frame->points.size()>76800 ||
       raw->points.size()>76800 || frame->points.size()!=size_t(frame->width)*frame->height ||
       raw->points.empty() || raw->points.size()!=size_t(raw->width)*raw->height) {
      frame_queue_.reject(0);latch_pipeline_failure(PipelineFailure::Callback);return;
    }
    const auto bytes=XinTan::wc_frame_storage_bytes(*frame);
    if(!frame_queue_.try_copy(bytes,[&](){
      auto copy=XinTan::wc_copy_frame_pair(frame);copy->wc_driver_callback_steady_ns=callback_ns;return copy;
    })) {
      latch_pipeline_failure(PipelineFailure::QueueFull);return;
    }
    last_frame_ns_=frame->wc_receive_steady_ns;
  }
  void process_frame(const std::shared_ptr<XinTan::Frame>& frame) {
    // Queued samples remain eligible after stop was requested: these were
    // accepted while the verified stream was active and must drain in order.
    const uint64_t receive=frame->wc_receive_system_ns,mono=frame->wc_receive_steady_ns;
    const uint64_t callback_mono=frame->wc_driver_callback_steady_ns;
    const uint64_t worker_mono=steady_ns();
    if(have_last_time_ && frame->frame_id==last_sdk_frame_id_ && frame->timeStampS==last_sec_ && frame->timeStampNS==last_nsec_){++duplicates_;throw std::runtime_error("duplicate SDK sample");}
    FrameMsg message;
    message.session_id=session_id_;message.side=side_;message.sensor_id=serial_;
    message.stream_epoch=epoch_;message.frame_sequence=sequence_++;
    message.source_config_hash=source_hash_;message.coordinate_convention="FLU";message.units="m";
    message.host_receive_time.sec=static_cast<int32_t>(receive/1'000'000'000ULL);
    message.host_receive_time.nanosec=receive%1'000'000'000ULL;
    message.host_monotonic_ns=mono;
    message.header.stamp=message.host_receive_time;message.header.frame_id=frame_id_;
    message.common_time_valid=false;message.common_time_ns=static_cast<int64_t>(receive);
    message.time_source="arrival_only";message.clock_model_id="";message.uncertainty_valid=false;
    message.device_time_components_valid=frame->timeStampNS<1'000'000'000;
    message.device_timestamp_seconds=frame->timeStampS;message.device_timestamp_nanoseconds=frame->timeStampNS;
    message.device_time_type="sdk_type_"+std::to_string(frame->timeStampType);
    message.device_sync_state="sdk_state_"+std::to_string(frame->timeStampState);
    message.device_timestamp_unit="ns";
    if(message.device_time_components_valid && frame->timeStampS <= (std::numeric_limits<uint64_t>::max()-frame->timeStampNS)/1'000'000'000ULL){
      message.device_timestamp_valid=true;message.device_timestamp_raw=frame->timeStampS*1'000'000'000ULL+frame->timeStampNS;
    }
    if(frame->frame_version==3){
      message.device_timestamp_valid=true;message.device_timestamp_raw=frame->info.timestamp[0];message.device_timestamp_unit="sdk_v3_ms";
      const std::string frame_serial(reinterpret_cast<const char*>(frame->info.sn),strnlen(reinterpret_cast<const char*>(frame->info.sn),sizeof(frame->info.sn)));
      if(frame_serial!=serial_)throw std::runtime_error("frame-embedded serial mismatch");
    }
    message.diagnostic_flags={"host_receive_before_sdk_host_filter","time_model_unvalidated","physical_axes_require_validation","physical_scale_requires_validation","sdk_cloud_coord=ClOUDCOORD_CAR_1","coordinate_contract=xt_sdk_car_fru_to_flu_yneg_v1","sdk_native_coordinate_convention=FRU","coordinate_conversion_kind=representation_reflection_not_extrinsic","coordinate_evidence=v7_extrinsics_20261005_axes01_axes02","representation=before_host_filter","intensity_is_raw_amplitude"};
    message.diagnostic_flags.push_back("sdk_frame_id="+std::to_string(frame->frame_id));
    if(!message.device_time_components_valid)message.diagnostic_flags.push_back("invalid_device_nanoseconds");
    if(have_last_time_ && (frame->timeStampS<last_sec_ || (frame->timeStampS==last_sec_ && frame->timeStampNS<last_nsec_))){
      // Fail this sample without publishing a fabricated new epoch into the
      // current source session. Restart requires a new explicit source epoch.
      throw std::runtime_error("device clock moved backward; explicit restart required");
    }
    last_sec_=frame->timeStampS;last_nsec_=frame->timeStampNS;last_sdk_frame_id_=frame->frame_id;have_last_time_=true;
    message.diagnostic_flags.push_back("configuration_status=PARTIAL_XTCFG");
    message.diagnostic_flags.push_back(std::string("device_config_policy=")+wc_xt_driver::device_config_policy_name(device_policy_));
    message.diagnostic_flags.push_back("device_config_protection="+protection_status_);
    message.diagnostic_flags.push_back("observed_freq_channel="+observed_channel_);
    message.diagnostic_flags.push_back(std::string("device_setting_disposition=")+(device_policy_==wc_xt_driver::DeviceConfigPolicy::PreserveCurrent?"PRESERVED_CURRENT_NOT_OLD_XTCFG":"XTCFG_DIFF_SETTERS"));
    message.diagnostic_flags.push_back("unsupported=Setting.pclFilterOn:Windows_pointcloud_processor");
    message.diagnostic_flags.push_back("host_filter_and_dispatch_elapsed_ns="+std::to_string(callback_mono-mono));
    message.diagnostic_flags.push_back("host_elapsed_is_not_measurement_latency");
    message.diagnostic_flags.push_back("driver_queue_elapsed_ns="+std::to_string(worker_mono-callback_mono));
    const auto fill=[&](FrameMsg& output,const std::shared_ptr<XinTan::Frame>& input) {
      // cartesianTransform.cpp traverses row p, then column q and stores each
      // point at p*width+q, including invalid NaN slots. Preserve that actual
      // layout as provenance while retaining the existing flat ROS layout.
      auto& layout_flags=output.diagnostic_flags;
      layout_flags.erase(std::remove_if(layout_flags.begin(),layout_flags.end(),[](const std::string& flag){
        return flag.rfind("sdk_width=",0)==0 || flag.rfind("sdk_height=",0)==0 || flag.rfind("sdk_point_order=",0)==0;
      }),layout_flags.end());
      if(input->width>0 && input->height>0 && input->points.size()==size_t(input->width)*input->height){
        layout_flags.push_back("sdk_width="+std::to_string(input->width));
        layout_flags.push_back("sdk_height="+std::to_string(input->height));
        layout_flags.push_back("sdk_point_order=row_major");
      }
      auto& cloud=output.cloud;cloud.header=output.header;cloud.height=1;cloud.width=input->points.size();
      cloud.is_bigendian=false;cloud.is_dense=false;cloud.point_step=16;cloud.row_step=16*cloud.width;
      for(size_t n=0;n<4;++n){sensor_msgs::msg::PointField f;f.name=std::array<std::string,4>{"x","y","z","intensity"}[n];f.offset=4*n;f.datatype=7;f.count=1;cloud.fields.push_back(f);}
      cloud.data.resize(cloud.row_step);output.raw_count=cloud.width;output.valid_count=0;
      for(size_t n=0;n<input->points.size();++n){
        const auto& p=input->points[n];
        const float amplitude=n<input->amplData.size()?static_cast<float>(input->amplData[n]):std::numeric_limits<float>::quiet_NaN();
        // SDK CAR is FRU for these XT-M60 streams (axes01/axes02 physical evidence).
        // Correct handedness at the representation boundary, for raw and filtered alike.
        // This det=-1 conversion is NOT an installation rotation or an extrinsic.
        const float values[4]={p.x,-p.y,p.z,amplitude};
        std::memcpy(cloud.data.data()+16*n,values,16);
        if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z))++output.valid_count;
      }
    };
    fill(message,frame->wc_before_host_filter);
    FrameMsg filtered=message;filtered.cloud=sensor_msgs::msg::PointCloud2{};
    filtered.source_config_hash=filtered_hash_;
    auto& flags=filtered.diagnostic_flags;
    for(auto& flag:flags){
      if(flag=="representation=before_host_filter")flag="representation=host_filtered";
      if(flag=="intensity_is_raw_amplitude")flag="intensity_is_sdk_processed_amplitude";
    }
    flags.push_back("raw_source_config_hash="+source_hash_);
    flags.push_back("before_host_filter_cloud_sha256="+sha256(std::string(reinterpret_cast<const char*>(message.cloud.data.data()),message.cloud.data.size())));
    fill(filtered,frame);
    // Retain every structurally valid sample, including filter warmup/all-NaN
    // frames. Availability readiness and mapping usefulness are separate gates.
    source_journal_<<"{\"sensor_id\":\""<<message.sensor_id<<"\",\"stream_epoch\":\""<<message.stream_epoch
      <<"\",\"sequence\":"<<message.frame_sequence<<",\"host_receive_ns\":"<<receive
      <<",\"host_monotonic_ns\":"<<mono
      <<",\"raw_valid_count\":"<<message.valid_count<<",\"filtered_valid_count\":"<<filtered.valid_count
      <<",\"raw_sha256\":\""
      <<sha256(std::string(reinterpret_cast<const char*>(message.cloud.data.data()),message.cloud.data.size()))
      <<"\",\"filtered_sha256\":\""
      <<sha256(std::string(reinterpret_cast<const char*>(filtered.cloud.data.data()),filtered.cloud.data.size()))<<"\"}\n";
    if(!source_journal_)throw std::runtime_error("source frame journal write failed");
    ++journal_frames_;
    source_->publish(message);filtered_source_->publish(filtered);
    ++published_;
    if(!first_valid_written_ && message.valid_count>0 && filtered.valid_count>0) {
      source_journal_.flush();
      if(!source_journal_)throw std::runtime_error("first valid source journal flush failed");
      std::ostringstream marker;
      marker<<"{\"session_id\":\""<<session_id_<<"\",\"sensor_id\":\""<<serial_
        <<"\",\"side\":\""<<side_<<"\",\"stream_epoch\":\""<<epoch_
        <<"\",\"first_valid_monotonic_ns\":"<<mono<<",\"ready\":true}\n";
      write_record("source_ready.json.tmp",marker.str());
      fs::rename(session_dir_/"source_ready.json.tmp",session_dir_/"source_ready.json");
      first_valid_written_=true;
    }
    if(mirror_){cloud_->publish(message.cloud);filtered_cloud_->publish(filtered.cloud);}
  }

  void diagnostic(const std::string& text,int level) {
    diagnostic_msgs::msg::DiagnosticArray array;array.header.stamp=now();
    diagnostic_msgs::msg::DiagnosticStatus state;state.level=level;state.name=get_fully_qualified_name();state.hardware_id=serial_;state.message=text;
    auto add=[&](const std::string& k,const std::string& v){diagnostic_msgs::msg::KeyValue kv;kv.key=k;kv.value=v;state.values.push_back(kv);};
    add("configuration_status",read_only_?"READ_ONLY_PROBE":"PARTIAL_XTCFG");
    add("device_config_policy",wc_xt_driver::device_config_policy_name(device_policy_));
    add("device_config_protection",protection_status_);add("observed_freq_channel",observed_channel_);
    add("device_setting_disposition",read_only_?"REQUESTED_NOT_APPLIED_PROBE":device_policy_==wc_xt_driver::DeviceConfigPolicy::PreserveCurrent?"PRESERVED_CURRENT_NOT_OLD_XTCFG":"XTCFG_DIFF_SETTERS");
    add("unsupported","Setting.pclFilterOn:Windows_pointcloud_processor");
    add("side",side_);add("device_ip",device_ip_);add("receive_ip",receive_ip_);add("receive_port",std::to_string(port_));
    add("published_frames",std::to_string(published_.load()));add("time_source","arrival_only");
    add("duplicate_frames_rejected",std::to_string(duplicates_.load()));
    add("own_stream_stop_pending",streaming_.load()?"true":"false");
    add("driver_failure_latched",failed_.load()?"true":"false");
    add("sdk_malformed_frames_rejected",std::to_string(malformed_frames_.load()));
    add("stream_epoch",epoch_); // immutable session epoch; no I/O mutex
    const auto q=frame_queue_.snapshot();
    add("source_queue_pending_frames",std::to_string(q.pending_frames));
    add("source_queue_highwater_frames",std::to_string(q.highwater_frames));
    add("source_queue_highwater_bytes",std::to_string(q.highwater_bytes));
    add("source_queue_rejected_frames",std::to_string(q.rejected));
    add("source_worker_processed",std::to_string(worker_processed_.load()));
    add("source_pipeline_failure",pipeline_failure_name());
    if(sdk_){const auto s=sdk_->getQueueDiagnostics();
      add("sdk_raw_dropped",std::to_string(s.raw_dropped));add("sdk_image_dropped",std::to_string(s.image_dropped));
      add("sdk_raw_highwater_frames",std::to_string(s.raw_highwater_frames));
      add("sdk_image_highwater_frames",std::to_string(s.image_highwater_frames));}
    array.status.push_back(state);diagnostics_->publish(array);
  }
  std::string side_,serial_,device_ip_,receive_ip_,frame_id_,session_id_,run_root_,epoch_,requested_hash_,source_hash_,filtered_hash_;
  wc_xt_driver::XtConfig requested_;
  wc_xt_driver::DeviceConfigPolicy device_policy_=wc_xt_driver::DeviceConfigPolicy::PreserveCurrent;
  std::string protection_status_="NOT_CHECKED",observed_channel_="UNAVAILABLE";
  fs::path session_dir_;
  std::ofstream source_journal_;uint64_t journal_frames_=0;
  int port_=0,deadline_seconds_=20,max_runtime_seconds_=30,source_stale_seconds_=3,tick_count_=0,shutdown_timeout_seconds_=10;
  bool mirror_=false,read_only_=true,initialized_=false,have_last_time_=false,fault_stop_attempted_=false,require_recorder_=false;
  uint64_t started_at_=0,sequence_=0,last_sec_=0,last_sdk_frame_id_=0;uint32_t last_nsec_=0;
  std::atomic<bool> connected_event_{false},ready_{false},streaming_{false},failed_{false},paused_{false};
  std::atomic<bool> finished_{false},cleanup_started_{false};
  std::atomic<uint64_t> last_frame_ns_{0},published_{0},duplicates_{0},malformed_frames_{0},worker_processed_{0};
  using FrameQueue=wc_xt_driver::BoundedWorkQueue<std::shared_ptr<XinTan::Frame>>;
  FrameQueue frame_queue_;std::thread frame_worker_;
  std::atomic<PipelineFailure> pipeline_failure_{PipelineFailure::None};
  XinTan::WcQueueDiagnostics sdk_queue_final_;bool first_valid_written_=false,publication_acknowledged_=false,raw_acknowledged_=false,filtered_acknowledged_=false;
  std::unique_ptr<Lock> identity_lock_,udp_lock_;
  std::unique_ptr<XtSdk> sdk_;
  rclcpp::Publisher<FrameMsg>::SharedPtr source_,filtered_source_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_,filtered_cloud_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_;rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){
 return wc_xt_process::run(argc,argv,[](){return std::make_shared<XtSourceNode>();});
}
#endif // WC_XT_SHUTDOWN_TEST_ONLY
