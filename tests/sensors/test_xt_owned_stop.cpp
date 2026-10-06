// Pure control-state regression: no ROS, SDK, socket, serial or device object.
#include "wc_xt_driver/owned_stop.hpp"
#include <iostream>
#include <stdexcept>

static void require(bool ok, const char* reason) {
  if(!ok)throw std::runtime_error(reason);
}
int main() {
  try {
    std::atomic<bool> pending{true},failed{false};int calls=0;
    auto reject=[&](){++calls;return false;};
    auto first=wc_xt_driver::request_owned_stop(pending,failed,reject);
    require(!first.acknowledged&&!first.error.empty()&&pending&&failed,"first NACK must retain owned stream and latch failure");
    auto second=wc_xt_driver::request_owned_stop(pending,failed,reject);
    require(!second.acknowledged&&calls==2&&pending&&failed,"second stop must retry, not falsely succeed");
    auto cleanup=wc_xt_driver::request_owned_stop(pending,failed,[&](){++calls;return true;});
    require(cleanup.acknowledged&&!pending&&failed&&calls==3,"cleanup ACK must confirm stop but preserve historical failure");
    auto again=wc_xt_driver::request_owned_stop(pending,failed,reject);
    require(again.acknowledged&&calls==3&&failed,"confirmed stop is idempotent without clearing failure");
    pending=true;failed=false;
    auto exception=wc_xt_driver::request_owned_stop(pending,failed,[]()->bool{throw std::runtime_error("synthetic transport failure");});
    require(!exception.acknowledged&&pending&&failed&&exception.error.find("synthetic transport failure")!=std::string::npos,"exception must remain diagnosable and retryable");
    auto unknown=wc_xt_driver::request_owned_stop(pending,failed,[]()->bool{throw 7;});
    require(!unknown.acknowledged&&pending&&failed,"unknown exception must not lose ownership");
    wc_xt_driver::request_owned_stop(pending,failed,[](){return true;});
    require(!pending&&failed,"explicit retry cannot erase exception history");
    pending=false;failed=false;calls=0;
    auto probe=wc_xt_driver::request_owned_stop(pending,failed,reject);
    require(probe.acknowledged&&!failed&&calls==0,"read-only probe must issue no stop command");
    pending=true;
    auto normal=wc_xt_driver::request_owned_stop(pending,failed,[](){return true;});
    require(normal.acknowledged&&!pending&&!failed,"ordinary acknowledged stop should succeed");
    std::cout<<"PASS SOFTWARE_ONLY: owned stop NACK, retry, cleanup, exception, history, probe and normal ACK\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
