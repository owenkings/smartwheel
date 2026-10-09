#ifdef NDEBUG
#undef NDEBUG
#endif
#include "wc_frame_storage.h"
#include <cassert>
#include <string>
int main(){
  std::string tag="no_hardware_test";
  auto frame=std::make_shared<XinTan::Frame>(tag,0,7,2,1,0);
  frame->points.resize(2);frame->points[0].x=1;frame->points[1].y=2;
  frame->amplData={3,4};frame->frameData={1,2,3};frame->wc_receive_system_ns=123;
  frame->wc_receive_steady_ns=456;frame->hasPointcloud=true;
  auto raw=std::make_shared<XinTan::Frame>(*frame);raw->points[0].x=10;
  frame->wc_before_host_filter=raw;
  const auto budget=XinTan::wc_frame_storage_bytes(*frame);
  auto copy=XinTan::wc_copy_frame_pair(frame);
  assert(copy.get()!=frame.get() && copy->wc_before_host_filter.get()!=raw.get());
  assert(copy->wc_receive_system_ns==123 && copy->wc_receive_steady_ns==456);
  assert(copy->points[0].x==1 && copy->wc_before_host_filter->points[0].x==10);
  frame->points[0].x=100;raw->points[0].x=200;raw->amplData[0]=99;frame->frameData[0]=9;
  assert(copy->points[0].x==1 && copy->wc_before_host_filter->points[0].x==10);
  assert(copy->wc_before_host_filter->amplData[0]==3 && copy->frameData[0]==1);
  assert(XinTan::wc_frame_storage_bytes(*copy)<=budget);
  assert(!copy->wc_before_host_filter->wc_before_host_filter);
}
