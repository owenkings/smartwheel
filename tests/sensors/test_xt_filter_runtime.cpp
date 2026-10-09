#include "xtsdk.h"
#include "xtdaemon.h"
#include <cstring>
#include <dlfcn.h>
#include <cmath>
#include "wc_xt_driver/xtcfg.hpp"
#include <fstream>
#include <iostream>
#include <stdexcept>
int main(int argc,char**argv){try{
 if(argc!=2)throw std::runtime_error("config directory required");
 // Check every legacy-TBB import and the two hidden OpenCV calibration imports
 // BEFORE constructing a vendor filter or opening any device/worker.
 for(const char* symbol : {
   "_ZN3tbb10interface58internal9task_base7destroyERNS_4taskE",
   "_ZN3tbb10interface78internal15task_arena_base24internal_max_concurrencyEPKNS0_10task_arenaE",
   "_ZN3tbb8internal36get_initial_auto_partitioner_divisorEv",
   "_ZN3tbb10interface78internal15task_arena_base19internal_initializeEv",
   "_ZTIN3tbb4taskE",
   "_ZNK3tbb8internal27allocate_continuation_proxy8allocateEm",
   "_ZNK3tbb18task_group_context28is_group_execution_cancelledEv",
   "_ZNK3tbb8internal20allocate_child_proxy8allocateEm",
   "_ZN3tbb10interface78internal15task_arena_base21internal_current_slotEv",
   "_ZN2cv23estimateAffinePartial2DERKNS_11_InputArrayES2_RKNS_12_OutputArrayEidmdm",
   "_ZN2cv16estimateAffine2DERKNS_11_InputArrayES2_RKNS_12_OutputArrayEidmdm",
   "_ZNK3tbb10interface78internal15task_arena_base16internal_executeERNS1_13delegate_baseE",
   "_ZN3tbb18task_group_context5resetEv",
   "_ZN3tbb10interface78internal15task_arena_base18internal_terminateEv",
   "_ZN3tbb4task13note_affinityEt",
   "_ZN3tbb18task_group_contextD1Ev",
   "_ZN3tbb18task_group_context4initEv",
   "_ZNK3tbb8internal32allocate_root_with_context_proxy4freeERNS_4taskE",
   "_ZNK3tbb8internal32allocate_root_with_context_proxy8allocateEm"}) {
   if(!dlsym(RTLD_DEFAULT,symbol))throw std::runtime_error(std::string("filter dependency symbol missing before load: ")+symbol);
 }

 for(const auto side:{"left","right"}) {
   std::ifstream f(std::string(argv[1])+"/"+side+"-2026-09-11.xtcfg");std::ostringstream text;text<<f.rdbuf();
   auto cfg=wc_xt_driver::XtConfig::parse(text.str());
   XinTan::XtSdk sdk("./xt_filter_test_logs",side);
   cfg.apply_host(sdk);
   if(sdk.getSdkFilterFlags()!=253)throw std::runtime_error("actual linked filter ABI/flags mismatch");
   if(sdk.isconnect())throw std::runtime_error("software test unexpectedly connected");
   // Real public packing executes before the disconnected transport rejects it.
   // Five distinct slots exercise the repaired std::array, including final slot.
   if(sdk.setMultiModFreq(XinTan::FREQ_12M,XinTan::FREQ_6M,XinTan::FREQ_24M,XinTan::FREQ_3M,XinTan::FREQ_1_5M))throw std::runtime_error("disconnected setter unexpectedly acknowledged");
   sdk.clearAllSdkFilter();if(sdk.getSdkFilterFlags())throw std::runtime_error("filter clear failed");
 }
 {
   XinTan::XtSdk log_owner("./xt_filter_test_logs","raw-pair");
   std::string tag=log_owner.getLogtagName();
   XinTan::XtDaemon daemon(tag); // No startup, worker or port open.
   daemon.needPointcloud=true;
   XinTan::CamParameterS lens{80,80,80,30,0,0,0,0,0};
   daemon.cartesianTransform->maptable(lens);
   daemon.cartesianTransform->pointout_coord=1;
   daemon.cartesianTransform->setcutcorner(90);
   daemon.cartesianTransform->cutMinAmp=70;
   if(!daemon.baseFilter->setMedianFilter(3))throw std::runtime_error("median ABI unavailable");
   auto frame=std::make_shared<XinTan::Frame>(tag,XinTan::Frame::AMPLITUDE,42,160,60,16);
   std::memset(&frame->info,0,sizeof(frame->info));
   frame->frame_version=2;frame->timeStampS=12;frame->timeStampNS=345;
   frame->timeStampState=0;frame->timeStampType=0;
   frame->distData.assign(9600,1000);frame->rawdistData=frame->distData;
   frame->amplData.assign(9600,100);frame->reflectivity.assign(9600,.5f);
   frame->motion_vec.assign(9600,0);
   frame->amplData[0]=0;frame->distData[30*160+80]=5000;
   const auto original_depth=frame->distData;const auto original_amp=frame->amplData;
   daemon.reportImage(frame);
   auto raw=frame->wc_before_host_filter;
   if(frame->distData[30*160+80]==5000)throw std::runtime_error("actual median did not process synthetic depth outlier");
   if(!raw||raw->wc_before_host_filter||raw->frame_id!=42||raw->timeStampNS!=345||raw->timeStampS!=12)throw std::runtime_error("raw pair identity lost");
   if(raw->distData!=original_depth||raw->amplData!=original_amp)throw std::runtime_error("host filtering modified original decoded input");
   if(raw->wc_receive_system_ns!=frame->wc_receive_system_ns||raw->wc_receive_steady_ns!=frame->wc_receive_steady_ns||!frame->wc_receive_system_ns)throw std::runtime_error("pair host timestamp differs");
   if(raw->points.size()!=9600||frame->points.size()!=9600||!std::isfinite(raw->points[0].x)||std::isfinite(frame->points[0].x))throw std::runtime_error("raw geometry bypass did not preserve low-amplitude/corner point");
   auto baseline=std::make_shared<XinTan::Frame>(*raw);baseline->points.clear();
   daemon.cartesianTransform->pcltransCamparm(baseline,true);
   for(size_t i=0;i<9600;++i)if(raw->points[i].x!=baseline->points[i].x||raw->points[i].y!=baseline->points[i].y||raw->points[i].z!=baseline->points[i].z)throw std::runtime_error("raw geometry differs from unfiltered conversion");
 }
 std::cout<<"PASS actual pinned filter ABI/config flags and five-frequency disconnected packing; NO HARDWARE STARTUP\n";
 return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}}
