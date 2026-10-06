// No SDK instance, sockets, threads, sensor/motor commands, or vendor library.
// Production policy callbacks and exact SDK config-layout extraction are used.
#include "wc_xt_driver/device_config_policy.hpp"
#include <algorithm>
#include <cstring>
#include <iostream>
#include <stdexcept>
using namespace wc_xt_driver;
using Bytes=std::vector<uint8_t>;
static void check(bool ok,const char* reason){if(!ok)throw std::runtime_error(reason);}
template<class Call>static void rejected(Call call,const char* reason){bool did=false;try{call();}catch(const std::invalid_argument&){did=true;}check(did,reason);}
static Bytes unhex(const std::string& text){
  check(text.size()%2==0,"even hex");Bytes out;
  for(size_t i=0;i<text.size();i+=2)out.push_back(static_cast<uint8_t>(std::stoul(text.substr(i,2),nullptr,16)));
  return out;
}
static bool changed(const DeviceConfigSnapshot& a,const DeviceConfigSnapshot& b,const std::string& field){
  const auto changes=device_config_changes(a,b);return std::find(changes.begin(),changes.end(),field)!=changes.end();
}
int main(){try{
  const auto preserve=parse_device_config_policy("preserve_current"),apply=parse_device_config_policy("apply_xtcfg");
  for(const std::string invalid:{"","true","preserve","APPLY_XTCFG","apply_xtcfg ","is_use_devconfig"})
    rejected([&](){parse_device_config_policy(invalid);},"unknown policy rejected");
  for(auto policy:{preserve,apply}) {
    std::vector<std::string> calls;
    apply_device_config_policy(policy,true,[&](){calls.push_back("imaging");},[&](){calls.push_back("host");});
    check(calls.empty(),"probe executes zero imaging setters and zero host filters for either policy");
  }
  std::vector<std::string> calls;
  apply_device_config_policy(preserve,false,[&](){calls.push_back("imaging");},[&](){calls.push_back("host");});
  check(calls==std::vector<std::string>{"host"},"preserve capture applies host filter with zero imaging setters");
  calls.clear();apply_device_config_policy(apply,false,[&](){calls.push_back("imaging");},[&](){calls.push_back("host");});
  check(calls==std::vector<std::string>{"imaging","host"},"apply_xtcfg retains imaging-before-host order");
  calls.clear();bool host_failed=false;
  try{apply_device_config_policy(preserve,false,[&](){calls.push_back("imaging");},[&](){calls.push_back("host");throw std::runtime_error("fixture host failure");});}
  catch(const std::runtime_error&){host_failed=true;}
  check(host_failed && calls==std::vector<std::string>{"host"},"host failure is propagated and cannot trigger device setters");
  calls.clear();rejected([&](){apply_device_config_policy(static_cast<DeviceConfigPolicy>(99),true,[&](){calls.push_back("imaging");},[&](){calls.push_back("host");});},"invalid enum cannot bypass via probe");
  check(calls.empty(),"invalid policy does not call anything");
  check(xtcfg_disposition(preserve,false,"Setting.freq1")=="REQUESTED_NOT_APPLIED_DEVICE_PRESERVED","old nominal frequency not claimed applied");
  check(xtcfg_disposition(preserve,false,"Setting.HDR")=="REQUESTED_NOT_APPLIED_DEVICE_PRESERVED","old HDR not claimed applied");
  check(xtcfg_disposition(preserve,false,"Setting.minLSB")=="DEVICE_PRESERVED_HOST_LSB_CUT_APPLIED","device amplitude preserved while host LSB crop remains applied");
  check(xtcfg_disposition(preserve,false,"Filters.spatialEnable")=="HOST_SETTING_APPLIED","host spatial retained");
  check(xtcfg_disposition(apply,true,"Setting.HDR")=="REQUESTED_NOT_APPLIED_PROBE","probe takes precedence");

  // Exact read-only device_before.txt responses, supplied by the root's probe
  // reports/recapture/box_probe_20260913T061527Z. These are decoder fixtures,
  // not a test that connects to the devices or proves their optical behaviour.
  const auto left=unhex("52aacc334e000000030101010107004006c8001e0040060000d00700009f0000003b0046000000000a0a0300000000006400e8030807881300000000000800000000000000000000000000000000");
  const auto right=unhex("52aacc334e000000030101010107074006c8001e004006c800d00700009f0000003b0046000100000a0a0300000000006400e8030807881300000000001800000000000000000000000000000000");
  static_assert(offsetof(XinTan::DevCfg_t,channel)==37,"reviewed SDK channel location changed; audit required");
  check(left.size()==78 && right.size()==sizeof(XinTan::DevCfg_t),"real configuration byte size matches actual SDK");
  const auto a=device_config_snapshot(left),b=device_config_snapshot(right);
  check(a.known_fields_available && b.known_fields_available,"both actual config fixtures parsed");
  check(a.format=="SDK_V3_DEVCFG" && a.fields.at("version")==Bytes{0},"protocol detection does not require internal version=3");
  check(a.channel_valid && a.channel==0 && b.channel_valid && b.channel==1,"left channel 0 and right channel 1 retained");
  check(a.fields.at("freq")==Bytes({1,1,1,7,0}),"left nominal wire frequency indices");
  check(b.fields.at("freq")==Bytes({1,1,1,7,7}),"right nominal wire frequency indices");
  for(const auto* input:{&left,&right}) {
    const auto initial=device_config_snapshot(*input);
    auto after_start=*input;
    after_start[offsetof(XinTan::DevCfg_t,imageflags)]=1;
    after_start[offsetof(XinTan::DevCfg_t,usefps)]=7;
    check(device_config_changes(initial,device_config_snapshot(after_start)).empty(),"stream format and runtime FPS are not configuration overwrite");
    check(device_config_changes(initial,device_config_snapshot(*input)).empty(),"unchanged actual start snapshot passes");
  }
  for(const auto item:std::vector<std::pair<std::string,size_t>>{
      {"channel",offsetof(XinTan::DevCfg_t,channel)}, {"hdrmode",offsetof(XinTan::DevCfg_t,hdrmode)},
      {"freq",offsetof(XinTan::DevCfg_t,freq)+4}, {"integtime",offsetof(XinTan::DevCfg_t,integtime)+8},
      {"timesync_type",offsetof(XinTan::DevCfg_t,timesync_type)}, {"ptpdomain",offsetof(XinTan::DevCfg_t,ptpdomain)},
      {"setfps",offsetof(XinTan::DevCfg_t,setfps)}}) {
    auto changed_bytes=right;changed_bytes[item.second]^=1;
    check(changed(b,device_config_snapshot(changed_bytes),item.first),"known protection field mutation detected");
  }
  auto bad_magic=right;bad_magic[0]^=1;
  check(!device_config_snapshot(bad_magic).known_fields_available,"bad v3 magic not silently trusted");
  auto short_v3=right;short_v3.pop_back();
  check(!device_config_snapshot(short_v3).known_fields_available,"truncated v3 is not reinterpreted as legacy");
  auto extended=right;extended.push_back(0);
  check(!device_config_snapshot(extended).known_fields_available,"unknown v3 layout not trusted");
  check(!device_config_changes(b,device_config_snapshot({})).empty(),"missing post-start config cannot pass protection");
  Bytes legacy(14,0);const auto legacy_unknown=device_config_snapshot(legacy);
  check(legacy_unknown.known_fields_available && !legacy_unknown.channel_valid,"short legacy has no known channel even with stored zero");
  check(describe_device_config_snapshot(legacy_unknown).find("freq_channel=UNAVAILABLE")!=std::string::npos,"missing channel rendered unavailable");
  legacy.resize(28,0);const auto legacy_known=device_config_snapshot(legacy);
  check(legacy_known.channel_valid && legacy_known.channel==0,"present legacy channel zero is an actual observed value");
  check(!device_config_changes(legacy_unknown,legacy_known).empty(),"changed known-field availability is not unchanged");
  auto reserved=right;reserved[offsetof(XinTan::DevCfg_t,reserver)]^=1;
  check(device_config_changes(b,device_config_snapshot(reserved)).empty(),"unknown reserved fields are outside explicitly documented protection scope");
  std::cout<<"PASS: production probe/preserve/apply policy, zero forbidden setters, host filters, real 78-byte config fixtures, channel validity, known field changes and malformed readback\n";
  return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<"\n";return 1;}}
