#pragma once
#include "cmdstructs.h" // reviewed SDK wire structures; no SDK instance or device I/O
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace wc_xt_driver {
enum class DeviceConfigPolicy { PreserveCurrent, ApplyXtConfig };
inline DeviceConfigPolicy parse_device_config_policy(const std::string& value) {
  if(value=="preserve_current")return DeviceConfigPolicy::PreserveCurrent;
  if(value=="apply_xtcfg")return DeviceConfigPolicy::ApplyXtConfig;
  throw std::invalid_argument("device_config_policy must be preserve_current or apply_xtcfg");
}
inline const char* device_config_policy_name(DeviceConfigPolicy policy) {
  switch(policy) {
    case DeviceConfigPolicy::PreserveCurrent:return "preserve_current";
    case DeviceConfigPolicy::ApplyXtConfig:return "apply_xtcfg";
  }
  throw std::invalid_argument("invalid device configuration policy");
}

// Production uses this exact branch. Probe never calls either callback; normal
// preservation still applies the host filters, without any device setter.
template<class Imaging, class Host>
void apply_device_config_policy(DeviceConfigPolicy policy,bool probe,Imaging imaging,Host host) {
  device_config_policy_name(policy); // reject invalid enum even for a probe
  if(probe)return;
  if(policy==DeviceConfigPolicy::ApplyXtConfig)imaging();
  host();
}
inline std::string xtcfg_disposition(DeviceConfigPolicy policy,bool probe,const std::string& key) {
  if(probe)return "REQUESTED_NOT_APPLIED_PROBE";
  if(key=="Setting.deviceType"||key=="Setting.renderType")return "GUI_METADATA_ONLY";
  if(key=="Setting.pclFilterOn")return "UNSUPPORTED_WINDOWS_POINTCLOUD_PROCESSOR_SWITCH";
  if(key=="Setting.imgType")return "STREAM_FORMAT_POINTCLOUDAMP_REQUESTED";
  if(policy==DeviceConfigPolicy::PreserveCurrent && key=="Setting.minLSB")
    return "DEVICE_PRESERVED_HOST_LSB_CUT_APPLIED";
  if(key.rfind("Filters.",0)==0 || key=="Setting.hmirror" || key=="Setting.vmirror" || key=="Setting.cut_corner")
    return "HOST_SETTING_APPLIED";
  return policy==DeviceConfigPolicy::PreserveCurrent ? "REQUESTED_NOT_APPLIED_DEVICE_PRESERVED" : "APPLIED_SDK_OR_READBACK_MATCH_SEE_COMMAND_LOG";
}

struct DeviceConfigSnapshot {
  std::string format="UNKNOWN";
  bool known_fields_available=false;
  bool channel_valid=false;
  uint8_t channel=0;
  std::map<std::string,std::vector<uint8_t>> fields;
};
inline std::string config_bytes_hex(const std::vector<uint8_t>& bytes) {
  std::ostringstream out;out<<std::hex<<std::setfill('0');
  for(auto byte:bytes)out<<std::setw(2)<<unsigned(byte);
  return out.str();
}
inline DeviceConfigSnapshot device_config_snapshot(const std::vector<uint8_t>& raw) {
  DeviceConfigSnapshot snapshot;
  const bool v3_magic=raw.size()>=4 && raw[0]==0x52 && raw[1]==0xaa && raw[2]==0xcc && raw[3]==0x33;
  if(v3_magic && raw.size()!=sizeof(XinTan::DevCfg_t)) {
    snapshot.format="V3_MAGIC_WITH_UNEXPECTED_SIZE";return snapshot;
  }
  auto add=[&](const char* name,size_t offset,size_t length) {
    if(offset>raw.size()||length>raw.size()-offset)throw std::invalid_argument("device config field out of response bounds");
    snapshot.fields.emplace(name,std::vector<uint8_t>(raw.begin()+offset,raw.begin()+offset+length));
  };
  // getDevConfig() uses the exact packed DevCfg_t size for v3. Check the
  // documented magic as well. Do not infer layout from a plausible channel 0.
  if(raw.size()==sizeof(XinTan::DevCfg_t)) {
    if(!v3_magic) {
      snapshot.format="V3_SIZE_WITH_INVALID_MAGIC";return snapshot;
    }
    snapshot.format="SDK_V3_DEVCFG";
#define WC_CONFIG_FIELD(field) add(#field,offsetof(XinTan::DevCfg_t,field),sizeof(((XinTan::DevCfg_t*)nullptr)->field))
    WC_CONFIG_FIELD(version);WC_CONFIG_FIELD(endiantype);
    WC_CONFIG_FIELD(hdrmode);WC_CONFIG_FIELD(freq);WC_CONFIG_FIELD(integtime);WC_CONFIG_FIELD(integtimegs);
    WC_CONFIG_FIELD(roix);WC_CONFIG_FIELD(roiy);WC_CONFIG_FIELD(miniAmp);WC_CONFIG_FIELD(channel);
    WC_CONFIG_FIELD(binning);WC_CONFIG_FIELD(reduce);WC_CONFIG_FIELD(setfps);WC_CONFIG_FIELD(vcsel);
    WC_CONFIG_FIELD(timesync_type);WC_CONFIG_FIELD(ptpdomain);WC_CONFIG_FIELD(bautorun);WC_CONFIG_FIELD(bdhcp);
    WC_CONFIG_FIELD(bcut_filteron);WC_CONFIG_FIELD(cutIntDist);
#undef WC_CONFIG_FIELD
    snapshot.channel_valid=true;snapshot.channel=raw[offsetof(XinTan::DevCfg_t,channel)];
    // imageflags is selected by stream start; usefps is runtime feedback.
    // Unknown reserved bytes remain archived but are not called validated.
  } else if(raw.size()>13 && raw.size()<sizeof(XinTan::DevCfg_t)) {
    snapshot.format="SDK_LEGACY_PARTIAL";
    add("legacy_modFreq",1,1);add("legacy_hdrmode",2,1);add("legacy_miniAmp",3,2);
    add("legacy_integrationTimes_1_to_3",5,6);add("legacy_integrationTimeGs",11,2);add("legacy_isFilterOn",13,1);
    if(raw.size()>25) {add("legacy_roi",14,8);add("legacy_bCompensateOn",23,1);add("legacy_binning",24,1);}
    if(raw.size()>27) {
      add("legacy_setmaxfps",25,1);add("legacy_freqChannel",26,1);
      snapshot.channel_valid=true;snapshot.channel=raw[26];
    }
  } else return snapshot;
  snapshot.known_fields_available=true;return snapshot;
}
inline std::vector<std::string> device_config_changes(const DeviceConfigSnapshot& before,const DeviceConfigSnapshot& after) {
  std::vector<std::string> changes;
  if(!before.known_fields_available||!after.known_fields_available)changes.push_back("KNOWN_CONFIGURATION_FIELDS_UNAVAILABLE");
  if(before.format!=after.format)changes.push_back("RESPONSE_FORMAT_CHANGED");
  for(const auto& item:before.fields) {
    const auto found=after.fields.find(item.first);
    if(found==after.fields.end()||found->second!=item.second)changes.push_back(item.first);
  }
  for(const auto& item:after.fields)if(!before.fields.count(item.first))changes.push_back("ADDED_FIELD:"+item.first);
  return changes;
}
inline std::string describe_device_config_snapshot(const DeviceConfigSnapshot& value) {
  std::ostringstream out;
  out<<"raw_config_format="<<value.format<<"\nknown_protected_fields_available="<<(value.known_fields_available?"true":"false")
     <<"\nfreq_channel_valid="<<(value.channel_valid?"true":"false")<<"\nfreq_channel=";
  if(value.channel_valid)out<<unsigned(value.channel);else out<<"UNAVAILABLE";
  out<<"\nprotection_scope=known_config_fields_only;excludes_stream_imageflags_runtime_usefps_unknown_reserved\n";
  for(const auto& field:value.fields)out<<"protected_field_hex."<<field.first<<"="<<config_bytes_hex(field.second)<<"\n";
  return out.str();
}
} // namespace wc_xt_driver
