#pragma once
#include <array>
#include <cmath>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
namespace wc_xt_driver {
// Strict format validation is not a point-cloud quality acceptance gate.
// Every requested field receives an application disposition in the run record.
class XtConfig {
 public:
  std::map<std::string,std::string> values;
  static std::string trim(const std::string& s) {
    const auto begin=s.find_first_not_of(" \t\r\n");
    return begin==std::string::npos ? "" : s.substr(begin,s.find_last_not_of(" \t\r\n")-begin+1);
  }
  static XtConfig parse(const std::string& text) {
    if(text.size()>65536)throw std::invalid_argument("xtcfg exceeds 64 KiB");
    XtConfig c;std::istringstream in(text);std::string line,section;
    while(std::getline(in,line)) {
      line=trim(line);if(line.empty()||line[0]=='#'||line[0]==';')continue;
      if(line[0]=='[') {
        if(line.back()!=']')throw std::invalid_argument("malformed xtcfg section");
        section=line.substr(1,line.size()-2);
        if(section!="Setting"&&section!="Filters")throw std::invalid_argument("unknown xtcfg section");
        continue;
      }
      const auto at=line.find('=');
      if(section.empty()||at==std::string::npos)throw std::invalid_argument("malformed xtcfg assignment");
      const auto key=section+"."+trim(line.substr(0,at));
      if(!c.values.emplace(key,trim(line.substr(at+1))).second)throw std::invalid_argument("duplicate xtcfg key: "+key);
    }
    const std::set<std::string> integer_keys={
      "Setting.imgType","Setting.deviceType","Setting.HDR","Setting.int1","Setting.int2","Setting.int3","Setting.int4","Setting.int5","Setting.intgs","Setting.minLSB","Setting.hmirror","Setting.vmirror","Setting.maxfps","Setting.cut_corner","Setting.freq1","Setting.freq2","Setting.freq3","Setting.freq4","Setting.freq5","Setting.renderType","Setting.pclFilterOn","Setting.hbinning","Setting.vbinning",
      "Filters.medianSize","Filters.kalmanEnable","Filters.kalmanThreshold","Filters.edgeEnable","Filters.edgeThreshold","Filters.dustEnable","Filters.dustThreshold","Filters.dustFrames","Filters.postprocessThreshold","Filters.postprocessEnable","Filters.dynamicsEnabled","Filters.dynamicsWinsize","Filters.dynamicsMotionsize","Filters.reflectiveEnable","Filters.spatialEnable","Filters.spatialDelta","Filters.spatialIterations","Filters.averageEnable","Filters.averageSize"};
    const std::set<std::string> real_keys={"Filters.kalmanFactor","Filters.ref_th_min","Filters.ref_th_max","Filters.spatialAlpha"};
    if(c.values.size()!=integer_keys.size()+real_keys.size())throw std::invalid_argument("missing or unknown xtcfg keys");
    for(auto& k:integer_keys)c.integer(k,0,65535);
    for(auto& k:real_keys)c.number(k,0,65535);
    for(auto& kv:c.values)if(!integer_keys.count(kv.first)&&!real_keys.count(kv.first))throw std::invalid_argument("unknown xtcfg key: "+kv.first);
    if(c.integer("Setting.imgType")!=4)throw std::invalid_argument("PointCloud+Amp imgType=4 required");
    c.integer("Setting.HDR",0,2);c.integer("Setting.maxfps",1,255);
    for(auto k:{"Setting.hmirror","Setting.vmirror","Setting.pclFilterOn","Setting.hbinning","Setting.vbinning","Filters.kalmanEnable","Filters.edgeEnable","Filters.dustEnable","Filters.postprocessEnable","Filters.dynamicsEnabled","Filters.reflectiveEnable","Filters.spatialEnable","Filters.averageEnable"})c.integer(k,0,1);
    // A mirror would change the FLU frame convention. These supplied configs are zero.
    if(c.integer("Setting.hmirror")||c.integer("Setting.vmirror"))throw std::invalid_argument("mirrored geometry needs an explicit coordinate contract");
    for(int n=1;n<=5;++n)c.frequency(n);
    c.number("Filters.kalmanFactor",0,1);c.number("Filters.spatialAlpha",0,1);
    const auto median=c.integer("Filters.medianSize",0,31);
    if(median && median%2==0)throw std::invalid_argument("median size must be odd");
    c.integer("Filters.dustFrames",2,9);c.integer("Filters.spatialIterations",1,255);
    c.integer("Filters.dynamicsWinsize",1,255);c.integer("Filters.dynamicsMotionsize",1,255);
    if(c.number("Filters.ref_th_min")>c.number("Filters.ref_th_max"))throw std::invalid_argument("reflectivity minimum exceeds maximum");
    return c;
  }
  int integer(const std::string& key,int low=0,int high=65535)const {
    auto it=values.find(key);if(it==values.end())throw std::invalid_argument("missing xtcfg key: "+key);
    size_t used=0;int n=0;try{n=std::stoi(it->second,&used);}catch(...){throw std::invalid_argument("invalid xtcfg integer: "+key);}
    if(used!=it->second.size()||n<low||n>high)throw std::invalid_argument("xtcfg integer range: "+key);
    return n;
  }
  double number(const std::string& key,double low=0,double high=65535)const {
    auto it=values.find(key);if(it==values.end())throw std::invalid_argument("missing xtcfg key: "+key);
    size_t used=0;double n=0;try{n=std::stod(it->second,&used);}catch(...){throw std::invalid_argument("invalid xtcfg number: "+key);}
    if(used!=it->second.size()||!std::isfinite(n)||n<low||n>high)throw std::invalid_argument("xtcfg number range: "+key);
    return n;
  }
  int frequency(int n)const {
    const auto f=integer("Setting.freq"+std::to_string(n),0,6);
    if(f==5)throw std::invalid_argument("SDK multi-frequency has no verified 0.75 MHz wire mapping");return f;
  }
  int frequency_wire(int n)const {return std::array<int,7>{1,3,0,7,15,-1,4}[frequency(n)];}
  template<class Sdk> void apply_host(Sdk& sdk)const {
    if(!sdk.clearAllSdkFilter())throw std::runtime_error("clear host filters failed");
    sdk.setTransMirror(false,false);sdk.setPointsLsbCut(integer("Setting.minLSB"));sdk.setCutCorner(integer("Setting.cut_corner"));
    auto need=[](bool ok,const char* name){if(!ok)throw std::runtime_error(std::string("SDK filter rejected: ")+name);};
    if(integer("Filters.medianSize"))need(sdk.setSdkMedianFilter(integer("Filters.medianSize")),"median");
    if(integer("Filters.kalmanEnable"))need(sdk.setSdkKalmanFilter(static_cast<unsigned>(std::lround(number("Filters.kalmanFactor")*1000)),integer("Filters.kalmanThreshold"),2000),"kalman");
    if(integer("Filters.edgeEnable"))need(sdk.setSdkEdgeFilter(integer("Filters.edgeThreshold")),"edge");
    if(integer("Filters.dustEnable"))need(sdk.setSdkDustFilter(integer("Filters.dustThreshold"),integer("Filters.dustFrames"),300,100),"dust");
    if(integer("Filters.postprocessEnable"))sdk.setPostProcess(integer("Filters.postprocessThreshold"),integer("Filters.dynamicsEnabled"),integer("Filters.dynamicsWinsize"),integer("Filters.dynamicsMotionsize"));
    if(integer("Filters.reflectiveEnable"))need(sdk.setSdkReflectiveFilter(number("Filters.ref_th_min"),number("Filters.ref_th_max")),"reflective");
    if(integer("Filters.averageEnable"))need(sdk.setSdkAvgFilter(integer("Filters.averageSize"),300),"average");
    if(integer("Filters.spatialEnable"))need(sdk.setSpatialFilter(number("Filters.spatialAlpha"),integer("Filters.spatialDelta"),integer("Filters.spatialIterations")),"spatial");
    unsigned flags=0;
    if(integer("Filters.kalmanEnable"))flags|=1;
    if(integer("Filters.averageEnable"))flags|=2;
    if(integer("Filters.edgeEnable"))flags|=4;
    if(integer("Filters.medianSize"))flags|=8;
    if(integer("Filters.dustEnable"))flags|=16;
    if(integer("Filters.postprocessEnable"))flags|=32;
    if(integer("Filters.reflectiveEnable"))flags|=64;
    if(integer("Filters.spatialEnable"))flags|=128;
    if(sdk.getSdkFilterFlags()!=flags)throw std::runtime_error("SDK effective host filter flags differ");
  }
};
}
