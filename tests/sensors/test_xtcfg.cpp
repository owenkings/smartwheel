// Pure configuration/mapping and command-policy test. No device handles.
#include "wc_xt_driver/xtcfg.hpp"
#include "wc_xt_driver/imaging_allowlist.hpp"
#include <fstream>
#include <iostream>
#include <vector>
static void check(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
static std::string read(const std::string& path){std::ifstream f(path);std::ostringstream s;s<<f.rdbuf();return s.str();}
struct FakeSdk {
 unsigned flags=0;std::vector<std::string> calls;bool reject_spatial=false;
 bool clearAllSdkFilter(){flags=0;calls.push_back("clear");return true;}
 void setTransMirror(bool h,bool v){check(!h&&!v,"mirror mismatch");}
 void setPointsLsbCut(int n){check(n==70,"amp mapping");}
 void setCutCorner(int n){check(n==90,"corner mapping");}
 bool setSdkMedianFilter(int n){check(n==3,"median mapping");flags|=8;calls.push_back("median");return true;}
 bool setSdkKalmanFilter(int f,int t,int dt){check(f==300&&t==300&&dt==2000,"kalman mapping");flags|=1;calls.push_back("kalman");return true;}
 bool setSdkEdgeFilter(int t){check(t==150,"edge mapping");flags|=4;calls.push_back("edge");return true;}
 bool setSdkDustFilter(int t,int n,int dt,int valid){check(t==9000&&n==2&&dt==300&&valid==100,"dust mapping");flags|=16;calls.push_back("dust");return true;}
 void setPostProcess(float t,int mode,int w,int m){check(t==5&&mode==1&&w==9&&m==3,"post/dynamics mapping");flags|=32;calls.push_back("post");}
 bool setSdkReflectiveFilter(float a,float b){check(a==.5f&&b==2,"reflective mapping");flags|=64;calls.push_back("reflective");return true;}
 bool setSdkAvgFilter(int,int){throw std::runtime_error("disabled average unexpectedly called");}
 bool setSpatialFilter(float a,unsigned d,unsigned i){check(std::abs(a-.7f)<1e-6&&d==70&&i==2,"spatial mapping");if(reject_spatial)return false;flags|=128;calls.push_back("spatial");return true;}
 unsigned getSdkFilterFlags(){return flags;}
};
int main(int argc,char**argv){try{
 check(argc==2,"config directory required");
 for(const auto side:{"left","right"}) {
   const auto text=read(std::string(argv[1])+"/"+side+"-2026-09-11.xtcfg");auto c=wc_xt_driver::XtConfig::parse(text);
   check(c.values.size()==46,"field coverage changed");
   check(c.frequency_wire(1)==1&&c.frequency_wire(4)==7,"frequency enum translation");
   check(c.integer("Setting.int5")== (std::string(side)=="left"?0:200),"left/right fifth exposure lost");
   check(c.frequency_wire(5)==(std::string(side)=="left"?0:7),"fifth frequency overwritten");
   FakeSdk sdk;c.apply_host(sdk);
   check(sdk.flags==253,"effective flags");
   check(sdk.calls==std::vector<std::string>{"clear","median","kalman","edge","dust","post","reflective","spatial"},"host API sequence");
   sdk.reject_spatial=true;bool failed=false;try{c.apply_host(sdk);}catch(...){failed=true;}check(failed,"filter failure hidden");
   for(const auto suffix:{"\n[Filters]\nspatialEnable=0\n","\n[Setting]\nunreviewed=1\n"}) {
     failed=false;try{wc_xt_driver::XtConfig::parse(text+suffix);}catch(...){failed=true;}check(failed,"duplicate/unknown accepted");
   }
 }
 auto allow=wc_xt_driver::allowed_sensor_command;
 for(int id=0;id<256;++id) {
   if(id==0||id==2||id==4||id==5||id==18)check(allow(id,{}),"read rejected");
   else check(!allow(id,{}),"empty non-read command permitted");
 }
 check(allow(8,std::vector<uint8_t>(12))&&!allow(8,std::vector<uint8_t>(11)),"exposure payload bound");
 check(allow(27,{1,1,1,7,0})&&allow(27,{1,1,1,7,7}),"five frequency payload");
 check(!allow(27,{1,1,1,7})&&!allow(27,{1,1,1,7,7,7})&&!allow(27,{1,1,1,7,255}),"frequency bounds");
 check(allow(1,{2,1})&&!allow(1,{2,2}),"start bounds");
 for(int id:{3,6,7,11,19,29,49,50,152,153,202})check(!allow(id,{0,0,0,0,0,0}),"unapproved command permitted");
 std::cout<<"PASS strict 46-key left/right parsing, SDK arguments, spatial failure, 256-ID command policy and five-frequency bounds\n";
 return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}}
