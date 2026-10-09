// Exercises the actual reviewed vendor decoder with synthetic bytes only.
// No XtSdk instance, device socket, serial handle or vendor filter is opened.
#include "frame.h"
#include <cstring>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
void configure(XinTan::Frame& frame) {
  std::memset(&frame.info, 0, sizeof(frame.info));
  frame.frame_version=3;
  frame.info.magicToken=0x33CCAA50;
  frame.info.imageflags=XinTan::IMG_DIST|XinTan::IMG_AMP|XinTan::IMG_LEVEL;
  frame.info.unit_div=1;
  for (auto& integration : frame.info.integtime) integration=200;
  // Frequency code 0 exists in the actual vendor map. No device is queried.
  for (auto& frequency : frame.info.freq) frequency=0;
  frame.resetData();
}
XinTan::XByteArray bytes(size_t pixels, uint8_t packed_level) {
  XinTan::XByteArray result(56+pixels*4+(pixels+1)/2,0);
  for (size_t i=0;i<pixels;++i) {
    result[16+i*4]=0xe8; result[17+i*4]=3; // 1000 mm
    result[18+i*4]=100;                   // positive raw amplitude
  }
  for (size_t i=0;i<(pixels+1)/2;++i) result[16+pixels*4+i]=packed_level;
  return result;
}
}

int main() {
  try {
    std::string tag="wc_synthetic_hdr_bounds";
    XinTan::Frame frame(tag,XinTan::Frame::AMPLITUDE,1,2,2,16);
    for (uint8_t level=0;level<5;++level) {
      configure(frame);
      auto data=bytes(4,uint8_t(level|(level<<4)));
      frame.sortData(data);
      require(frame.sort_data_valid,"valid HDR stage was rejected");
      require(frame.leveldata.size()==4,"level pixel count changed");
      for (auto decoded : frame.leveldata) require(decoded==level,"valid HDR stage was altered");
      require(frame.rawdistData[0]==1000,"valid depth changed");
    }
    unsigned rejected=0;
    for (uint8_t level=5;level<16;++level) {
      for (bool high : {false,true}) {
        configure(frame);
        auto data=bytes(4,high?uint8_t(level<<4):level);
        frame.sortData(data);
        require(!frame.sort_data_valid,"out-of-range HDR nibble was accepted");
        require(!frame.hasPointcloud&&frame.points.empty(),"malformed frame retained cloud");
        ++rejected;
      }
    }
    require(rejected==22,"malformed stage coverage incomplete");
    // A successful earlier decode must not leave validity true on the same object.
    configure(frame); frame.sortData(bytes(4,0x40));
    require(frame.sort_data_valid,"recovery frame failed");
    frame.sortData(bytes(4,0x50));
    require(!frame.sort_data_valid,"stale successful validity survived rejection");
    configure(frame); auto truncated=bytes(4,0); truncated.resize(24);
    frame.sortData(truncated);
    require(!frame.sort_data_valid,"truncated level frame was accepted");
    configure(frame); frame.info.freq[4]=0xff;
    frame.sortData(bytes(4,0x44));
    require(!frame.sort_data_valid,"unknown selected frequency escaped rejection");
    XinTan::Frame odd(tag,XinTan::Frame::AMPLITUDE,2,3,1,16);
    configure(odd); odd.sortData(bytes(3,0));
    require(!odd.sort_data_valid,"odd pixel count allowed final nibble overrun");
    configure(frame); frame.sortData(bytes(4,0x04));
    require(frame.sort_data_valid,"valid frame failed after malformed frame sequence");
    std::cout<<"PASS: 5 valid stages, 22 invalid nibble positions, stale validity, truncation, unknown frequency, odd dimensions and recovery\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr<<"FAIL: "<<error.what()<<"\n";
    return 1;
  }
}
