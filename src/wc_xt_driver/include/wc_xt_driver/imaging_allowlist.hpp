#pragma once
#include <cstdint>
#include <vector>
namespace wc_xt_driver {
inline bool allowed_sensor_command(uint8_t id, const std::vector<uint8_t>& data) {
  switch(id) {
    case 0: case 2: case 4: case 5: case 18: return data.empty();
    // SDK start payload is verified separately against the SDK implementation.
    case 1: return data.size()==2 && data[0]>=1 && data[0]<=3 && data[1]<=1;
    case 8: return data.size()==12; // six uint16 exposure times
    case 9: return data.size()==2;  // minimum amplitude
    case 10: return data.size()==1 && data[0]<=2; // HDR wire enum
    case 12: return data.size()==1 && data[0]<=1; // vertical binning only
    case 27:
      if(data.size()!=5)return false;
      for(auto code:data)if(code!=0&&code!=1&&code!=3&&code!=4&&code!=7&&code!=15)return false;
      return true;
    case 56: return data.size()==1 && data[0]>0; // maximum FPS
    default: return false;
  }
}
}
