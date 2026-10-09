#pragma once
#include <algorithm>
#include <cstdint>
#include <map>
#include <vector>
namespace wc_xt_driver {
// Layout verified against upstream udppacket(): 20-byte header, SN, total,
// payload length and byte offset. Bounded, duplicate-aware frame reassembly.
class UdpAssembler {
 public:
  struct Counters { uint64_t invalid=0, duplicate=0, evicted=0, complete=0; } counters;
  bool feed(const std::vector<uint8_t>& p, bool little, std::vector<uint8_t>& out) {
    out.clear();
    if (p.size()<20) return invalid();
    auto read=[&](size_t at,size_t n) { uint32_t v=0; for(size_t i=0;i<n;++i) v|=uint32_t(p[at+i])<<(8*(little?i:n-1-i)); return v; };
    const auto sn=uint16_t(read(0,2));
    const auto total=read(2,4), length=read(6,2), offset=read(8,4);
    if(total<12 || total>1'200'000 || !length || length>1400 || p.size()!=20+length || offset>total || length>total-offset) return invalid();
    if(!frames_.count(sn)) {
      if(frames_.size()==3) {
        auto old=std::min_element(frames_.begin(),frames_.end(),[](const auto&a,const auto&b){return a.second.order<b.second.order;});
        frames_.erase(old); ++counters.evicted;
      }
      frames_.emplace(sn, Frame{std::vector<uint8_t>(total),std::vector<uint8_t>(total),0,++order_});
    }
    auto& f=frames_.at(sn);
    if(f.bytes.size()!=total) {frames_.erase(sn); return invalid();}
    bool duplicate=true;
    for(size_t i=0;i<length;++i) {
      if(f.seen[offset+i] && f.bytes[offset+i]!=p[20+i]) {frames_.erase(sn); return invalid();}
      duplicate=duplicate && f.seen[offset+i];
    }
    if(duplicate) {++counters.duplicate; return false;}
    for(size_t i=0;i<length;++i) {
      if(!f.seen[offset+i]) {f.seen[offset+i]=1; ++f.received;}
      f.bytes[offset+i]=p[20+i];
    }
    if(f.received!=total) return false;
    const auto& b=f.bytes;
    if(!std::equal(start_,start_+4,b.begin()) || !std::equal(end_,end_+4,b.end()-4)) {frames_.erase(sn);return invalid();}
    const uint32_t n=little ? uint32_t(b[4])|uint32_t(b[5])<<8|uint32_t(b[6])<<16|uint32_t(b[7])<<24 : uint32_t(b[7])|uint32_t(b[6])<<8|uint32_t(b[5])<<16|uint32_t(b[4])<<24;
    if(n!=total-12) {frames_.erase(sn);return invalid();}
    out.assign(b.begin()+8,b.end()-4); frames_.erase(sn); ++counters.complete; return true;
  }
  void reset(){frames_.clear();}
  size_t pending()const{return frames_.size();}
 private:
  bool invalid(){++counters.invalid;return false;}
  struct Frame {std::vector<uint8_t> bytes,seen;size_t received;uint64_t order;};
  std::map<uint16_t,Frame> frames_;
  uint64_t order_=0;
  static constexpr uint8_t start_[4]={0x7e,0xff,0xaa,0x55};
  static constexpr uint8_t end_[4]={0xff,0x7e,0x55,0xaa};
};
}
