#include "wc_xt_driver/udp_assembler.hpp"
#include <iostream>
#include <stdexcept>
using Bytes=std::vector<uint8_t>;
void check(bool ok,const char*what){if(!ok)throw std::runtime_error(what);}
void put(Bytes&b,size_t pos,uint32_t value,size_t count,bool little){for(size_t n=0;n<count;++n)b[pos+n]=value>>(8*(little?n:count-1-n));}
Bytes frame(bool little){Bytes b(64,3);b[0]=0x7e;b[1]=0xff;b[2]=0xaa;b[3]=0x55;put(b,4,52,4,little);b[60]=0xff;b[61]=0x7e;b[62]=0x55;b[63]=0xaa;return b;}
Bytes packet(const Bytes&b,int sn,size_t pos,size_t count,bool little){Bytes p(20+count,0);put(p,0,sn,2,little);put(p,2,b.size(),4,little);put(p,6,count,2,little);put(p,8,pos,4,little);std::copy(b.begin()+pos,b.begin()+pos+count,p.begin()+20);return p;}
int main(){try{
 for(bool little:{false,true}){
  wc_xt_driver::UdpAssembler a;Bytes out;auto b=frame(little);
  auto first=packet(b,11,0,20,little),last=packet(b,11,20,44,little);
  check(!a.feed(last,little,out),"out-of-order partial");
  check(!a.feed(last,little,out),"duplicate must not finish frame");
  check(a.counters.duplicate==1,"duplicate counted");
  check(a.feed(first,little,out),"frame completes");
  check(out==Bytes(b.begin()+8,b.end()-4),"payload preserved");
  for(size_t n=0;n<20;++n)check(!a.feed(Bytes(n),little,out),"truncated header rejected");
  auto bad=first;put(bad,8,63,4,little);check(!a.feed(bad,little,out),"offset overrun");
  bad=first;put(bad,2,0xffffffff,4,little);check(!a.feed(bad,little,out),"huge total rejected");
  bad=first;bad.pop_back();check(!a.feed(bad,little,out),"truncated payload rejected");
  check(!a.feed(first,little,out),"start conflicting frame");
  bad=first;bad[20]^=1;check(!a.feed(bad,little,out),"conflicting duplicate rejected");
  check(a.pending()==0,"conflicting frame discarded");
  for(int sn=20;sn<24;++sn)check(!a.feed(packet(b,sn,0,20,little),little,out),"partial frame");
  check(a.pending()==3 && a.counters.evicted==1,"bounded reassembly");
  a.reset();check(a.pending()==0,"reset clears epochs");
  b[4]^=1;check(!a.feed(packet(b,30,0,b.size(),little),little,out),"envelope length mismatch rejected");
 }
 std::cout<<"PASS: endian, order, duplicate, truncation, bounds, overlap, capacity, reset\n";return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}}
