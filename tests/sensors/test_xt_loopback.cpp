// Synthetic Linux loopback only. Never connects a sensor or a control port.
#include "communicationNet.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>
void check(bool ok,const char*what){if(!ok)throw std::runtime_error(what);}
int bound(const char*ip,uint16_t port){int s=socket(AF_INET,SOCK_DGRAM|SOCK_CLOEXEC,0);sockaddr_in a{};a.sin_family=AF_INET;a.sin_port=htons(port);inet_pton(AF_INET,ip,&a.sin_addr);if(bind(s,reinterpret_cast<sockaddr*>(&a),sizeof(a))){close(s);return -1;}return s;}
void send_packet(int sender,uint16_t port){std::vector<uint8_t> b(35,0);b[0]=1;b[2]=15;b[6]=15;b[20]=0x7e;b[21]=0xff;b[22]=0xaa;b[23]=0x55;b[24]=3;b[28]=0xfc;b[31]=0xff;b[32]=0x7e;b[33]=0x55;b[34]=0xaa;sockaddr_in a{};a.sin_family=AF_INET;a.sin_port=htons(port);inet_pton(AF_INET,"127.0.0.1",&a.sin_addr);check(sendto(sender,b.data(),b.size(),0,reinterpret_cast<sockaddr*>(&a),sizeof(a))==ssize_t(b.size()),"send loopback");}
int main(){try{
 int reserved=bound("127.0.0.1",0);check(reserved>=0,"reserve ephemeral port");sockaddr_in a{};socklen_t n=sizeof(a);check(getsockname(reserved,reinterpret_cast<sockaddr*>(&a),&n)==0,"read assigned port");const auto port=ntohs(a.sin_port);close(reserved);
 boost::asio::io_service io;std::string tag="synthetic-loopback";
 XinTan::CommnunicationNet first(io,tag),second(io,tag),collision(io,tag);
 first.localBindIp="127.0.0.1";first.address="127.0.0.3";first.endianType=0;
 second.localBindIp="127.0.0.2";second.address="127.0.0.4";
 collision.localBindIp="127.0.0.1";
 check(first.openUdp(port),"first concrete bind");check(second.openUdp(port),"same port different IP");check(!collision.openUdp(port),"same tuple collision rejected");
 int wrong=bound("127.0.0.4",0),right=bound("127.0.0.3",0);check(wrong>=0&&right>=0,"loopback source bind");
 send_packet(wrong,port);std::this_thread::sleep_for(std::chrono::milliseconds(5));XinTan::XByteArray out;check(!first.udpReceiveFrame(out),"wrong source rejected");
 send_packet(right,port);std::this_thread::sleep_for(std::chrono::milliseconds(5));check(first.udpReceiveFrame(out),"expected source accepted");check(out.size()==3&&out[0]==0xfc,"source payload preserved");
 close(wrong);close(right);first.closeUdp();second.closeUdp();check(collision.openUdp(port),"released tuple reusable");collision.closeUdp();
 std::cout<<"PASS: SYNTHETIC loopback same-port concrete-IP isolation and source filtering\n";return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}}
