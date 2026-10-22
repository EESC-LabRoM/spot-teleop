// Request-driven RGB-D transport over Tailscale TCP. SDK capture timestamps are preserved.
#include <sl/Camera.hpp>
#include <cstdio>
#include <jpeglib.h>
#include <zlib.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <atomic>
#include <csignal>
#include <sstream>
#include <vector>
#include <thread>
#include <mutex>
#include <iostream>
#include <cmath>
std::atomic<bool> running{true};
void stop(int){running=false;}
bool sendall(int fd,const void* p,size_t n){auto b=(const char*)p;while(n){auto k=send(fd,b,n,MSG_NOSIGNAL);if(k<=0)return false;b+=k;n-=k;}return true;}
int main(int argc,char**argv){
 signal(SIGINT,stop);signal(SIGTERM,stop);signal(SIGPIPE,SIG_IGN);
 sl::Camera cam;sl::InitParameters init;init.camera_resolution=sl::RESOLUTION::HD720;init.camera_fps=15;
 init.depth_mode=sl::DEPTH_MODE::PERFORMANCE;init.coordinate_units=sl::UNIT::METER;init.depth_minimum_distance=.3;init.depth_maximum_distance=3.;init.sdk_verbose=1;init.input.setFromSerialNumber(34046618);
 auto e=cam.open(init);if(e!=sl::ERROR_CODE::SUCCESS){std::cerr<<sl::toString(e)<<std::endl;return 1;}
 sl::Resolution res(640,360),dres(320,180);auto k=cam.getCameraInformation(res).camera_configuration.calibration_parameters.left_cam;
 std::mutex lock;std::string latest_header;std::vector<unsigned char> latest_rgb,latest_depth;
 std::thread capture([&]{sl::Mat rgb,depth;unsigned count=0;
  while(running){if(cam.grab()!=sl::ERROR_CODE::SUCCESS)continue;
   auto ts=cam.getTimestamp(sl::TIME_REFERENCE::IMAGE).getNanoseconds();
   cam.retrieveImage(rgb,sl::VIEW::LEFT,sl::MEM::CPU,res);cam.retrieveMeasure(depth,sl::MEASURE::DEPTH,sl::MEM::CPU,dres);
   jpeg_compress_struct c;jpeg_error_mgr err;c.err=jpeg_std_error(&err);jpeg_create_compress(&c);unsigned char* out=nullptr;unsigned long len=0;jpeg_mem_dest(&c,&out,&len);
   c.image_width=640;c.image_height=360;c.input_components=3;c.in_color_space=JCS_RGB;jpeg_set_defaults(&c);jpeg_set_quality(&c,70,TRUE);jpeg_start_compress(&c,TRUE);
   std::vector<unsigned char> row(640*3);
   while(c.next_scanline<c.image_height){auto src=rgb.getPtr<sl::uchar1>()+c.next_scanline*rgb.getStepBytes();for(int x=0;x<640;++x){row[3*x]=src[4*x+2];row[3*x+1]=src[4*x+1];row[3*x+2]=src[4*x];}JSAMPROW r=row.data();jpeg_write_scanlines(&c,&r,1);}
   jpeg_finish_compress(&c);std::vector<unsigned char> jpg(out,out+len);free(out);jpeg_destroy_compress(&c);
   std::vector<uint16_t> mm(320*180);for(int y=0;y<180;++y){auto src=(float*)((char*)depth.getPtr<float>()+y*depth.getStepBytes());for(int x=0;x<320;++x){float z=src[x];mm[y*320+x]=std::isfinite(z)&&z>=.3f&&z<=3.f?uint16_t(std::lround(z*1000)):0;}}
   uLongf size=compressBound(mm.size()*2);std::vector<unsigned char> packed(size);compress2(packed.data(),&size,(Bytef*)mm.data(),mm.size()*2,1);packed.resize(size);
   std::ostringstream h;h<<"{\"stamp_ns\":"<<ts<<",\"rgb_bytes\":"<<jpg.size()<<",\"depth_bytes\":"<<packed.size()<<",\"fx\":"<<k.fx<<",\"fy\":"<<k.fy<<",\"cx\":"<<k.cx<<",\"cy\":"<<k.cy<<"}\n";
   {std::lock_guard<std::mutex> g(lock);latest_header=h.str();latest_rgb=std::move(jpg);latest_depth=std::move(packed);}
   if(++count%150==0)std::cout<<"RGBD frames "<<count<<std::endl;
  }
 });
 int s=socket(AF_INET,SOCK_STREAM,0),one=1;setsockopt(s,SOL_SOCKET,SO_REUSEADDR,&one,sizeof(one));sockaddr_in addr{};addr.sin_family=AF_INET;addr.sin_port=htons(30020);inet_pton(AF_INET,argc>1?argv[1]:"100.118.129.74",&addr.sin_addr);
 if(bind(s,(sockaddr*)&addr,sizeof(addr))||listen(s,1)){perror("listen");running=false;capture.join();cam.close();return 2;}
 std::cout<<"RGBD TCP ready port 30020; serial "<<cam.getCameraInformation().serial_number<<" SDK "<<sl::Camera::getSDKVersion()<<std::endl;
 while(running){fd_set f;FD_ZERO(&f);FD_SET(s,&f);timeval wait{1,0};if(select(s+1,&f,nullptr,nullptr,&wait)<=0)continue;int c=accept(s,nullptr,nullptr);setsockopt(c,IPPROTO_TCP,TCP_NODELAY,&one,sizeof(one));timeval timeout{5,0};setsockopt(c,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));setsockopt(c,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));
  char request;while(running&&recv(c,&request,1,0)==1){std::string h;std::vector<unsigned char> rgb,depth;{std::lock_guard<std::mutex> g(lock);h=latest_header;rgb=latest_rgb;depth=latest_depth;}if(h.empty())break;if(!sendall(c,h.data(),h.size())||!sendall(c,rgb.data(),rgb.size())||!sendall(c,depth.data(),depth.size()))break;}close(c);
 }
 close(s);capture.join();cam.close();return 0;
}
