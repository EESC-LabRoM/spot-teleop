// ZED operator camera sender: encode stereo only; depth and inference run on the A2000.
#include <sl/Camera.hpp>
#include <atomic>
#include <csignal>
#include <iostream>
#include <thread>
std::atomic<bool> running{true};
void stop(int) { running=false; }
int main(int argc,char** argv) {
    std::signal(SIGINT,stop);std::signal(SIGTERM,stop);
    sl::Camera camera;
    sl::InitParameters init;
    init.camera_resolution=sl::RESOLUTION::HD720;
    init.camera_fps=15;
    init.depth_mode=sl::DEPTH_MODE::NONE;
    init.sdk_verbose=1;
    auto result=camera.open(init);
    if(result!=sl::ERROR_CODE::SUCCESS) {std::cerr<<sl::toString(result)<<std::endl;return 1;}
    sl::StreamingParameters stream;
    stream.codec=sl::STREAMING_CODEC::H264;
    stream.bitrate=5000;
    stream.chunk_size=1024; // Fits Tailscale MTU 1280, including UDP/IP headers.
    stream.port=(argc>1)?std::stoi(argv[1]):30000;
    result=camera.enableStreaming(stream);
    if(result!=sl::ERROR_CODE::SUCCESS) {std::cerr<<sl::toString(result)<<std::endl;camera.close();return 2;}
    std::cout<<"ZED serial "<<camera.getCameraInformation().serial_number<<" streaming stereo H264 on port "<<stream.port<<std::endl;
    unsigned count=0;
    while(running) {
        result=camera.grab();
        if(result!=sl::ERROR_CODE::SUCCESS) {std::cerr<<sl::toString(result)<<std::endl;std::this_thread::sleep_for(std::chrono::milliseconds(50));}
        else if(++count%150==0) std::cout<<"Frames "<<count<<std::endl;
    }
    camera.disableStreaming();camera.close();return 0;
}
