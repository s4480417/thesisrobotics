#ifndef KIN_DRIVER_HPP
#define KIN_DRIVER_HPP

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;

void func(float& e);

class Kinect {
private:
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;
    Mat raw_color;
    Mat raw_depth;
    Mat foreground_color;
    Mat foreground_depth;
public:
    Kinect();
    int start();
    int stop();
    std::string getSerialNumber();
    std::string getFirmwareVersion();
    void updateStream();
    Mat getRawColor();
    Mat getRawDepth();
    Mat getForegroundColor();
    Mat getForegroundDepth();
};

int foo(int x);

#endif