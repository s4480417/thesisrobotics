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
#include <iostream>

#include <ros/ros.h>

using namespace cv;

void func(float& e) {
     e = (e - 500.0) / 1500.0;
     if (e <= 1e-3) {
         e = 1.0;
     }
     e = 255.0 * (1.0 - e);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "kinect_main");
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;
    pipeline = new libfreenect2::OpenGLPacketPipeline();
    if (freenect2.enumerateDevices() == 0) {
        std::cout << "No device connected!" << std::endl;
        return -1;
    }
    std::string serial = freenect2.getDefaultDeviceSerialNumber();
    if (pipeline) {
        dev = freenect2.openDevice(serial, pipeline);
    } else {
        dev = freenect2.openDevice(serial);
    }
    if (dev == 0) {
        std::cout << "Failure opening device" << std::endl;
        return -1;
    }
    libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
    libfreenect2::FrameMap frames;
    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);
    if (!dev->start()) {
        return -1;
    }
    std::cout << "Device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "Device firmware: " << dev->getFirmwareVersion() << std::endl;
    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4);
    libfreenect2::Frame registered(512, 424, 4);    

    ros::NodeHandle nh_registered;
    ros::NodeHandle nh_undistorted;
    image_transport::ImageTransport it_registered(nh_registered);
    image_transport::ImageTransport it_undistorted(nh_undistorted);
    image_transport::Publisher pub_registered = it_registered.advertise("kin/img_registered", 1);
    image_transport::Publisher pub_undistorted = it_undistorted.advertise("kin/img_undistorted", 1);

    for (;;) {
        if (!listener.waitForNewFrame(frames, 10*10000)) {
            std::cout << "Timeout!" << std::endl;
            return -1;
        }
        libfreenect2::Frame *color = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
        registration->apply(color, depth, &undistorted, &registered);

        Mat img_color = Mat(Size(color->width, color->height), CV_8UC4, color->data);
        Mat img_ir = Mat(Size(ir->width, ir->height), CV_8UC1, ir->data);
        Mat img_depth = Mat(Size(depth->width, depth->height), CV_32FC1, depth->data);
        
        Mat img_registered = Mat(Size(registered.width, registered.height), CV_8UC4, registered.data);
        Mat img_undistorted = Mat(Size(undistorted.width, undistorted.height), CV_32FC1, undistorted.data);
        cvtColor(img_registered, img_registered, CV_RGBA2RGB);
        
        Mat img_mask;
        
        compare(Scalar(0, 0, 0, 0), img_registered, img_mask, CMP_EQ);
        cvtColor(img_mask, img_mask, CV_RGBA2GRAY);

        std::for_each(img_undistorted.begin<float>(), img_undistorted.end<float>(), func);
        img_undistorted.convertTo(img_undistorted, CV_8UC1);

        Rect roi;
        roi.x = 0;
        roi.y = 50;
        roi.width = img_depth.size().width;
        roi.height = img_depth.size().height - 100;
        
        img_registered = img_registered(roi);
        img_undistorted = img_undistorted(roi);
        medianBlur(img_undistorted, img_undistorted, 5);
        Mat mask;
        threshold(img_undistorted, mask, 0, 255, THRESH_OTSU | THRESH_BINARY);
        Mat foreground_color;
        Mat foreground_depth;
        img_registered.copyTo(foreground_color, mask);
        img_undistorted.copyTo(foreground_depth, mask);
        sensor_msgs::ImagePtr msg_registered = cv_bridge::CvImage(std_msgs::Header(), "bgr8", foreground_color).toImageMsg();
        sensor_msgs::ImagePtr msg_undistorted = cv_bridge::CvImage(std_msgs::Header(), "mono8", foreground_depth).toImageMsg();

        pub_registered.publish(msg_registered);
        pub_undistorted.publish(msg_undistorted);
        ros::spinOnce();
        
        listener.release(frames);
        
        if (waitKey(1) == 'c') {
            imwrite("test.bmp", img_registered);
            std::cout << "screenshot!" << std::endl;
        }
    }
    dev->stop();
    dev->close();
    
    return 0;
}