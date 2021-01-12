#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#include <ros/ros.h>

using namespace cv;

//[](float& e) { e = 1.0 - (e - 500.0) / 1500.0; }

void func(float& e) {
     e = 1.0 - (e - 500.0) / 1500.0;
}

int main(int argc, char** argv) {
    namedWindow("Color Filled", WINDOW_NORMAL);
    namedWindow("Depth Filled", WINDOW_NORMAL);

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
        cvtColor(img_registered, img_registered, CV_RGBA2RGB);
        Mat img_undistorted = Mat(Size(undistorted.width, undistorted.height), CV_32FC1, undistorted.data);
        
        Mat img_mask;
        
        compare(Scalar(0, 0, 0, 0), img_registered, img_mask, CMP_EQ);
        cvtColor(img_mask, img_mask, CV_RGBA2GRAY);

        std::for_each(img_undistorted.begin<float>(), img_undistorted.end<float>(), func);
        img_undistorted *= 255.0;
        img_undistorted.convertTo(img_undistorted, CV_8UC1);


        Mat img_color_filled;
        Mat img_depth_filled;
        inpaint(img_registered, img_mask, img_color_filled, 1.0, INPAINT_NS);
        inpaint(img_undistorted, img_mask, img_depth_filled, 1.0, INPAINT_TELEA);

        Rect roi;
        roi.x = 0;
        roi.y = 50;
        roi.width = img_depth.size().width;
        roi.height = img_depth.size().height - 100;

        imshow("Color Filled", img_color_filled(roi));
        imshow("Depth Filled", img_depth_filled(roi));
        
//        std::cout << img_depth_filled.size() << img_color_filled.size() << std::endl;
        
        listener.release(frames);
        
        waitKey(16);
    }
    dev->stop();
    dev->close();
    
    return 0;
}