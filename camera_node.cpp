//=============================================================================
// This code is part of a ROS node that will connect to PointGrey cameras
//=============================================================================
//=============================================================================
// Author: Joonas Melin
//=============================================================================



#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <FlyCapture2.h>
using namespace FlyCapture2;

#include <iostream>
#include <exception>
#include <boost/thread/thread.hpp>

#include "camera.h"
#include "stringException.h"

//This is an helper function to print the camera information
//Implementation from ptgrey example 
void PrintBuildInfo()
{
    FC2Version fc2Version;
    Utilities::GetLibraryVersion( &fc2Version );
    char version[128];
    sprintf(
                version,
                "FlyCapture2 library version: %d.%d.%d.%d\n",
                fc2Version.major, fc2Version.minor, fc2Version.type, 
                fc2Version.build );

    std::cout << version << std::endl;

    char timeStamp[512];
    sprintf( timeStamp, "Application build date: %s %s\n\n", 
        __DATE__, __TIME__ );

    std::cout << timeStamp << std::endl;
}


//The main function
int main(int argc, char** argv)
{
    //Thread handling the reading in case of no trigger signal given
    boost::thread* publisher_thread;

    //Initializing the node
    ros::init(argc, argv, ("ptgrey"));

    //parameter handling, the private node handler for parameters
    ros::NodeHandle nhS("~/ptgrey_camera");


    //Getting the parameters
    int serial_number = 0;
    nhS.getParam("serial", serial_number);

    std::string name = "default";
    nhS.getParam("name", name);

    bool trigOn = false;
    nhS.getParam("extern_trigger", trigOn);

    int imWidth = 1280;
    nhS.getParam("width", imWidth);

    int imHeight = 1024;
    nhS.getParam("height", imHeight);

    int imFps = 50;
    nhS.getParam("fps", imFps);

    //Determining the desired pixelformat
    std::string imColor = "MONO8";
    nhS.getParam("color", imColor);
    FlyCapture2::PixelFormat color = FlyCapture2::PIXEL_FORMAT_MONO8;
    if(imColor.compare("MONO8") == 0) color = FlyCapture2::PIXEL_FORMAT_MONO8;
    else if(imColor.compare("MONO12") == 0) color = FlyCapture2::PIXEL_FORMAT_MONO12;
    else if(imColor.compare("MONO16") == 0) color = FlyCapture2::PIXEL_FORMAT_MONO16;
    else if(imColor.compare("RAW8") == 0) color = FlyCapture2::PIXEL_FORMAT_RAW8;
    else if(imColor.compare("RAW12") == 0) color = FlyCapture2::PIXEL_FORMAT_RAW12;
    else if(imColor.compare("RAW16") == 0) color = FlyCapture2::PIXEL_FORMAT_RAW16;
    else if(imColor.compare("RGB8") == 0) color = FlyCapture2::PIXEL_FORMAT_RGB8;
    else if(imColor.compare("RGB16") == 0) color = FlyCapture2::PIXEL_FORMAT_RGB16;

    std::string imMode = "MODE_0";
    nhS.getParam("mode", imMode);
    FlyCapture2::Mode mode = FlyCapture2::MODE_0;
    if(imMode.compare("MODE_0") == 0) mode = FlyCapture2::MODE_0;
    else if(imMode.compare("MODE_1") == 0) mode = FlyCapture2::MODE_1;

    int bufferTimeout = 50;
    nhS.getParam("buffer_timeout", bufferTimeout);

    std::string trigTopic = "/copter2/stereo_cam/stamp";
    nhS.getParam("trig_topic", trigTopic);

    std::string outTopic = "/copter2/camera/";
    nhS.getParam("topic_base", outTopic);

    int openDelay = 0;
    nhS.getParam("open_delay", openDelay);

    bool autoExp = true;
    nhS.getParam("auto_exposure", autoExp);

    std::string frame_id = "camera";
    nhS.getParam("frame_id", frame_id);


    //Making new node handles for image transport
    ros::NodeHandle nh( outTopic + name);
    image_transport::ImageTransport* it = 
        new image_transport::ImageTransport(nh);

    //Advertising a new topic for the images
    std::string topic = "image_raw";
    image_transport::CameraPublisher pub = it->advertiseCamera(topic, 1);

    //Advertising topic for the over exposed precentage of the pixels
    ros::Publisher histPub = nh.advertise<std_msgs::Float32>(
                outTopic + name + "/over_percent", 1000);

    //Making camera infomanager for storing the
    //camera info(calibration) in a ROS friendly way
    boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo(
        new camera_info_manager::CameraInfoManager(nh));

    //Container for the camera (Can also be  vector for multiple cameras)
    roscam::Camera* cams;
    cams = new roscam::Camera(pub, histPub, nh, name, cinfo, frame_id);


    ROS_INFO("Checking for trigger signal");
    ros::Subscriber stampSub;
    //Checking if we are getting accurate triggering stamps
    //or if we just need to wing it
    if(trigOn){
        ROS_INFO("Binding to trig signal,Publishing image 
            each time trigger message is received");
        //Subscribing to a trigger topic, that is assigned to a callback which 
        //reads the image. This is done to correctly timestamp the images
        stampSub = nh.subscribe(trigTopic,1, 
            &roscam::Camera::captureCallbackNormal, cams);
    }
    else{
        //In case we are not receiving accurate triggering information, we just
        //fork the execution to another thread
        ROS_INFO("Creating new thread for reading stuff");
        ROS_WARN("External trigger not provided, estimating the timing");
        publisher_thread = new boost::thread(boost::bind(
            &roscam::Camera::captureCallbackLoop, cams));
    }

    //Printing debug information
    PrintBuildInfo();

    ROS_INFO("Camera serial: %i", serial_number);
    ROS_INFO("Image dimensions (WidthxHeight): %i x %i", imWidth, imHeight);
    ROS_INFO("External trigger: %i", trigOn);
    ROS_INFO("Trigger topic: %s", trigTopic.c_str());
    ROS_INFO("Color: %s", imColor.c_str());
    ROS_INFO("FPS: %i", imFps);
    ROS_INFO("Buffer_timeout: %i", bufferTimeout);
    ROS_INFO("Mode: %s", imMode.c_str());
    ROS_INFO("Open_delay: %i", openDelay);
    ROS_INFO("Auto_exposure: %i", autoExp);

    //Reserving buffers for the images
    //@ todo needs to be vector in case several cameras are desired
    unsigned char* pBuffers; //buffers for the images

    ROS_INFO("Waiting for open_delay to pass, to make sure that we dont overload
     USB3 the controllers in case of several simultaneous nodes");
    boost::this_thread::sleep( boost::posix_time::seconds(openDelay) );

    //Trying to open the cameras
    ROS_INFO("Opening camera %i", serial_number);
    try{
        cams->openCameras(pBuffers, serial_number, imWidth, imHeight, trigOn,
                          color, mode, imFps, bufferTimeout, autoExp);
    }catch(StringException& caught){
        std::cout << "Exception: " << caught.what() << std::endl;
    }


    ROS_INFO("Capturing images from %i", serial_number);

    //Waiting for the callbacks to happen
    ros::spin();

    //Now we have been closed down, doing cleanup
    delete publisher_thread;
    cams->stopCameras();

    delete pBuffers;
    delete cams;

    return 0;
}
