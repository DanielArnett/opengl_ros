#include "simple_renderer_nodecore.h"

#include <chrono>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace opengl_ros;

SimpleRendererNode::SimpleRendererNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh_private), it_(nh)
{
    imagePublisher_  = it_.advertise("image_out", 1);
    //imageSubscriber_ = it_.subscribe("image_in" , 1, &SimpleRendererNode::imageCallback, this);

    int width, height;
    nh_.param<int>("width" , width , 640);
    nh_.param<int>("height", height, 480);

    std::string vertexShader, fragmentShader;
    nh_.param<std::string>("vertex_shader"  , vertexShader  , "");
    nh_.param<std::string>("fragment_shader", fragmentShader, "");

    renderer_ = std::make_unique<cgs::SimpleRenderer>(
        width, height, 
        vertexShader, fragmentShader
    );

    float x, y;
    nh_.param<float>("x"  , y  , 0);
    nh_.param<float>("y"   , x   , 0);
    renderer_->uniform("x"  , x);
    renderer_->uniform("y"  , y);
    
    output_.create(height, width, CV_8UC4);
    cv::Mat image;
    image.create(height, width, CV_8UC4);
    ros::Rate loop_rate(10);
    std_msgs::Header header;
    int count = 0;
    while(ros::ok()) {
        //cv_bridge::CvImageConstPtr cv_ptr;
        //try
        //{
        //    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGRA8);
        //}
        //catch (cv_bridge::Exception& e)
        //{
        //    ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
        //    return;
        //}
    
        //auto start = std::chrono::system_clock::now();
    
        cv::imwrite("/home/NEA.com/daniel.arnett/Pictures/input.png", image);
        //ROS_ERROR_STREAM(image.at<Vec3b>(0, 0)[0]);
        renderer_->render(output_, image);
    
        //ROS_DEBUG_STREAM(
        //    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - start).count() << "ns";
        //);
    
        //Publish
        cv_bridge::CvImage outImage;
        header.seq = count;
        header.stamp = ros::Time::now();
        outImage = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGRA8, output_);
        //outImage.header = cv_ptr->header;
        //outImage.encoding = cv_ptr->encoding;
        //outImage.image = output_;
        imagePublisher_.publish(outImage.toImageMsg());

        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
}

void SimpleRendererNode::run()
{
    ros::spin();
}