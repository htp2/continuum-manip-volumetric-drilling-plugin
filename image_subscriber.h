#ifndef IMAGE_SUBSCRIBER_H
#define IMAGE_SUBSCRIBER_H

#include "ros/ros.h"
#include <string>
#include <sensor_msgs/Image.h>
#include <afFramework.h>


class ImageSubscriber{
public:
    ImageSubscriber(std::string a_namespace, std::string a_plugin);
    ~ImageSubscriber();
    void init(std::string a_namespace, std::string a_plugin);
    ros::NodeHandle* m_rosNode;
    std::shared_ptr<chai3d::cImage> m_xray_image;

private:
    void imageCallback(sensor_msgs::ImagePtr img_msg);
    ros::Subscriber m_imageSub;
};

#endif // IMAGE_SUBSCRIBER_H
