#include "image_subscriber.h"
#include <ambf_server/RosComBase.h>
#include <sensor_msgs/Image.h>

using namespace std;

ImageSubscriber::ImageSubscriber(string a_namespace, string a_plugin){
    init(a_namespace, a_plugin);
}

ImageSubscriber::~ImageSubscriber(){
    m_imageSub.shutdown();
}

void ImageSubscriber::init(string a_namespace, string a_plugin){
    m_rosNode = afROSNode::getNode();
    m_imageSub = m_rosNode->subscribe<sensor_msgs::ImagePtr>("/snake_imager/image",1, &imageCallback, this);
    m_xray_image = std::make_shared<chai3d::cImage>(100,100);
}

void ImageSubscriber::imageCallback(sensor_msgs::ImagePtr img_msg){
    // m_xray_image
    m_xray_image->
    m_xray_image->setData(img_msg->data, (img_msg->width*img_msg->height) );
}
