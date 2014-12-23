#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

class ImageStamper
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber image_sub_;
  ros::Subscriber trigger_sub_;

  image_transport::CameraPublisher image_pub_;
  
  std_msgs::Header trigger_;
  
public:
  ImageStamper() :
    it_(nh_)
  {
    std::string image_topic = nh_.resolveName("image");
    std::string image_topic_out = nh_.resolveName("image_out");
    image_sub_ = it_.subscribeCamera(image_topic, 1, &ImageStamper::imageCb, this);
    trigger_sub_ = nh_.subscribe<std_msgs::Header>("trigger", 10, &ImageStamper::triggerCB, this);
    image_pub_ = it_.advertiseCamera(image_topic_out, 10);
  }

  ~ImageStamper() {
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg) {
    sensor_msgs::Image img;
    
    img = *msg;
    
    img.header.stamp = trigger_.stamp;
    
    sensor_msgs::CameraInfo info;
    
    info = *info_msg;
    
    info.header.stamp = trigger_.stamp;
    
    image_pub_.publish(img, info);
  }
  
  void triggerCB(const std_msgs::HeaderConstPtr& msg) {
    trigger_ = *msg;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_stamper");
  ImageStamper ic;
  ros::spin();
  return 0;
}
