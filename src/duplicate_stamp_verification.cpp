#include <ros/ros.h>
#include <sensor_msgs/Image.h>

ros::Time t;
unsigned int count = 0;
void imgCallback(const sensor_msgs::Image::ConstPtr& msg){
  if(!t.isValid()){
    t = msg->header.stamp;
  } else {
    if (msg->header.stamp == t){
      ROS_INFO("Double time stamp %d", ++count);
    }
    t = msg->header.stamp;
  }
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "dup_stamp_verify");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("sync/cam0/image_raw", 100, imgCallback);

  ros::spin();

  return 0;
}
