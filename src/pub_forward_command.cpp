#include <ros/ros.h>
#include <std_msgs/Bool.h>
using namespace std;
using namespace std_msgs;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "forward_command_pub");
  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<std_msgs::Bool> ("forward_command", 1);
  std_msgs::Bool track;
  track.data = true;

  ros::Rate r(10);

  while (n.ok())
  {
    pub.publish(track);
    ROS_INFO("forward command publisher~~~!");
    r.sleep();
  }
}

