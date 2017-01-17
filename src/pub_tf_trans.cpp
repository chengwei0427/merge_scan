#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
//要使用TransformBroadcaster来发布转换消息，这样就必须包括tf包里面的tranform_broadcaster.h头文件

   tf::TransformBroadcaster broadcaster;
    tf::Transform transform;
     tf::Quaternion q;

   int main(int argc, char** argv){
     ros::init(argc, argv, "robot_tf_publisher");//初始化ros
     ros::NodeHandle n;
     
     double p_Origin_X;
     double p_Origin_Y;
     double p_Origin_Z;
     double p_Quaternion_R;
     double p_Quaternion_P;
     double p_Quaternion_Y;
     n.param("Origin_X", p_Origin_X, 0.0);
     n.param("Origin_Y", p_Origin_Y, 0.0);
     n.param("Origin_Z", p_Origin_Z, 0.0);
     n.param("Quaternion_R", p_Quaternion_R, 0.0);
     n.param("Quaternion_P", p_Quaternion_P, 0.0);
     n.param("Quaternion_Y", p_Quaternion_Y, 0.0);

    
     transform.setOrigin(tf::Vector3(p_Origin_X, p_Origin_Y, p_Origin_Z));
        q.setRPY(p_Quaternion_R, p_Quaternion_P, p_Quaternion_Y);
        transform.setRotation(q);

        ros::Rate r(0.1);

    while(n.ok()){
      broadcaster.sendTransform(
        tf::StampedTransform(
          transform, /* tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)) */
          ros::Time::now(),"point_link", "camera_link"));
      //用TransformBroadcaster来发布一个转换，需要五个参数：
      //第一个为旋转变换，由btQuaternion指定，其中btQuaternion包括pitch,roll,yaw
      //第二个为一个平移转换，由tVector3指定，对应robot base的laser的偏差为x轴偏差为10cm，z轴偏差为20cm;
      //第三个为发布一个时间戳，这里设置为ros::Time::now(),
      //第四个为父节点的名称,这里为base_link；
      //第五个为子节点的名称，这里为base_laser
      ROS_INFO("transform publisher~~~!");
      r.sleep();
      }
  }
