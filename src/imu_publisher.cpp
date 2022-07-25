#include "imu_publisher.h"

namespace lidar_localization{

imu_publisher::imu_publisher(ros::NodeHandle &nh, std::string topic_name,std::string frame_id,int buff_size)
:nh_(nh),frame_id(frame_id){
     publisher_=nh_.advertise<sensor_msgs::Imu>(topic_name,buff_size);
}

void imu_publisher::Publish(IMUData imu_data){
    sensor_msgs::Imu imu_pub;
    ros::Time ros_time(float(imu_data.time));
    imu_pub.header.stamp=ros_time;
    imu_pub.linear_acceleration.x=imu_data.linear_acceleration.x;
    imu_pub.linear_acceleration.y=imu_data.linear_acceleration.y;
    imu_pub.linear_acceleration.z=imu_data.linear_acceleration.z;

    imu_pub.angular_velocity.x=imu_data.angular_velocity.x;
    imu_pub.angular_velocity.y=imu_data.angular_velocity.y;
    imu_pub.angular_velocity.z=imu_data.angular_velocity.z;

    imu_pub.orientation.w=imu_data.orientation.w;
    imu_pub.orientation.x=imu_data.orientation.x;
    imu_pub.orientation.y=imu_data.orientation.y;
    imu_pub.orientation.z=imu_data.orientation.z;

    publisher_.publish(imu_pub);
}
}