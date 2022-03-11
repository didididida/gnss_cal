#include <ros/ros.h>
#include <ros/publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>

static int frame = 0; // index for frame
static pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

class accumPoint{
 private:
 ros::NodeHandle nh;
 sensor_msgs::PointCloud2 raw_points;
 ros::Subscriber sub;
 ros::Publisher pub;

 public:
 accumPoint(){
    sub = nh.subscribe("/livox/lidar",3,&accumPoint::callback,this);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/accumu_point",1);
 }
 void callback(const sensor_msgs::PointCloud2::ConstPtr &msg);
 void getPoint_cb(const sensor_msgs::PointCloud2::ConstPtr &livox_msg);
 void spin();
};
void accumPoint::spin(){
    ros::spin();
}
void accumPoint::callback(const sensor_msgs::PointCloud2::ConstPtr &msg){
    if(frame<3){
    pcl::PointCloud<pcl::PointXYZ>::Ptr transfer_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg,*transfer_cloud);
    *tmp_cloud+=*transfer_cloud;
    frame++;
    }else{
    pcl::toROSMsg(*tmp_cloud,raw_points);
    frame = 0;
    tmp_cloud->clear();
    }
}
int main(int argc,char*argv[]){
    ros::init(argc,argv,"accumulate_point");
    accumPoint accumpoint;
    accumpoint.spin();
    return 0;
}