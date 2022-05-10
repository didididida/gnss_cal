#include "sync_test.h"
#include <ros/ros.h>

using namespace lidar_localization;
int main(int argc,char*argv[]){

    ros::init(argc, argv, "sychronized_data");
    ros::NodeHandle nh;

    std::shared_ptr<sync_data>sync_data_ptr = std::make_shared<sync_data>(nh,"sync");

    ros::Rate rate(10);
    while(ros::ok()){
      ros::spinOnce();
      sync_data_ptr->run();
      rate.sleep();
    }

    return 0;
}
