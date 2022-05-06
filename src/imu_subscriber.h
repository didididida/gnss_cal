#ifndef IMU_SUBSCRIBER
#define IMU_SUBSCRIBER

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>

namespace lidar_localization{

class IMUData {
  public:
    struct LinearAcceleration {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };

    struct AngularVelocity {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };

    class Orientation {
      public:
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double w = 0.0;
      
       public:
        void Normlize() {
          double norm = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0) + pow(w, 2.0));
          x /= norm;
          y /= norm;
          z /= norm;
          w /= norm;
        }
    };

    double time = 0.0;
    LinearAcceleration linear_acceleration;
    AngularVelocity angular_velocity;
    Orientation orientation;
  
   public:
    Eigen::Matrix3f GetOrientationMatrix();
    static bool SyncData(std::deque<IMUData>& UnsyncedData, std::deque<IMUData>& SyncedData, double sync_time);
};

class IMUSubscriber{
    public:
        IMUSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
        IMUSubscriber() = default;
        void ParseData(std::deque<IMUData>& deque_imu_data);
        
    private:
        void msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);

    private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<IMUData> new_imu_data_;

    std::mutex buff_mutex_; 
 
};
}
#endif