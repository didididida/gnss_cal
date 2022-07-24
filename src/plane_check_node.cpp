//ROS
#include <ros/ros.h>
#include <ros/publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include "gnss_cal/algorithmParametersConfig.h"

//PCL
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl_ros/impl/transforms.hpp>

//msg
#include "gnss_cal/detect_planes.h"
#include "gnss_cal/single_plane.h"
//tf
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <stdio.h>

class Color{
private:
    uint8_t r;
    uint8_t g;
    uint8_t b;

public:
    Color(uint8_t R,uint8_t G,uint8_t B):r(R),g(G),b(B){

    }

    void getColor(uint8_t &R,uint8_t &G,uint8_t &B){
        R = r;
        G = g;
        B = b;
    }
    void getColor(double &rd, double &gd, double &bd){
        rd = (double)r/255;
        gd = (double)g/255;
        bd = (double)b/255;
    }
    uint32_t getColor(){
        return ((uint32_t)r<<16|(uint32_t)g<<8|(uint32_t)b);
    }
};

class planeFilter{
public:
    planeFilter(ros::NodeHandle nh):_nh(nh){
         initialize();
    }
    
    void updateParameters(gnss_cal::algorithmParametersConfig& config, uint32_t level){
        threshold=config.threshold;
    }
    void initialize(){
    // get node name
    _name = ros::this_node::getName();

    // get publish
   
    //_subs = _nh.subscribe("/laser_cloud_map",1,&planeFilter::pointCloudCb,this);
    _sub_pointcloud = new message_filters::Subscriber<sensor_msgs::PointCloud2>(_nh,"/laser_cloud_map",100);
    //_sub_pointcloud = new message_filters::Subscriber<sensor_msgs::PointCloud2>(_nh,"/velodyne_cloud_registered",100);
    _sub_tf = new tf::MessageFilter<sensor_msgs::PointCloud2>(*_sub_pointcloud,tf_listener,"/camera_init",100);
    _sub_tf->registerCallback(boost::bind(&planeFilter::pointCloudCb, this, _1));

    _pub_inliers = _nh.advertise< sensor_msgs::PointCloud2 >("inliers",2);
    _pub_coefficient = _nh.advertise<gnss_cal::detect_planes>("planes_coefficient",1);
    
    //dynamic config
    ros::param::param<double>("~threshold",threshold,0);
    drCallback = boost::bind( &planeFilter::updateParameters, this, _1, _2);
    dRserver.setCallback(drCallback);
    

    // create color pallet
    createColors();

    ROS_INFO("%s: node initialized",_name.c_str());

    }
      
      
    
   
     
    void pointCloudCb(const sensor_msgs::PointCloud2::ConstPtr &msg){
     
    //convert to pcl point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg,*cloud_msg);

    ROS_INFO("%s: new ponitcloud (%i,%i)(%zu)",_name.c_str(),cloud_msg->width,cloud_msg->height,
    cloud_msg->size());
   
    //filter cloud
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_msg);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits(0.7,10000);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZ>);
    pass.filter (*cloud_1);
    
    pcl::PassThrough<pcl::PointXYZ> pass_2;
    pass_2.setInputCloud(cloud_1);
    pass_2.setFilterFieldName ("x");
    pass_2.setFilterLimits(-20,65);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2 (new pcl::PointCloud<pcl::PointXYZ>);
    pass_2.filter (*cloud_2);

    pcl::PassThrough<pcl::PointXYZ> pass_3;
    pass_3.setInputCloud(cloud_2);
    pass_3.setFilterFieldName ("y");
    pass_3.setFilterLimits(-10,10);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_3 (new pcl::PointCloud<pcl::PointXYZ>);
    pass_3.filter (*cloud_3);
 


    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_3);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_world (new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*cloud_world);

    
    tf::StampedTransform WorldToSensorTF;
    try
    {
        tf_listener.lookupTransform("aft_mapped","camera_init",msg->header.stamp,WorldToSensorTF);

    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
        return;
    }

    Eigen::Matrix4f worldToSensor;
    pcl_ros::transformAsMatrix(WorldToSensorTF,worldToSensor);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud_world,*cloud,worldToSensor);
    
    // Get segmentation ready
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE); //sansac to extract planes
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold(_max_distance);
    
    //indicator to identify the plane's size
    bool small_plane = true;

    // Create pointcloud to publish inliers
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pub(new pcl::PointCloud<pcl::PointXYZRGB>);
    int original_size(cloud->height*cloud->width);
    int n_planes(0);
    gnss_cal::detect_planes planes_msg;
    planes_msg.header.stamp.sec = msg->header.stamp.sec + 1651246016;
    planes_msg.header.stamp.nsec = msg->header.stamp.nsec;
    planes_msg.header.frame_id = "aft_mapped";


    // To segment multi-planes in point cloud
    while(cloud->height*cloud->width>=_min_percentage*original_size/100){
        
        // Fit a plane
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        // Check result
        if (inliers->indices.size() == 0)
            break;
        if (inliers->indices.size() >= threshold *0.01 * original_size)
             small_plane = false;
        else small_plane = true;
        
        //points in small planes just eliminate
        if(small_plane){
            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            extract.setNegative(true);
            pcl::PointCloud<pcl::PointXYZ> cloudF;
            extract.filter(cloudF);
            cloud->swap(cloudF);
            continue;
        }

        //color big planes
        for(int i=0;i<inliers->indices.size();++i){
            //get point
            pcl::PointXYZ pt = cloud->points[inliers->indices[i]];
            //copy to a new point cloud with color
            pcl::PointXYZRGB pt_color;

            pt_color.x = pt.x;
            pt_color.y = pt.y;
            pt_color.z = pt.z;

            uint32_t rgb;
            rgb = colors[n_planes].getColor();
            pt_color.rgb = *reinterpret_cast<float*>(&rgb);
            cloud_pub->points.push_back(pt_color);
        }
        
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud,inliers->indices,*plane_cloud);
        pcl::PointXYZ pmin;
        pcl::PointXYZ pmax;
        pcl::getMinMax3D(*plane_cloud,pmin,pmax);

        // Extract inliers for the next iteration
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZ> cloudF;
        extract.filter(cloudF);
        cloud->swap(cloudF);
    
        
        //store in a ros message
        gnss_cal::single_plane submsg;
        submsg.index = n_planes;
        submsg.a = coefficients->values[0];
        submsg.b = coefficients->values[1];
        submsg.c = coefficients->values[2];
        submsg.d = coefficients->values[3];
        submsg.z_max = pmax.z;
        submsg.z_min = pmin.z;
        planes_msg.Coeff.push_back(submsg);
        n_planes++;
    }
    // Publish coefficient
    _pub_coefficient.publish(planes_msg);
    // Publish points
    sensor_msgs::PointCloud2 cloud_publish;
    pcl::toROSMsg(*cloud_pub,cloud_publish);
    cloud_publish.header = msg->header;
    _pub_inliers.publish(cloud_publish);

    }

    void createColors(){
        uint8_t r = 0;
        uint8_t g = 0;
        uint8_t b = 0;
        for (int i=0;i<20;i++){
            while (r<70 && g < 70 && b < 70){
                r = rand()%(255);
                g = rand()%(255);
                b = rand()%(255);
            }
            Color c(r,g,b);
            r = 0;
            g = 0;
            b = 0;
            colors.push_back(c);
        }
    }
    void spin(){
        ros::spin();
    }
    
    ~planeFilter(){
        delete _sub_pointcloud;
        delete _sub_tf;
    }

private:
    ros::NodeHandle _nh;
    std::string _name;

    // Publishers
    ros::Publisher _pub_inliers;// Display inliers for each plane
    ros::Publisher _pub_coefficient;
    // Subscriber
    //ros::Subscriber _subs;
    message_filters::Subscriber<sensor_msgs::PointCloud2>*_sub_pointcloud;
    tf::MessageFilter<sensor_msgs::PointCloud2>*_sub_tf;
    tf::TransformListener tf_listener;
    


    // Algorithm parameters
    double _min_percentage = 5;
    double _max_distance = 0.27;

    // Colors
    std::vector<Color> colors;
    
    //dynamic config
    double threshold;
     dynamic_reconfigure::Server<gnss_cal::algorithmParametersConfig> dRserver;
    dynamic_reconfigure::Server<gnss_cal::algorithmParametersConfig>::CallbackType drCallback;
};

int main(int argc,char*argv[]){
    ros::init(argc,argv,"planecheck");
    ros::NodeHandle nh("~");
    planeFilter pf (nh);
    pf.spin();
    return 0;
}
