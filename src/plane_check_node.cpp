//ROS
#include <ros/ros.h>
#include <ros/publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
//PCL
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
//msg
#include "gnss_cal/detect_planes.h"
#include "gnss_cal/single_plane.h"

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
    void initialize(){
    // get node name
    _name = ros::this_node::getName();

    // get publish
    _pub_inliers = _nh.advertise< sensor_msgs::PointCloud2 >("inliers",2);
    _pub_coefficient = _nh.advertise<gnss_cal::detect_planes>("planes_coefficient",1);
    _subs = _nh.subscribe("/livox/lidar",1,&planeFilter::pointCloudCb,this);
    
    // create color pallet
    createColors();

    ROS_INFO("%s: node initialized",_name.c_str());

    }
   
    void pointCloudCb(const sensor_msgs::PointCloud2::ConstPtr &msg){
     
     //convert to pcl point cloud
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg (new pcl::PointCloud<pcl::PointXY>);
     pcl::fromROSMsg(*msg,*cloud_msg);
     ROS_DEBUG("%s: new ponitcloud (%i,%i)(%zu)",_name.c_str(),cloud_msg->width,cloud_msg->height,
     cloud_msg->size());


    //filter cloud
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_msg);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits(0.001,10000);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pass.filter (*cloud);


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

    // To segment multi-planes in point cloud
    while(cloud->height*cloud->width>=_min_percentage*original_size/100){
        gnss_cal::detect_planes msg;
        // Fit a plane
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        // Check result
        if (inliers->indices.size() == 0)
            break;
        if (inliers->indices.size() >= 0.03 * original_size)
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
        // get boundary info
        // not write 

        // Extract inliers for the next iteration
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZ> cloudF;
        extract.filter(cloudF);
        cloud->swap(cloudF);
        n_planes++;
        
        //store in a ros message
        gnss_cal::single_plane submsg;
        submsg.index = n_planes;
        submsg.a = coefficients->values[0];
        submsg.b = coefficients->values[1];
        submsg.c = coefficients->values[2];
        submsg.d = coefficients->values[3];
        msg.Coeff.push_back(submsg);
        n_planes++;
    }
    // Publish coefficient
    _pub_coefficient.publish(msg);
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


private:
    ros::NodeHandle _nh;
    std::string _name;

    // Publishers
    ros::Publisher _pub_inliers;// Display inliers for each plane
    ros::Publisher _pub_coefficient;
    // Subscriber
    ros::Subscriber _subs;

    // Algorithm parameters
    double _min_percentage;
    double _max_distance;
    bool _color_pc_with_error;

    // Colors
    std::vector<Color> colors;
};

int main(int argc,char*argv[]){
    ros::init(argc,argv,"plane check");
    ros::NodeHandle nh("~");
    planeFilter pf (nh);
    pf.spin();
    return 0;
}