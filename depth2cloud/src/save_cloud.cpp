#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
    
static size_t counter = 0;
std::string topic_sub = "/stairs_cloud";
std::string path="/home/pc/rospakage/other/draw_box/src/depth2cloud/pcd/";
void SubscribePointCloud(const sensor_msgs::PointCloud2ConstPtr& lidar_message) {
    pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
    pcl::fromROSMsg(*lidar_message, point_cloud);
    
    counter++;
    std::string file_name = path+"point_cloud_" + std::to_string(counter) + ".pcd";
    pcl::io::savePCDFile(file_name, point_cloud);
}
    
int main(int argc, char **argv) {
    ros::init(argc, argv, "point_cloud_subscriber");
    ros::NodeHandle node_handle;
    ros::NodeHandle nh_("~");

    nh_.getParam("topic_sub",topic_sub);
    nh_.getParam("path",path);

    ros::Subscriber point_cloud_sub =  node_handle.subscribe(topic_sub, 1, SubscribePointCloud);
    ros::spin();
    
    return 0;
}