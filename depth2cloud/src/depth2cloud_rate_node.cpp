#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <string.h>

// PCL 库
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>

// 定义点云类型
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud; 

using namespace std;
//namespace enc = sensor_msgs::image_encodings;

//ros
ros::Subscriber sub_depth;
ros::Publisher pub_cloud;

string topic_sub_depth="/camera/depth/image_rect_raw";
string topic_pub_pcl="/cloud_dpc";
string pointcloudFrame = "/camera_depth_optical_frame";
bool be_32FC=false;
// 相机内参
double camera_factor = 1000;
double camera_cx = 422.4590759277344;
double camera_cy = 240.69679260253906;
double camera_fx = 431.00274658203125;
double camera_fy = 431.00274658203125;

// 全局变量：图像矩阵和点云
cv_bridge::CvImagePtr  depth_ptr;
cv::Mat depth_pic;

void depth_Callback(const sensor_msgs::ImageConstPtr& depth_msg)
{
  try
  {
	if(be_32FC){
		depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1); //gazebo
	}else{
		depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1); //realsense
	}
    depth_pic = depth_ptr->image;
    // cv::GaussianBlur(depth_pic, depth_pic, cv::Size(5, 5), 0, 0);
    PointCloud::Ptr cloud ( new PointCloud );
    sensor_msgs::PointCloud2 pub_pointcloud;
    for (int m = 0; m < depth_pic.rows; m++){
      for (int n = 0; n < depth_pic.cols; n++){
        // 获取深度图中(m,n)处的值
		double d;
		if(be_32FC){
			d = (double)depth_pic.ptr<float>(m)[n]; //gazebo
		}else{
			d = (double)depth_pic.ptr<ushort>(m)[n];//realsense
		}
        //cout<<d<<endl;
		d = d / camera_factor;
        // d 可能没有值，若如此，跳过此点
        if (d >=3||d<=0.5)
          continue;
        pcl::PointXYZRGB p;

        // 计算这个点的空间坐标
        p.z = d;
        p.x = (n - camera_cx) * p.z / camera_fx;
        p.y = (m - camera_cy) * p.z / camera_fy;

        // 从rgb图像中获取它的颜色
        // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
        p.b = 255;
        p.g = 255;
        p.r = 255;
        // 把p加入到点云中
        cloud->points.push_back( p );
      }
    }
    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    // ROS_INFO("point cloud size = %d",cloud->width);
    cloud->is_dense = false;
    pcl::toROSMsg(*cloud,pub_pointcloud);
    pub_pointcloud.header.frame_id = pointcloudFrame;
    //pub_pointcloud.header.frame_id = depth_ptr->header.frame_id;
    pub_pointcloud.header.stamp = depth_ptr->header.stamp;
    pub_cloud.publish(pub_pointcloud);
    cloud->points.clear();
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'mono16'.", depth_msg->encoding.c_str());
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "depth2cloud");
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");
  nh_.getParam("topicPubPcl", topic_pub_pcl);
  nh_.getParam("topicSubDepth", topic_sub_depth);
  nh_.getParam("pointcloudFrame", pointcloudFrame);
  nh_.getParam("camera_factor", camera_factor);
  nh_.getParam("camera_cx", camera_cx);
  nh_.getParam("camera_cy", camera_cy);
  nh_.getParam("camera_fx", camera_fx);
  nh_.getParam("camera_fy", camera_fy);
  nh_.getParam("be_32FC", be_32FC);
  sub_depth = nh.subscribe(topic_sub_depth, 1, depth_Callback);
  pub_cloud = nh.advertise<sensor_msgs::PointCloud2>(topic_pub_pcl, 1);
  ros::spin();
}
