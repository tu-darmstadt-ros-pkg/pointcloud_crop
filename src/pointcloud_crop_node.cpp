#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>

class PointCloudCrop
{
public:
  PointCloudCrop(ros::NodeHandle& nh)
  {
    if (!loadParam(nh, "min_x", min_x_) ||
        !loadParam(nh, "max_x", max_x_) ||
        !loadParam(nh, "min_y", min_y_) ||
        !loadParam(nh, "max_y", max_y_) ||
        !loadParam(nh, "min_z", min_z_) ||
        !loadParam(nh, "max_z", max_z_) ||
        !loadParam(nh, "negative", negative_))
    {
      return;
    }

    crop_box_.setNegative(negative_);
    crop_box_.setMin(Eigen::Vector4f(min_x_, min_y_, min_z_, 1));
    crop_box_.setMax(Eigen::Vector4f(max_x_, max_y_, max_z_, 1));

    sub_ = nh.subscribe<sensor_msgs::PointCloud2>("input", 1, &PointCloudCrop::callback, this);
    pub_ = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
  }

private:
  template <typename T>
  bool loadParam(ros::NodeHandle& nh, const std::string& param_name, T& param_var)
  {
    if (!nh.getParam(param_name, param_var))
    {
      ROS_ERROR("Failed to get param '%s'", param_name.c_str());
      return false;
    }
    return true;
  }

  void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);
    crop_box_.setInputCloud(cloud);
    crop_box_.filter(*cloud);
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud, output_msg);
    output_msg.header = msg->header;
    pub_.publish(output_msg);
  }

  pcl::CropBox<pcl::PointXYZI> crop_box_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  std::string input_topic_;
  std::string output_topic_;
  double min_x_, max_x_, min_y_, max_y_, min_z_, max_z_;
  bool negative_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_crop_node");
  ros::NodeHandle nh("~");
  PointCloudCrop pointcloud_crop(nh);
  ros::spin();
  return 0;
}
