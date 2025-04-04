// ground_remover.hpp
#ifndef GROUND_REMOVER_HPP_
#define GROUND_REMOVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

class GroundRemover : public rclcpp::Node
{
public:
  explicit GroundRemover();
  
private:
  // Callback per ricevere la pointcloud
  void filter(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  
  // Subscriber e publisher
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_ground_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_non_ground_;
  
  // Parametri per la segmentazione (configurabili in modo dinamico)
  double distance_threshold_;
  int max_iterations_;
};

#endif  // GROUND_REMOVER_HPP_
