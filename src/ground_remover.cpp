#include "ground_remover.hpp"

#include <iostream>
#include <chrono>

GroundRemover::GroundRemover()
    : Node("ground_remover")
{
  // Inizializza parametri, subscriber, publisher, ecc.
  RCLCPP_INFO(this->get_logger(), "Ground_remover node has started.");

  // parametri default
  this->declare_parameter("distance_threshold", 0.05);
  this->declare_parameter("max_iterations", 30);
  this->declare_parameter("min_filter", -0.5);
  this->declare_parameter("max_filter", 0.35);

  // Inizializzazione subscriber e publisher
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar_points", 10, std::bind(&GroundRemover::filter, this, std::placeholders::_1));

  publisher_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground", 10);
}

void GroundRemover::filter(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // tempo di esecuzione
  auto start = std::chrono::high_resolution_clock::now();
  
  // Convertire da PointCloud2 a PCL
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);
  
  // Parametri di segmentazione
  this->get_parameter("distance_threshold", distance_threshold_);
  this->get_parameter("max_iterations", max_iterations_);
  this->get_parameter("min_filter", min_filter_);
  this->get_parameter("max_filter", max_filter_);
  
  // filtrare la nuvola di punti a livello pavimento
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");    // dipende da come è orientata la tua nuvola
  pass.setFilterLimits(min_filter_, max_filter_); // Limita la zona in cui si cerca il piano
  pass.filter(*cloud);
  
  // ridurre la densità della nuvola di punti
  // ci mette uno sfracasso di tempo
  /*auto start = std::chrono::high_resolution_clock::now();

  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(0.05f, 0.05f, 0.05f); // Riduce la densità della nuvola di punti
  vg.filter(*cloud);

  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  std::cout << "Tempo di esecuzione: " << duration.count() << " microsecondi" << std::endl;*/

  // creazione dell'oggetto per la segmentazione
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(distance_threshold_);
  seg.setMaxIterations(max_iterations_);

  // Segmentazione del piano
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  // Controllo se sono stati trovati inliers
  if (inliers->indices.empty())
  {
    RCLCPP_WARN(this->get_logger(), "Errore durante segmentazione!");
    return;
  }

  // estrazione degli inliers (piano) e degli outliers (non piano)
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);

  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);


  extract.setNegative(false);
  extract.filter(*ground_cloud);

  // Convertire da PCL a PointCloud2 e pubblicare
  sensor_msgs::msg::PointCloud2 ground_msg;

  pcl::toROSMsg(*ground_cloud, ground_msg);


  ground_msg.header = msg->header;

  publisher_ground_->publish(ground_msg);


  // Calcolo del tempo di esecuzione
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  std::cout << "Tempo di esecuzione: " << duration.count() << " milisecondi" << std::endl;

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GroundRemover>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
