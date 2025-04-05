#include "ground_remover.hpp"
#include <iostream>
#include <pcl/filters/passthrough.h>
// #include <pcl/features/normal_3d_omp.h> calcolo delle normali in parallelo

GroundRemover::GroundRemover()
    : Node("ground_remover")
{
  // Inizializza parametri, subscriber, publisher, ecc.
  RCLCPP_INFO(this->get_logger(), "Ground_remover node has started.");

  // parametri default
  this->declare_parameter("distance_threshold", 0.05);
  this->declare_parameter("max_iterations", 50);

  // Inizializzazione subscriber e publisher
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar_points", 10, std::bind(&GroundRemover::filter, this, std::placeholders::_1));

  publisher_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground", 10);
  publisher_non_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/not_ground", 10);
}

void GroundRemover::filter(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Convertire da PointCloud2 a PCL
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  // filtro i punti che sono altezza pavimento per ridurre tempo calcolo normali
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-0.5, 0.3); 
  pass.filter(*cloud);
  

  // Parametri di segmentazione
  this->get_parameter("distance_threshold", distance_threshold_);
  this->get_parameter("max_iterations", max_iterations_);

  RCLCPP_INFO(this->get_logger(), "Numero di punti nella nuvola: %zu", cloud->points.size());

  // calcolo delle normali con più thread
  // pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal;
  // normal.setNumberOfThreads(4); // numero di thread per il calcolo delle normali
  
  //calcolo delle normali
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal;
  normal.setInputCloud(cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  normal.setSearchMethod(tree);
  // normal.setRadiusSearch(0.05); // raggio di ricerca per le normali
  normal.setKSearch(10);        // numero di vicini per calcolare la normale
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  normal.compute(*cloud_normals);

  // creazione dell'oggetto per la segmentazione
  pcl::SACSegmentationFromNormals<pcl::PointXYZ,pcl::Normal> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(distance_threshold_);
  seg.setMaxIterations(max_iterations_);
  seg.setNormalDistanceWeight(0.1); // peso della normale
  seg.setInputCloud(cloud);
  seg.setInputNormals(cloud_normals);

  // Segmentazione del piano
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  seg.segment(*inliers, *coefficients);

  // Controllo se sono stati trovati inliers
  if (inliers->indices.empty())
  {
    RCLCPP_WARN(this->get_logger(), "Nessun piano trovato!");
    return;
  }

  // estrazione degli inliers (piano) e degli outliers (non piano)
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);

  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Estrazione del piano e del landscape
  extract.setNegative(false);
  extract.filter(*ground_cloud);
  extract.setNegative(true);
  extract.filter(*non_ground_cloud);

  // Convertire da PCL a PointCloud2 e pubblicare
  sensor_msgs::msg::PointCloud2 ground_msg;
  sensor_msgs::msg::PointCloud2 non_ground_msg;
  pcl::toROSMsg(*ground_cloud, ground_msg);
  pcl::toROSMsg(*non_ground_cloud, non_ground_msg);

  ground_msg.header = msg->header;
  non_ground_msg.header = msg->header;

  publisher_ground_->publish(ground_msg);
  publisher_non_ground_->publish(non_ground_msg);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GroundRemover>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
