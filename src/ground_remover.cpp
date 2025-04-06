#include "ground_remover.hpp"

GroundRemover::GroundRemover()
    : Node("ground_remover")
{
  // Inizializza parametri, subscriber, publisher, ecc.
  RCLCPP_INFO(this->get_logger(), "Ground_remover node has started.");

  // parametri default
  this->declare_parameter("distance_threshold", 0.05);
  this->declare_parameter("max_iterations", 100);

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
  
  // filtrare la nuvola di punti a livello pavimento
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");    // dipende da come è orientata la tua nuvola
  pass.setFilterLimits(-1.0,1.5); // Limita la zona in cui si cerca il piano
  pass.filter(*cloud);
  
  // ridurre la densità della nuvola di punti
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(0.05f, 0.05f, 0.05f); // Riduce la densità della nuvola di punti
  vg.filter(*cloud);

  // Parametri di segmentazione
  this->get_parameter("distance_threshold", distance_threshold_);
  this->get_parameter("max_iterations", max_iterations_);

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
    RCLCPP_WARN(this->get_logger(), "Nessun piano trovato!");
    return;
  }

  // estrazione degli inliers (piano) e degli outliers (non piano)
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);

  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);

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
