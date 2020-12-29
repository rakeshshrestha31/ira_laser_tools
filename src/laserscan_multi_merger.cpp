#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <string.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <laser_geometry/laser_geometry.hpp>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>

using namespace std;
using namespace pcl;
// using namespace laserscan_multi_merger;

class LaserscanMerger : public rclcpp::Node
{
public:
    LaserscanMerger();

    void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan, std::string topic);
    void pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud);
    // void reconfigureCallback(laserscan_multi_mergerConfig &config, uint32_t level);

private:
    laser_geometry::LaserProjection projector_;
    std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_publisher_;
    vector< rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr > scan_subscribers;
    vector<bool> clouds_modified;

    vector<pcl::PCLPointCloud2> clouds;
    vector<string> input_topics;

    void laserscan_topic_parser();

    double angle_min;
    double angle_max;
    double angle_increment;
    double time_increment;
    double scan_time;
    double range_min;
    double range_max;

    string destination_frame;
    string cloud_destination_topic;
    string scan_destination_topic;
    string laserscan_topics;
};

// void LaserscanMerger::reconfigureCallback(laserscan_multi_mergerConfig &config, uint32_t level)
// {
// 	this->angle_min = config.angle_min;
// 	this->angle_max = config.angle_max;
// 	this->angle_increment = config.angle_increment;
// 	this->time_increment = config.time_increment;
// 	this->scan_time = config.scan_time;
// 	this->range_min = config.range_min;
// 	this->range_max = config.range_max;
// }

void LaserscanMerger::laserscan_topic_parser()
{
  // TODO: get all topics
	// LaserScan topics to subscribe
	// rclcpp::master::V_TopicInfo topics;
	// rclcpp::master::getTopics(topics);

    istringstream iss(laserscan_topics);
	vector<string> tokens;
	copy(istream_iterator<string>(iss), istream_iterator<string>(), back_inserter<vector<string> >(tokens));
	vector<string> tmp_input_topics;

  // TODO: filter topics
  tmp_input_topics = tokens;

	// for(int i=0;i<tokens.size();++i)
	// {
  //   for(int j=0;j<topics.size();++j)
	// 	{
	// 		if( (tokens[i].compare(topics[j].name) == 0) && (topics[j].datatype.compare("sensor_msgs/msg/LaserScan") == 0) )
	// 		{
	// 			tmp_input_topics.push_back(topics[j].name);
	// 		}
	// 	}
	// }

	sort(tmp_input_topics.begin(),tmp_input_topics.end());
	std::vector<string>::iterator last = std::unique(tmp_input_topics.begin(), tmp_input_topics.end());
	tmp_input_topics.erase(last, tmp_input_topics.end());


	// Do not re-subscribe if the topics are the same
	if( (tmp_input_topics.size() != input_topics.size()) || !equal(tmp_input_topics.begin(),tmp_input_topics.end(),input_topics.begin()))
	{

    // Hopefully the subscribers use RAII and shutdown after reallocation
    // TODO: check if reallocation will shutdown older subscribers
		// Unsubscribe from previous topics
		// for(int i=0; i<scan_subscribers.size(); ++i)
		// 	scan_subscribers[i].shutdown();

		input_topics = tmp_input_topics;
		if(input_topics.size() > 0)
		{
            scan_subscribers.resize(input_topics.size());
			clouds_modified.resize(input_topics.size());
			clouds.resize(input_topics.size());
      RCLCPP_INFO(this->get_logger(),"Subscribing to topics\t%ld",scan_subscribers.size());
			for(int i=0; i<input_topics.size(); ++i)
			{
        scan_subscribers[i] = this->create_subscription<sensor_msgs::msg::LaserScan>(
            input_topics[i].c_str(), 1,
            // need explicit casting from std::Bind to specific typed std::function
            (std::function<void(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)>)
              std::bind(&LaserscanMerger::scanCallback, this, std::placeholders::_1, input_topics[i])
        );
				clouds_modified[i] = false;
				cout << input_topics[i] << " ";
			}
		}
		else
    {
      RCLCPP_INFO(this->get_logger(),"Not subscribed to any topic.");
    }
	}
}

LaserscanMerger::LaserscanMerger()
  : Node("laser_multi_merger")
{
	// rclcpp::Node nh("~");

  this->declare_parameter<std::string>("destination_frame", "cart_frame");
  this->declare_parameter<std::string>("cloud_destination_topic", "/merged_cloud");
  this->declare_parameter<std::string>("scan_destination_topic", "/scan_multi");
  this->declare_parameter<std::string>("laserscan_topics", "");
  this->declare_parameter<double>("angle_min", -2.36);
  this->declare_parameter<double>("angle_max", 2.36);
  this->declare_parameter<double>("angle_increment", 0.0058);
  this->declare_parameter<double>("scan_time", 0.0333333);
  this->declare_parameter<double>("range_min", 0.45);
  this->declare_parameter<double>("range_max", 25.0);

  // TODO: setup callback for dynamically reconfigurable parameters
  this->get_parameter("destination_frame", destination_frame);
  this->get_parameter("cloud_destination_topic", cloud_destination_topic);
  this->get_parameter("scan_destination_topic", scan_destination_topic);
  this->get_parameter("laserscan_topics", laserscan_topics);
  this->get_parameter("angle_min", angle_min);
  this->get_parameter("angle_max", angle_max);
  this->get_parameter("angle_increment", angle_increment);
  this->get_parameter("scan_time", scan_time);
  this->get_parameter("range_min", range_min);
  this->get_parameter("range_max", range_max);

    this->laserscan_topic_parser();

	// point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2> (cloud_destination_topic.c_str(), 1, false);
	// laser_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan> (scan_destination_topic.c_str(), 1, false);
	point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2> (cloud_destination_topic.c_str(), 1);
	laser_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan> (scan_destination_topic.c_str(), 1);

  tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
}

void LaserscanMerger::scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan, std::string topic)
{
  sensor_msgs::msg::PointCloud2 tmpCloud1,tmpCloud2;
	// sensor_msgs::msg::PointCloud2 tmpCloud3;

    // Verify that TF knows how to transform from the received scan to the destination scan frame
  // No need to wait for transform with tf2
	// tfListener_.waitForTransform(scan->header.frame_id.c_str(), destination_frame.c_str(), scan->header.stamp, rclcpp::Duration(1));

  projector_.transformLaserScanToPointCloud(scan->header.frame_id, *scan, tmpCloud1, *tfBuffer_, laser_geometry::channel_option::Distance);

  geometry_msgs::msg::TransformStamped transform;
	try
	{
		transform = tfBuffer_->lookupTransform(destination_frame, tmpCloud1.header.frame_id, rclcpp::Time(0));
	}
  catch (tf2::TransformException &ex)
  {
    RCLCPP_ERROR(this->get_logger(), "%s",ex.what());
    return;
  }

  tf2::doTransform(tmpCloud1, tmpCloud2, transform);

	for(int i=0; i<input_topics.size(); ++i)
	{
		if(topic.compare(input_topics[i]) == 0)
		{
      // sensor_msgs::convertPointCloudToPointCloud2(tmpCloud2,tmpCloud3);
			pcl_conversions::toPCL(tmpCloud2, clouds[i]);
			clouds_modified[i] = true;
		}
	}	

    // Count how many scans we have
	int totalClouds = 0;
	for(int i=0; i<clouds_modified.size(); ++i)
		if(clouds_modified[i])
			++totalClouds;

    // Go ahead only if all subscribed scans have arrived
	if(totalClouds == clouds_modified.size())
	{
		pcl::PCLPointCloud2 merged_cloud = clouds[0];
		clouds_modified[0] = false;

		for(int i=1; i<clouds_modified.size(); ++i)
		{
			pcl::concatenate(merged_cloud, clouds[i], merged_cloud);
			clouds_modified[i] = false;
		}
	
    // TODO: make the point cloud topic publishable as well
		// point_cloud_publisher_->publish(merged_cloud);

		Eigen::MatrixXf points;
		getPointCloudAsEigen(merged_cloud,points);

		pointcloud_to_laserscan(points, &merged_cloud);
	}
}

void LaserscanMerger::pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud)
{
  sensor_msgs::msg::LaserScan::SharedPtr output = std::make_shared<sensor_msgs::msg::LaserScan>();
	output->header = pcl_conversions::fromPCL(merged_cloud->header);
	output->header.frame_id = destination_frame.c_str();
  output->header.stamp = this->now();  //fixes #265
	output->angle_min = this->angle_min;
	output->angle_max = this->angle_max;
	output->angle_increment = this->angle_increment;
	output->time_increment = this->time_increment;
	output->scan_time = this->scan_time;
	output->range_min = this->range_min;
	output->range_max = this->range_max;

	uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
	output->ranges.assign(ranges_size, output->range_max + 1.0);

	for(int i=0; i<points.cols(); i++)
	{
		const float &x = points(0,i);
		const float &y = points(1,i);
		const float &z = points(2,i);

		if ( std::isnan(x) || std::isnan(y) || std::isnan(z) )
		{
			RCLCPP_DEBUG(this->get_logger(), "rejected for nan in point(%f, %f, %f)\n", x, y, z);
			continue;
		}

		double range_sq = y*y+x*x;
		double range_min_sq_ = output->range_min * output->range_min;
		if (range_sq < range_min_sq_) {
			RCLCPP_DEBUG(this->get_logger(), "rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range_sq, range_min_sq_, x, y, z);
			continue;
		}

		double angle = atan2(y, x);
		if (angle < output->angle_min || angle > output->angle_max)
		{
			RCLCPP_DEBUG(this->get_logger(), "rejected for angle %f not in range (%f, %f)\n", angle, output->angle_min, output->angle_max);
			continue;
		}
		int index = (angle - output->angle_min) / output->angle_increment;


		if (output->ranges[index] * output->ranges[index] > range_sq)
			output->ranges[index] = sqrt(range_sq);
	}

	laser_scan_publisher_->publish(*output);
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);

  LaserscanMerger::SharedPtr _laser_merger = std::make_shared<LaserscanMerger>();

    // dynamic_reconfigure::Server<laserscan_multi_mergerConfig> server;
    // dynamic_reconfigure::Server<laserscan_multi_mergerConfig>::CallbackType f;

    // f = std::bind(&LaserscanMerger::reconfigureCallback,&_laser_merger, _1, _2);
    // server.setCallback(f);

	rclcpp::spin(_laser_merger);
  rclcpp::shutdown();

	return 0;
}
