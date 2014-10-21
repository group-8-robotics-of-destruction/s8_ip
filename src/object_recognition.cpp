#include <ros/ros.h>
#include <s8_common_node/Node.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>


// DEFINITIONS
#define HZ                  10
#define BUFFER_SIZE         10

#define NODE_NAME           		"s8_object_recognition_node"
#define TOPIC_POINT_CLOUD   		"/camera/depth/points"
#define TOPIC_EXTRACTED_OBJECTS		"/s8/extractedObjects"

typedef pcl::PointXYZ PointT;

class ObjectRecognition : public s8::Node {
	ros::Subscriber point_cloud_subscriber;
	ros::Publisher point_cloud_publisher;
public:
	ObjectRecognition()
	{
		//addParams();
		//printParams();
		point_cloud_subscriber = nh.subscribe(TOPIC_POINT_CLOUD, BUFFER_SIZE, &ObjectRecognition::point_cloud_callback, this);
		point_cloud_publisher  = nh.advertise<sensor_msgs::PointCloud2> (TOPIC_EXTRACTED_OBJECTS, BUFFER_SIZE);
	}
private:
	void point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& input)
	{
		pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
		/*
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		pcl::ModelCoefficients coefficients;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PassThrough<PointT> pass;
		pcl::ExtractIndices<PointT> extract;
		
		pcl::fromROSMsg (*input, *cloud);

		// Build a passthrough filter to remove spurious NaNs
		pass.setInputCloud (cloud);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (0.3, 1.0);
		pass.filter (*cloud_filtered);

		pass.setInputCloud (cloud_filtered);
		pass.setFilterFieldName ("x");
		pass.setFilterLimits (-0.1, 0.1);
		pass.filter (*cloud_filtered);

		pass.setInputCloud (cloud_filtered);
		pass.setFilterFieldName ("y");
		pass.setFilterLimits (0, 0.4);
		pass.filter (*cloud_filtered);

		sensor_msgs::PointCloud2 output;
  		pcl::toROSMsg(*cloud_filtered, output);
  		pub.publish (output);
  		*/
	}
};

int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);

    ObjectRecognition recognizer();
    ros::Rate loop_rate(HZ);

    while(ros::ok()) {
        //recognizer.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}