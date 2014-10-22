#include <ros/ros.h>
#include <s8_common_node/Node.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
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

// PARAMETERS
#define PARAM_FILTER_X_NAME				"filter_x"
#define PARAM_FILTER_X_DEFAULT			0.1
#define PARAM_FILTER_Z_NAME				"filter_z"
#define PARAM_FILTER_Z_DEFAULT			1.0
#define PARAM_FILTER_Y_NAME				"filter_y"
#define PARAM_FILTER_Y_DEFAULT			0.4
#define PARAM_SEG_DISTANCE_NAME			"seg_distance"
#define PARAM_SEG_DISTANCE_DEFAULT		0.01
#define PARAM_SEG_PERCENTAGE_NAME		"seg_percentage"
#define PARAM_SEG_PERCENTAGE_DEFAULT	0.2

typedef pcl::PointXYZ PointT;

class ObjectRecognition : public s8::Node 
{
	const int hz;

	ros::Subscriber point_cloud_subscriber;
	ros::Publisher point_cloud_publisher;

	double filter_x, filter_y, filter_z;
	double seg_distance, seg_percentage;

	sensor_msgs::PointCloud2 input, output, cloud_msg;
	//pcl::PointCloud<PointT>::Ptr cloud_filtered;

public:
	ObjectRecognition(int hz) : hz(hz)
	{
		add_params();
		//printParams();
		//pcl::PointCloud<PointT>::Ptr tmp (new pcl::PointCloud<PointT>);
		//cloud_filtered = tmp;

		point_cloud_subscriber = nh.subscribe(TOPIC_POINT_CLOUD, BUFFER_SIZE, &ObjectRecognition::point_cloud_callback, this);
		point_cloud_publisher  = nh.advertise<sensor_msgs::PointCloud2> (TOPIC_EXTRACTED_OBJECTS, BUFFER_SIZE);
	}

	void update() {
		//pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
		
		pcl::fromROSMsg (input, *cloud);

		filterCloud(cloud);
		cloud = segmentCloud(cloud);

        cloudPublish(cloud);
    }

private:
	void cloudPublish(pcl::PointCloud<PointT>::Ptr cloud_pub)
	{
		pcl::toROSMsg(*cloud_pub, output);
		point_cloud_publisher.publish (output);
	}

	void filterCloud(pcl::PointCloud<PointT>::Ptr cloud_filter)
    {
    	// Build a passthrough filter to remove spurious NaNs
		pcl::PassThrough<PointT> pass;

		pass.setInputCloud (cloud_filter);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (0.3, filter_z);
		pass.filter (*cloud_filter);

		pass.setInputCloud (cloud_filter);
		pass.setFilterFieldName ("x");
		pass.setFilterLimits (-filter_x, filter_x);
		pass.filter (*cloud_filter);

		pass.setInputCloud (cloud_filter);
		pass.setFilterFieldName ("y");
		pass.setFilterLimits (0, filter_y);
		pass.filter (*cloud_filter);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr segmentCloud(pcl::PointCloud<PointT>::Ptr cloud_seg)
    {
    	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
		pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

		pcl::SACSegmentation<pcl::PointXYZ> seg;
		pcl::ExtractIndices<PointT> extract;

		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setDistanceThreshold (seg_distance);

		int i = 0, nr_points = (int) cloud_seg->points.size ();
		// While 20% of the original cloud is still there
		while (cloud_seg->points.size () > seg_percentage * nr_points && i < 10)
		{
			//seg.setInputCloud (cloud);
			seg.setInputCloud (cloud_seg);
			seg.segment (*inliers, *coeff); 
			if (inliers->indices.size () == 0)
			{
				std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
				break;
			}
			if(inliers->indices.size() < nr_points/10){
				i++;
				std::cerr << i << "To small" << std::endl;
				continue;
			}
			//ROS_INFO("%d, %d",i, (int)inliers->indices.size());
			// Extract the planar inliers from the input cloud
			extract.setInputCloud (cloud_seg);
			extract.setIndices (inliers);
			extract.setNegative (true);
			extract.filter (*cloud_plane);
			ROS_INFO("%d, %d",(int)cloud_seg->points.size (), (int)cloud_plane->points.size ());
			cloud_seg.swap (cloud_plane);
			i++;
		}
		return cloud_seg;
		//cloudPublish(cloud_seg);
    }

	void point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
	{
		input = *cloud_msg;
	}

    void add_params() 
    {
        add_param(PARAM_FILTER_X_NAME, filter_x, PARAM_FILTER_X_DEFAULT);
        add_param(PARAM_FILTER_Y_NAME, filter_y, PARAM_FILTER_Y_DEFAULT);
        add_param(PARAM_FILTER_Z_NAME, filter_z, PARAM_FILTER_Z_DEFAULT);
        add_param(PARAM_SEG_DISTANCE_NAME, seg_distance, PARAM_SEG_DISTANCE_DEFAULT);
        add_param(PARAM_SEG_PERCENTAGE_NAME, seg_percentage, PARAM_SEG_PERCENTAGE_DEFAULT);
    }
};

int main(int argc, char **argv) {
    
    ros::init(argc, argv, NODE_NAME);

    ObjectRecognition recognizer(HZ);
    ros::Rate loop_rate(HZ);

    while(ros::ok()) {
        recognizer.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}