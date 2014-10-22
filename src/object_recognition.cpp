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
// OTHER
#include <vector>


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
#define PARAM_OBJ_DISTANCE_NAME			"obj_distance"
#define PARAM_OBJ_DISTANCE_DEFAULT		0.005
#define PARAM_OBJ_MIN_NAME				"obj_min"
#define PARAM_OBJ_MIN_DEFAULT			5.0

typedef pcl::PointXYZ PointT;

class ObjectRecognition : public s8::Node 
{
	const int hz;

	ros::Subscriber point_cloud_subscriber;
	ros::Publisher point_cloud_publisher;

	double filter_x, filter_y, filter_z;
	double seg_distance, seg_percentage;
	double obj_min, obj_distance;

	sensor_msgs::PointCloud2 input, output, cloud_msg;

public:
	ObjectRecognition(int hz) : hz(hz)
	{
		add_params();
		//printParams();
		point_cloud_subscriber = nh.subscribe(TOPIC_POINT_CLOUD, BUFFER_SIZE, &ObjectRecognition::point_cloud_callback, this);
		point_cloud_publisher  = nh.advertise<sensor_msgs::PointCloud2> (TOPIC_EXTRACTED_OBJECTS, BUFFER_SIZE);
	}

	void update() {
		pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
		
		pcl::fromROSMsg (input, *cloud);

		filterCloud(cloud);
		cloud = segmentCloud(cloud);
		cloudPublish(cloud);
		recognizeObject(cloud);
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

    pcl::PointCloud<PointT>::Ptr segmentCloud(pcl::PointCloud<PointT>::Ptr cloud_seg)
    {
    	pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT>);
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
				//std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
				break;
			}
			if(inliers->indices.size() < nr_points/10){
				i++;
				//std::cerr << i << "To small" << std::endl;
				continue;
			}
			//ROS_INFO("%d, %d",i, (int)inliers->indices.size());
			// Extract the planar inliers from the input cloud
			extract.setInputCloud (cloud_seg);
			extract.setIndices (inliers);
			extract.setNegative (true);
			extract.filter (*cloud_plane);
			//ROS_INFO("%d, %d",(int)cloud_seg->points.size (), (int)cloud_plane->points.size ());
			cloud_seg.swap (cloud_plane);
			i++;
		}
		return cloud_seg;
		//cloudPublish(cloud_seg);
    }

    void recognizeObject(pcl::PointCloud<PointT>::Ptr cloud_obj)
    {
    	pcl::PointCloud<PointT>::Ptr cloud_seg (new pcl::PointCloud<PointT>);
    	std::vector<std::vector<float> > coeffMatrix;
    	pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT>);
		pcl::ModelCoefficients coeff;
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

		pcl::SACSegmentation<pcl::PointXYZ> seg;
		pcl::ExtractIndices<PointT> extract;

		cloud_seg = cloud_obj;

		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setDistanceThreshold (obj_distance);

		int j = 0;
		int i = 0, nr_points = (int) cloud_seg->points.size ();
		if (nr_points < 150){
			return;
		}
		// While 20% of the original cloud is still there
		while (cloud_seg->points.size () > 0.1 * nr_points && i < 10)
		{
			//seg.setInputCloud (cloud);
			seg.setInputCloud (cloud_seg);
			seg.segment (*inliers, coeff); 
			if (inliers->indices.size () == 0)
			{
				std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
				break;
			}
			if(inliers->indices.size() < nr_points/obj_min){
				i++;
				//std::cerr << i << "To small" << std::endl;
				continue;
			}
			//ROS_INFO("%d, %d",i, (int)inliers->indices.size());
			// Extract the planar inliers from the input cloud
			extract.setInputCloud (cloud_seg);
			extract.setIndices (inliers);
			extract.setNegative (true);
			extract.filter (*cloud_plane);
			//ROS_INFO("%d, %d",(int)cloud_seg->points.size (), (int)cloud_plane->points.size ());
			cloud_seg.swap (cloud_plane);
			coeffMatrix.push_back(coeff.values);
			//ROS_INFO("%f,%f,%f,%f",coeffMatrix[0][0],coeffMatrix[0][1],coeffMatrix[0][2],coeffMatrix[0][3]);
			i++;
			j++;
		}
		float angle;
		if (coeffMatrix.size() == 2)
		{
			angle = getAngle(coeffMatrix[0],coeffMatrix[1]);
			ROS_INFO("%f", angle);
		}
		ROS_INFO("%lu", coeffMatrix.size());
    }

    float getAngle (std::vector<float> v1, std::vector<float> v2)
	{
		float dotproduct = v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2];
		float len1 = v1[0]*v1[0]+v1[1]*v1[1]+v1[2]*v1[2];
		float len2 = v2[0]*v2[0]+v2[1]*v2[1]+v2[2]*v2[2];
		float angleRad = acos(dotproduct/(sqrt(len1)*sqrt(len2)));
		return angleRad*180/3.1415;
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
        add_param(PARAM_OBJ_MIN_NAME, obj_min, PARAM_OBJ_MIN_DEFAULT);
        add_param(PARAM_OBJ_DISTANCE_NAME, obj_distance, PARAM_OBJ_DISTANCE_DEFAULT);
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