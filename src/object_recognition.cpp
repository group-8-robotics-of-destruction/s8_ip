#include <ros/ros.h>
#include <s8_common_node/Node.h>
// PCL specific includes
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
// OTHER
#include <vector>


// DEFINITIONS
#define HZ                  10
#define BUFFER_SIZE         10

#define NODE_NAME           		"s8_object_recognition_node"
#define TOPIC_POINT_CLOUD   		"/camera/depth_registered/points"
#define TOPIC_EXTRACTED_OBJECTS		"/s8/extractedObjects"

// PARAMETERS
#define PARAM_FILTER_X_NAME						"filter_x"
#define PARAM_FILTER_X_DEFAULT					0.1
#define PARAM_FILTER_Z_NAME						"filter_z"
#define PARAM_FILTER_Z_DEFAULT					1.0
#define PARAM_FILTER_Y_NAME						"filter_y"
#define PARAM_FILTER_Y_DEFAULT					0.2
#define PARAM_SEG_DISTANCE_NAME					"seg_distance"
#define PARAM_SEG_DISTANCE_DEFAULT				0.01
#define PARAM_SEG_PERCENTAGE_NAME				"seg_percentage"
#define PARAM_SEG_PERCENTAGE_DEFAULT			0.2
#define PARAM_PLANE_DISTANCE_NAME				"plane_distance"
#define PARAM_PLANE_DISTANCE_DEFAULT			0.002
#define PARAM_PLANE_ITEM_PERCENTAGE_NAME		"plane_item_percentage"
#define PARAM_PLANE_ITEM_PERCENTAGE_DEFAULT		0.05
#define PARAM_PLANE_CLUSTER_PERCENTAGE_NAME		"plane_cluster_percentage"
#define PARAM_PLANE_CLUSTER_PERCENTAGE_DEFAULT	0.1
#define PARAM_PLANE_MIN_SIZE_NAME				"plane_min_size"
#define PARAM_PLANE_MIN_SIZE_DEFAULT			100
#define PARAM_CYL_DISTANCE_NAME					"cyl_distance"
#define PARAM_CYL_DISTANCE_DEFAULT				0.01
#define PARAM_CYL_NORMAL_DISTANCE_NAME			"cyl_normal_distance"
#define PARAM_CYL_NORMAL_DISTANCE_DEFAULT		0.8
#define PARAM_CYL_ITEM_PERCENTAGE_NAME			"cyl_item_percentage"
#define PARAM_CYL_ITEM_PERCENTAGE_DEFAULT		0.05
#define PARAM_CYL_CLUSTER_PERCENTAGE_NAME		"cyl_cluster_percentage"
#define PARAM_CYL_CLUSTER_PERCENTAGE_DEFAULT	0.1
#define PARAM_CYL_MIN_SIZE_NAME					"cyl_min_size"
#define PARAM_CYL_MIN_SIZE_DEFAULT				2
#define PARAM_CYL_RAD_MIN_NAME					"cyl_rad_min"
#define PARAM_CYL_RAD_MIN_DEFAULT				0.01
#define PARAM_CYL_RAD_MAX_NAME					"cyl_rad_max"
#define PARAM_CYL_RAD_MAX_DEFAULT				0.04

#define PARAM_STAT_KMEAN_NAME					"std_kmean"
#define PARAM_STAT_KMEAN_DEFAULT				10
#define PARAM_STAT_STDDIST_NAME					"std_dist"
#define PARAM_STAT_STDDIST_DEFAULT				0.001
#define PARAM_VOXEL_LEAF_SIZE_NAME				"voxel_leaf_size"
#define PARAM_VOXEL_LEAF_SIZE_DEFAULT			0.005


typedef pcl::PointXYZRGB PointT;

class ObjectRecognition : public s8::Node 
{
	const int hz;

	ros::Subscriber point_cloud_subscriber;
	ros::Publisher point_cloud_publisher;

	double 	filter_x, filter_y, filter_z;
	double 	seg_distance, seg_percentage;
	double 	plane_distance, plane_cluster_percentage, plane_item_percentage;
	int 	plane_min_size;
	double 	cyl_distance, cyl_cluster_percentage, cyl_item_percentage, cyl_normal_distance, cyl_rad_min, cyl_rad_max;
	int 	cyl_min_size;
	double 	std_dist; 
	int 	std_kmean;
	double	voxel_leaf_size;

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

		passthroughFilterCloud(cloud);
		voxelGridCloud(cloud);
		cloud = segmentCloud(cloud);
		statisticalOutlierRemovalCloud(cloud);
		
		cloudPublish(cloud);
		
		pcl::PointCloud<PointT>::Ptr cloudCylinder (new pcl::PointCloud<PointT>);
		*cloudCylinder = *cloud;

		std::vector<std::vector<float> > coefficientsPlane;
		int inlierSizePlane = 0;
		recognizePlaneObject(cloud, &coefficientsPlane, &inlierSizePlane);

		std::vector<std::vector<float> > coefficientsCylinder;
		int inlierSizeCylinder = 0;
		recognizeCylinderObject(cloudCylinder, &coefficientsCylinder, &inlierSizeCylinder);

		ROS_INFO("Plane: %d, Cylinder: %d", inlierSizePlane, inlierSizeCylinder);
    }

private:
	void cloudPublish(pcl::PointCloud<PointT>::Ptr cloud_pub)
	{
		pcl::toROSMsg(*cloud_pub, output);
		point_cloud_publisher.publish (output);
	}

	// Removes outliers using a StatisticalOutlierRemoval filter
	void statisticalOutlierRemovalCloud(pcl::PointCloud<PointT>::Ptr cloud_stat)
	{
		pcl::StatisticalOutlierRemoval<PointT> sta;
		sta.setInputCloud (cloud_stat);
		sta.setMeanK (std_kmean);
		sta.setStddevMulThresh (std_dist);
		sta.filter (*cloud_stat);
	}

	// Down samples the point cloud using VoxelGrid filter
	// to make computations easier.
	void voxelGridCloud(pcl::PointCloud<PointT>::Ptr cloud_stat)
	{
		pcl::VoxelGrid<PointT> sor;
  		sor.setInputCloud (cloud_stat);
  		sor.setLeafSize ((float)voxel_leaf_size, (float)voxel_leaf_size, (float)voxel_leaf_size);
  		sor.filter (*cloud_stat);
	}

	// Build a passthrough filter to reduce field of view.
	void passthroughFilterCloud(pcl::PointCloud<PointT>::Ptr cloud_filter)
    {
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
		pass.setFilterLimits (-filter_y, 0.2);
		pass.filter (*cloud_filter);
    }

    pcl::PointCloud<PointT>::Ptr segmentCloud(pcl::PointCloud<PointT>::Ptr cloud_seg)
    {
    	pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT>);
		pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

		pcl::SACSegmentation<PointT> seg;
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
			if(inliers->indices.size() < nr_points/20){
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

    void recognizePlaneObject(pcl::PointCloud<PointT>::Ptr cloud_seg, std::vector<std::vector<float> > *coeffMatrix, int *inlierSize)
    {
    	pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT>);
    	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
		pcl::ModelCoefficients coeff;
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

		pcl::ExtractIndices<PointT> extract;
		pcl::NormalEstimation<PointT, pcl::Normal> ne;
  		pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 

  		// Estimate point normals
		ne.setSearchMethod (tree);
		ne.setKSearch (50);

		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setDistanceThreshold (plane_distance);
		seg.setNormalDistanceWeight (0.1);
		seg.setMaxIterations (1000);

		int i = 0, nr_points = (int) cloud_seg->points.size ();
		if (nr_points < plane_min_size){
			return;
		}
		// While 10% of the original cloud is still there
		while (cloud_seg->points.size () > plane_cluster_percentage * nr_points && i < 5)
		{
			//seg.setInputCloud (cloud);
			ne.setInputCloud (cloud_seg);
			ne.compute (*cloud_normals);
			seg.setInputCloud (cloud_seg);
			seg.setInputNormals (cloud_normals);
			seg.segment (*inliers, coeff);
			if (inliers->indices.size () == 0)
			{
				std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
				break;
			}
			if(inliers->indices.size() < nr_points*plane_item_percentage|| inliers->indices.size() < plane_min_size){
				i++;
				continue;
			}
			// Extract the planar inliers from the input cloud
			extract.setInputCloud (cloud_seg);
			extract.setIndices (inliers);
			extract.setNegative (true);
			extract.filter (*cloud_plane);
			cloud_seg.swap (cloud_plane);
			(*coeffMatrix).push_back(coeff.values);
			i++;
			*inlierSize += inliers->indices.size();
		}
		float angle;
		if ((*coeffMatrix).size() == 2)
		{
			angle = getAngle((*coeffMatrix)[0],(*coeffMatrix)[1]);
			//ROS_INFO("%f", angle);
		}
		//ROS_INFO("%lu", (*coeffMatrix).size());
    }

    void recognizeCylinderObject(pcl::PointCloud<PointT>::Ptr cloud_seg, std::vector<std::vector<float> > *coeffMatrix, int *inlierSize)
    {
    	pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT>);
    	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
		pcl::ModelCoefficients coeff;
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

		pcl::ExtractIndices<PointT> extract;

		pcl::NormalEstimation<PointT, pcl::Normal> ne;
  		pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 

  		// Estimate point normals
		ne.setSearchMethod (tree);
		ne.setKSearch (50);

		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_CYLINDER);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setNormalDistanceWeight (cyl_normal_distance);
		seg.setMaxIterations (1000);
		seg.setDistanceThreshold (cyl_distance);
  		seg.setRadiusLimits (0.015, 0.03);

		int i = 0, nr_points = (int) cloud_seg->points.size ();
		if (nr_points < cyl_min_size){
			return;
		}
		// While 10% of the original cloud is still there
		while (cloud_seg->points.size () > cyl_cluster_percentage * nr_points && i < 5)
		{
			//seg.setInputCloud (cloud);
			ne.setInputCloud (cloud_seg);
			ne.compute (*cloud_normals);
			seg.setInputCloud (cloud_seg);
			seg.setInputNormals (cloud_normals);
			seg.segment (*inliers, coeff); 
			if (inliers->indices.size () == 0)
			{
				std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
				break;
			}
			if(inliers->indices.size() < nr_points*cyl_item_percentage|| inliers->indices.size() < cyl_min_size){
				i++;
				continue;
			}
			// Extract the planar inliers from the input cloud
			extract.setInputCloud (cloud_seg);
			extract.setIndices (inliers);
			extract.setNegative (true);
			extract.filter (*cloud_plane);
			cloud_seg.swap (cloud_plane);
			(*coeffMatrix).push_back(coeff.values);
			i++;
			*inlierSize += inliers->indices.size();
		}
		float angle;
		if ((*coeffMatrix).size() == 2)
		{
			angle = getAngle((*coeffMatrix)[0],(*coeffMatrix)[1]);
			//ROS_INFO("%f", angle);
		}
		//ROS_INFO("%lu", (*coeffMatrix).size());
    }

    // Calculates the angle between two 3D planes.
    float getAngle (std::vector<float> v1, std::vector<float> v2)
	{
		float dotproduct 	= v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2];
		float len1 			= v1[0]*v1[0]+v1[1]*v1[1]+v1[2]*v1[2];
		float len2 			= v2[0]*v2[0]+v2[1]*v2[1]+v2[2]*v2[2];
		float angleRad 		= acos(dotproduct/(sqrt(len1)*sqrt(len2)));
		return angleRad*180/3.1415;
	}

	void point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
	{
		input = *cloud_msg;
	}

    void add_params() 
    {
    	// Passthrough filter parameters.
        add_param(PARAM_FILTER_X_NAME, filter_x, PARAM_FILTER_X_DEFAULT);
        add_param(PARAM_FILTER_Y_NAME, filter_y, PARAM_FILTER_Y_DEFAULT);
        add_param(PARAM_FILTER_Z_NAME, filter_z, PARAM_FILTER_Z_DEFAULT);
        // Segmentation parameters, remove floors and walls.
        add_param(PARAM_SEG_DISTANCE_NAME, seg_distance, PARAM_SEG_DISTANCE_DEFAULT);
        add_param(PARAM_SEG_PERCENTAGE_NAME, seg_percentage, PARAM_SEG_PERCENTAGE_DEFAULT);
        // Plane recognition parameters.
        add_param(PARAM_PLANE_MIN_SIZE_NAME, plane_min_size, PARAM_PLANE_MIN_SIZE_DEFAULT);
        add_param(PARAM_PLANE_DISTANCE_NAME, plane_distance, PARAM_PLANE_DISTANCE_DEFAULT);
        add_param(PARAM_PLANE_ITEM_PERCENTAGE_NAME, plane_item_percentage, PARAM_PLANE_ITEM_PERCENTAGE_DEFAULT);
        add_param(PARAM_PLANE_CLUSTER_PERCENTAGE_NAME, plane_cluster_percentage, PARAM_PLANE_CLUSTER_PERCENTAGE_DEFAULT);
        // Cylinder recognition parameters.
        add_param(PARAM_CYL_MIN_SIZE_NAME, cyl_min_size, PARAM_CYL_MIN_SIZE_DEFAULT);
        add_param(PARAM_CYL_DISTANCE_NAME, cyl_distance, PARAM_CYL_DISTANCE_DEFAULT);
        add_param(PARAM_CYL_NORMAL_DISTANCE_NAME, cyl_normal_distance, PARAM_CYL_NORMAL_DISTANCE_DEFAULT);
        add_param(PARAM_CYL_ITEM_PERCENTAGE_NAME, cyl_item_percentage, PARAM_CYL_ITEM_PERCENTAGE_DEFAULT);
        add_param(PARAM_CYL_CLUSTER_PERCENTAGE_NAME, cyl_cluster_percentage, PARAM_CYL_CLUSTER_PERCENTAGE_DEFAULT);
        add_param(PARAM_CYL_RAD_MIN_NAME, cyl_rad_min, PARAM_CYL_RAD_MIN_DEFAULT);
        add_param(PARAM_CYL_RAD_MAX_NAME, cyl_rad_max, PARAM_CYL_RAD_MAX_DEFAULT);
        // Statistical outlier parameters.
        add_param(PARAM_STAT_KMEAN_NAME, std_kmean, PARAM_STAT_KMEAN_DEFAULT);
        add_param(PARAM_STAT_STDDIST_NAME, std_dist, PARAM_STAT_STDDIST_DEFAULT);
        // Voxel filtering parameters
        add_param(PARAM_VOXEL_LEAF_SIZE_NAME, voxel_leaf_size, PARAM_VOXEL_LEAF_SIZE_DEFAULT);
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