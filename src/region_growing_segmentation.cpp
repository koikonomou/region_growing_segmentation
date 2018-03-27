#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <my_new_msgs/clustering.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/impl/point_types.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <pcl/common/projection_matrix.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <vector>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/region_growing.h>

ros::Publisher pub;
int ksearch, minclustersize, maxclustersize, numberofneighbours;
float a, b, curvaturethreshold;
double smoothnessthreshold;


void segmentation_cb(const sensor_msgs::PointCloud2& msg){

    pcl::PCLPointCloud2 cloud2;
    pcl_conversions::toPCL( msg , cloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::fromPCLPointCloud2(cloud2, *cloud);

    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud);
    normal_estimator.setKSearch (ksearch);
    normal_estimator.compute (*normals);

    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (a, b);
    pass.filter (*indices);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (minclustersize);
    reg.setMaxClusterSize (maxclustersize);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (numberofneighbours);
    reg.setInputCloud (cloud);
    //reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (smoothnessthreshold);
    reg.setCurvatureThreshold (curvaturethreshold);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
    std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
    std::cout << "These are the indices of the points of the initial" <<
      std::endl << "cloud that belong to the first cluster:" << std::endl;
    int j = 0;

    my_new_msgs::clustering msg_;

    std::vector<sensor_msgs::PointCloud2 > clouds; 

    for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin (); it != clusters.end (); ++it)
    {

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            cloud_cluster->points.push_back (cloud->points[*pit]); 
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        j++;

        sensor_msgs::PointCloud2 msgout;
        pcl::PCLPointCloud2 cloud2;
        pcl::toPCLPointCloud2(*cloud_cluster, cloud2);

        pcl_conversions::fromPCL(cloud2, msgout);
        clouds.push_back(msgout);
        msg_.clusters.push_back(msgout);

    }
    pub.publish(msg_);

}


int main (int argc, char** argv){
    ros::init (argc, argv, "region_clustering");
    ros::NodeHandle n_;

    n_.param("region_clustering/setKSearch", ksearch, 50);
    n_.param("region_clustering/setFilterLimits_a", a, 0.0f);
    n_.param("region_clustering/setFilterLimit_b", b, 0.1f);
    n_.param("region_clustering/setMinClusterSize", minclustersize, 50 );
    n_.param("region_clustering/setMaxClusterSize", maxclustersize, 1000000);
    n_.param("region_clustering/setNumberOfNeighbours", numberofneighbours, 30);
    n_.param("region_clustering/setSmoothnessThreshold", smoothnessthreshold, (3.0/ 180.0 * M_PI));
    n_.param("region_clustering/setCurvatureThreshold", curvaturethreshold, 1.0f);

    std::string input_topic;
    std::string out_topic;
    n_.param("region_clustering/cloud_topic",input_topic, std::string("/new_point_cloud"));
    n_.param("region_clustering/output_cloud_topic", out_topic, std::string("/region_clusters"));


    ros::Subscriber sub = n_.subscribe (input_topic, 1, segmentation_cb);

    pub = n_.advertise<my_new_msgs::clustering> (out_topic, 1);

    ros::spin ();
}