#include <stdlib.h>
#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread.hpp>
#include <boost/thread/scoped_thread.hpp>
#include <boost/chrono.hpp>
int user_data;
ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr processed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> m_rgb_source;
pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> m_rgb_filtered;

boost::mutex m_updateModelMutex;
bool m_update=false;
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
int view_left;
int view_right;

void pcl_pipeline()
{
	/*pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud (source_cloud);
	sor.setMeanK (50);
	sor.setStddevMulThresh (1.0);
	sor.filter (*processed_cloud);
	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud(source_cloud);

	sor.setLeafSize (1.0f, 1.0f, 1.0f);
//	sor.filter (*processed_cloud);*/


}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

	boost::mutex::scoped_lock updateLock(m_updateModelMutex);
		m_update = true;
		pcl::fromROSMsg(*cloud_msg, *source_cloud);
		//processed_cloud = source_cloud;
		//ROS_INFO_STREAM("Received a message");
		std::cout<<"Height:"<<source_cloud->height<<std::endl;
			std::cout<<"Width:"<<source_cloud->width<<std::endl;
		boost::scoped_thread<> pipeline_worker(boost::thread(pcl_pipeline));
		//pipeline_worker.join();
	updateLock.unlock();

}


boost::shared_ptr<pcl::visualization::PCLVisualizer> createViewer ()
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->initCameraParameters ();

  int v1(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor (0, 0, 0, v1);
  viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);

 // viewer->addPointCloud<pcl::PointXYZRGB> (source_cloud, rgb, "sample cloud1", v1);

  int v2(0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor (0.0, 0.0, 0.0, v2);
  viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);

  viewer->addCoordinateSystem (0.1);
  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> m_rgb_filtered(processed_cloud, 0, 255, 0);
  //viewer->addPointCloud<pcl::PointXYZRGB> (processed_cloud, single_color, "sample cloud2", v2);
 // viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals1, 10, 0.05, "normals1", v1);
 // viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals2, 10, 0.05, "normals2", v2);


  return (viewer);
}

void visualize()
{
	 while (!viewer->wasStopped ())
	       {
	           viewer->spinOnce (100);
	           //Get lock on the boolean update and check if cloud was updated
	           boost::mutex::scoped_lock updateLock(m_updateModelMutex);
	           if(m_update)
	           {
	        	   m_rgb_source.setInputCloud(source_cloud);
	        	   m_rgb_filtered.setInputCloud(processed_cloud);
	               if(!viewer->updatePointCloud(source_cloud, "Source cloud"))
	            	  viewer->addPointCloud(source_cloud, m_rgb_source, "Source cloud", view_left);
	               if(!viewer->updatePointCloud(processed_cloud, "Filtered cloud"))
	                  viewer->addPointCloud(processed_cloud, m_rgb_filtered, "Filtered cloud", view_right);
	               m_update = false;
	               //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Source cloud");
	               //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Filtered cloud");

	           }
	           updateLock.unlock();
	 }
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "pclstuff");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/vertex_rgb_data", 1, cloud_cb);
	viewer = createViewer();
	boost::thread vis_thread(visualize);
	ros::spin();
    return 0;

}
