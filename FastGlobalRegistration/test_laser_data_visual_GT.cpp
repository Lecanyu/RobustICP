//
// Visualize to check if the laser's ground-truth pose is correct 
//

#include "app.h"
#include "LaserClouds.h"

#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>


int main(int argc, char** argv)
{
	std::string laser_data_log = "C:/Users/range/Desktop/RobustICP/dataset/2d/fr079-complete.gfs.log";
	LaserClouds laser_clouds(laser_data_log);
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0, 0, 5, 0, 0, -1, 0, 1, 0);

	//for (int i = 0; i<laser_clouds._pointClouds.size(); ++i)
	for (int i = 0; i<15; ++i)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(*laser_clouds._pointClouds[i], *transformed, laser_clouds._posesGT_3d[i]);

		std::stringstream ss;
		ss << "point_cloud" << i;
		viewer->addPointCloud(transformed, ss.str());
	}

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}
