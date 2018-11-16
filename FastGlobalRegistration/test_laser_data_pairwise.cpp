//
// Apply ICP to register pairwisely
//

#include "app.h"
#include "LaserClouds.h"

#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>


int main(int argc, char** argv)
{
	std::string laser_data_log = "C:/Users/range/Desktop/RobustICP/dataset/2d/fr079-complete.gfs.log";

	LaserClouds laser_clouds(laser_data_log);
	laser_clouds.CalculateDescriptorNoKeypoints();

	int id1 = 0, id2 = 11;

	fgr::CApp app;
	app.LoadFeature(laser_clouds._pointsForICP[id1], laser_clouds._descriptorForICP[id1]);
	app.LoadFeature(laser_clouds._pointsForICP[id2], laser_clouds._descriptorForICP[id2]);
	app.NormalizePoints();
	app.AdvancedMatching();
	app.OptimizePairwise(true);
	Eigen::Matrix4f alignment = app.GetOutputTrans();


	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0, 0, 5, 0, 0, -1, 0, 1, 0);

	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*laser_clouds._pointClouds[id2], *transformed, alignment);
	viewer->addPointCloud(laser_clouds._pointClouds[id1], "1");
	viewer->addPointCloud(laser_clouds._pointClouds[id2], "2");
	viewer->addPointCloud(transformed, "3");

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 255, 0, "3");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}
