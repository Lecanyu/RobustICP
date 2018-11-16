//
// Directly use ICP to register laser point clouds.
//

#include "app.h"
#include "LaserClouds.h"

#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>


int main(int argc, char** argv)
{
	std::string laser_data_log = "C:/Users/range/Desktop/RobustICP/dataset/2d/fr079-complete.gfs.log";
	// incremental reconstruction indices
	int start_id = 0, end_id = 30;


	LaserClouds laser_clouds(laser_data_log);
	laser_clouds.CalculateDescriptorNoKeypoints();

	for (int i = start_id+1; i < end_id; ++i)
	{
		std::cout << "align " << i - 1 << "--" << i << "\n\n";
		fgr::CApp app;
		app.LoadFeature(laser_clouds._pointsForICP[i-1], laser_clouds._descriptorForICP[i-1]);
		app.LoadFeature(laser_clouds._pointsForICP[i], laser_clouds._descriptorForICP[i]);
		app.NormalizePoints();
		app.AdvancedMatching();
		app.OptimizePairwise(true);
		Eigen::Matrix4f alignment = app.GetOutputTrans();
		laser_clouds._poses_3d[i] = alignment * laser_clouds._poses_3d[i - 1];
	}
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0, 0, 5, 0, 0, -1, 0, 1, 0);

	for (int i = start_id; i<end_id; ++i)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(*laser_clouds._pointClouds[i], *transformed, laser_clouds._poses_3d[i]);

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
