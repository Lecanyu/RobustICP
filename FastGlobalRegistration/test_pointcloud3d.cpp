//
// Apply ICP to register 3d point clouds
//

#include "app.h"
#include "PointCloud3d.h"

#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>


int main(int argc, char** argv)
{
	std::string pcd_file1 = "C:/Users/range/Desktop/RobustICP/dataset/3d/pointcloud_ds0.pcd";
	std::string pcd_file2 = "C:/Users/range/Desktop/RobustICP/dataset/3d/pointcloud_ds4.pcd";

	PointCloud3d cloud1(pcd_file1);
	PointCloud3d cloud2(pcd_file2);

	cloud1.CalculateNormals();
	cloud2.CalculateNormals();

	cloud1.CalculateFeature();
	cloud2.CalculateFeature();

	std::vector<Eigen::Vector3f> points1;
	std::vector<Eigen::VectorXf> features1;
	cloud1.ConvertFeatures(points1, features1);

	std::vector<Eigen::Vector3f> points2;
	std::vector<Eigen::VectorXf> features2;
	cloud2.ConvertFeatures(points2, features2);

	fgr::CApp app;
	app.LoadFeature(points1, features1);
	app.LoadFeature(points2, features2);
	app.NormalizePoints();
	app.AdvancedMatching();
	app.OptimizePairwise(true);
	Eigen::Matrix4f alignment = app.GetOutputTrans();
	cloud2._pose = alignment;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_out(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud(*cloud2._pointCloud, *scene_out, cloud2._pose);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0, 0, 5, 0, 0, -1, 0, 1, 0);

	viewer->addPointCloud(cloud1._pointCloud, "point_cloud1");
	//viewer->addPointCloud(cloud2._pointCloud, "point_cloud2");
	viewer->addPointCloud(scene_out, "transformed_cloud2");
	//    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud1._pointCloud, cloud1._normals, 1, 0.08, "normals");
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}
