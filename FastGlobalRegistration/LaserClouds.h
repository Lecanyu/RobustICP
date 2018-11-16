//
// 2d laser point cloud interface aomong PCL, feature descriptor extraction and ICP requirement.
//

#pragma once

#include <vector>
#include <iostream>
#include <string>
#include <fstream>

#include <Eigen/Eigen>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Extract keyypoints and descriptors from 2d laser point cloud, see https://github.com/dlr1516/falkolib
#include <falkolib/Common/LaserScan.h>				// You can find a bunch of test cases in falkolib/test folder.
#include <falkolib/Feature/FALKOExtractor.h>
#include <falkolib/Feature/CGHExtractor.h>

class LaserClouds
{
public:
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> _pointClouds;		// z always equals to 0	
	std::vector<Eigen::Matrix4f> _posesGT_3d;
	std::vector<Eigen::Matrix4f> _poses_3d;
	std::vector<falkolib::LaserScan> _scans;

	std::vector<std::vector<Eigen::Vector3f>> _pointsForICP;			// will be filled and passed for ICP
	std::vector<std::vector<Eigen::VectorXf>> _descriptorForICP;		


	LaserClouds(std::string laser_data_log)
	{
		ParseLog(laser_data_log);
		for (int i = 0; i < _pointClouds.size(); ++i)
			_poses_3d.push_back(Eigen::Matrix4f::Identity());
	}

	// Calculate descriptor vector in all points instead of keypoints
	void CalculateDescriptorNoKeypoints()
	{
		falkolib::CGHExtractor<falkolib::FALKO> cgh(16, false, 0.1);

		_pointsForICP.resize(_scans.size());
		_descriptorForICP.resize(_scans.size());
		std::cout << "Calculating...\n";
		for (int i = 0; i < _scans.size(); ++i)
		{
			std::vector<falkolib::FALKO> keypoints;
			for (int j = 0; j < _scans[i].points.size(); ++j)
			{
				falkolib::FALKO p;
				p.index = j;
				keypoints.push_back(p);
			}

			std::vector<falkolib::CGH> cghDesc;
			cgh.compute(_scans[i], keypoints, cghDesc);


			for (int j = 0; j<keypoints.size(); ++j)
			{
				double x = _scans[i].points[keypoints[j].index](0);
				double y = _scans[i].points[keypoints[j].index](1);
				std::vector<double> descriptor_vec = cghDesc[j].getDiscriptor();

				Eigen::Vector3f p(x, y, 0);
				Eigen::VectorXf descriptor(descriptor_vec.size());
				for (int t = 0; t < descriptor_vec.size(); ++t)
					descriptor(t) = descriptor_vec[t];
				_descriptorForICP[i].push_back(descriptor);
				_pointsForICP[i].push_back(p);
			}
		}
	}

	// Calculate descriptor vector in keypoints
	void CalculateDescriptor()
	{
		falkolib::FALKOExtractor fe;
		fe.setMinExtractionRange(0);
		fe.setMaxExtractionRange(100);
		fe.enableSubbeam(true);
		fe.setNMSRadius(0.01);
		fe.setNeighB(0.1);
		fe.setBRatio(1.5);
		fe.setGridSectors(16);

		falkolib::CGHExtractor<falkolib::FALKO> cgh(16, false, 0.1);

		_pointsForICP.resize(_scans.size());
		_descriptorForICP.resize(_scans.size());
		std::cout << "Calculating...\n";
		for (int i = 0; i < _scans.size(); ++i)
		{
			std::vector<falkolib::FALKO> keypoints;
			fe.extract(_scans[i], keypoints);
			std::cout << "scan " << i <<", "<< keypoints.size() << " num keypoints extracted" << std::endl;
			std::vector<falkolib::CGH> cghDesc;
			cgh.compute(_scans[i], keypoints, cghDesc);

			// output point position and descriptor vector
			for (int j = 0; j<keypoints.size(); ++j)
			{
				double x = _scans[i].points[keypoints[j].index](0);
				double y = _scans[i].points[keypoints[j].index](1);
				std::vector<double> descriptor_vec = cghDesc[j].getDiscriptor();

				Eigen::Vector3f p(x, y, 0);
				Eigen::VectorXf descriptor(descriptor_vec.size());
				for (int t = 0; t < descriptor_vec.size(); ++t)
					descriptor(t) = descriptor_vec[t];
				_descriptorForICP[i].push_back(descriptor);
				_pointsForICP[i].push_back(p);
			}
		}
	}

private:
	void ParseLog(std::string laser_data_log)
	{
		std::ifstream infile(laser_data_log);
		std::string line;
		while(std::getline(infile, line))
		{
			std::vector<std::string> line_tokens;
			std::istringstream iss(line);
			for (std::string s; iss >> s; )
				line_tokens.push_back(s);
			if(line_tokens[0] == "FLASER")
			{
				std::vector<double> scaned_range;
				pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
				
				int i = 0;
				for(; i<atoi(line_tokens[1].c_str()); ++i)
				{
					float r = atof(line_tokens[i + 2].c_str());
					scaned_range.push_back(r);
				}
				
				float angle_inc = 0.5*3.1415926 / 180;
				falkolib::LaserScan scan;
				scan.setAngleMin(0);
				scan.setAngleInc(angle_inc);
				scan.setNumBeams(atoi(line_tokens[1].c_str()));
				scan.setTimestamp(0);
				scan.fromRanges(scaned_range);
				_scans.push_back(scan);

				for (int t = 0; t < scan.points.size(); ++t)
				{
					float x = scan.points[t](0);
					float y = scan.points[t](1);
					point_cloud->points.push_back(pcl::PointXYZ(x, y, 0));
				}
				_pointClouds.push_back(point_cloud);


				// TODO: Cannot figure out the scanner's pose from the carmen log file. Now the pose seems incorrect.
				Eigen::Matrix3f pose_gt;
				float xx = atof(line_tokens[i + 2].c_str());
				float yy = atof(line_tokens[i + 3].c_str());
				float theta = atof(line_tokens[i + 4].c_str());
				
				Eigen::Matrix4f rz, t;
				rz <<
					cos(theta), -sin(theta), 0, 0,
					sin(theta), cos(theta), 0, 0,
					0, 0, 1, 0,
					0, 0, 0, 1;
				t <<
					1, 0, 0, xx,
					0, 1, 0, yy,
					0, 0, 1, 0,
					0, 0, 0, 1;
				Eigen::Matrix4f pose_gt_3d = t * rz;

				_posesGT_3d.push_back(pose_gt_3d);
			}
		}

		
	}
};

