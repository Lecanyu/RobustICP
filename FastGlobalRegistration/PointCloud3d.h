//
// Created by lecanyu on 18-11-14.
//

#ifndef FASTGLOBALREGISTRATION_POINTCLOUD3D_H
#define FASTGLOBALREGISTRATION_POINTCLOUD3D_H

#include <vector>
#include <iostream>
#include <string>

#include <Eigen/Eigen>

#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>


class PointCloud3d
{
public:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _pointCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _pointCloudXYZ;
    Eigen::Matrix4d _pose;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr _feature;
    pcl::PointCloud<pcl::Normal>::Ptr _normals;


    PointCloud3d()
        :_pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>),
        _pose(Eigen::Matrix4d::Identity()),
        _feature(new pcl::PointCloud<pcl::FPFHSignature33>),
        _normals(new pcl::PointCloud<pcl::Normal>),
        _pointCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>) {}

    PointCloud3d(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud)
        :_pointCloud(point_cloud),
        _pose(Eigen::Matrix4d::Identity()),
        _feature(new pcl::PointCloud<pcl::FPFHSignature33>),
        _normals(new pcl::PointCloud<pcl::Normal>),
        _pointCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>) {}


    PointCloud3d(std::string pcd_file)
        :_pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>),
         _pose(Eigen::Matrix4d::Identity()),
         _feature(new pcl::PointCloud<pcl::FPFHSignature33>),
         _normals(new pcl::PointCloud<pcl::Normal>),
         _pointCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::io::loadPCDFile(pcd_file, *scene);
        _pointCloud = scene;
    }


    void CalculateNormals(double radius = 0.2)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        for(int i=0;i<_pointCloud->size(); ++i)
        {
            auto x = _pointCloud->points[i].x;
            auto y = _pointCloud->points[i].y;
            auto z = _pointCloud->points[i].z;
            point_xyz->push_back(pcl::PointXYZ(x, y, z));
        }
        _pointCloudXYZ = point_xyz;

        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud (_pointCloudXYZ);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        ne.setSearchMethod (tree);
        ne.setRadiusSearch (radius);
//        ne.setKSearch(8);
        ne.compute (*_normals);
    }

    void CalculateFeature(double radius = 0.5)
    {
        pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
        fpfh.setInputCloud (_pointCloudXYZ);
        fpfh.setInputNormals (_normals);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        fpfh.setSearchMethod (tree);
        fpfh.setRadiusSearch (radius);
//        fpfh.setKSearch(8);
        fpfh.compute (*_feature);
    }

    void ConvertFeatures(std::vector<Eigen::Vector3f>& points_out, std::vector<Eigen::VectorXf>& features_out)
    {
        for(int i=0;i<_pointCloudXYZ->size(); ++i)
        {
            auto x = _pointCloudXYZ->points[i].x;
            auto y = _pointCloudXYZ->points[i].y;
            auto z = _pointCloudXYZ->points[i].z;
            Eigen::Vector3f p;
            p<<x,y,z;

            auto fe = _feature->points[i];

            Eigen::VectorXf feature(fe.descriptorSize());
            for(int j=0;j<fe.descriptorSize();++j)
                feature[j] = fe.histogram[j];
            std::cout<<feature<<"\n";
            std::cout<<"";
        }
    }

};



#endif //FASTGLOBALREGISTRATION_POINTCLOUD3D_H
