#ifndef R_V_FILTER_H_
#define R_V_FILTER_H_
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/radius_outlier_removal.h>

void void_voxel_grid(pcl::PointCloud<pcl::PointXYZ>::Ptr voxelbefore,pcl::PointCloud<pcl::PointXYZ>::Ptr voxelafter);
void void_RadiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr RadiuRemovalbefore,pcl::PointCloud<pcl::PointXYZ>::Ptr RadiuRemovalafter);

#endif
