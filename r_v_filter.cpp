#include "r_v_filter.h"

/*
 * void void_voxel_grid(pcl::PointCloud<pcl::PointXYZI>::Ptr voxelbefore,pcl::PointCloud<pcl::PointXYZI>::Ptr voxelafter)
 * 
 */
void void_voxel_grid(pcl::PointCloud<pcl::PointXYZ>::Ptr voxelbefore,pcl::PointCloud<pcl::PointXYZ>::Ptr voxelafter)
{
    pcl::VoxelGrid<pcl::PointXYZ> sor;//滤波处理对象
    sor.setInputCloud(voxelbefore);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);//设置滤波器处理时采用的体素大小的参数
    sor.filter(*voxelafter);
    return;
}


/*
 * void void_RadiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZI>::Ptr RadiuRemovalbefore,pcl::PointCloud<pcl::PointXYZI>::Ptr RadiuRemovalafter)
 * 
 */
void void_RadiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr RadiuRemovalbefore,pcl::PointCloud<pcl::PointXYZ>::Ptr RadiuRemovalafter)
{
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(RadiuRemovalbefore);
	sor.setRadiusSearch(0.02);
	sor.setMinNeighborsInRadius(15);
	sor.setNegative(false);
	sor.filter(*RadiuRemovalafter);
	return;
}

