#include <iostream>
#include <string>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include "r_v_filter.h"

using namespace std;
using namespace pcl;

//路径名设置
#define PCD_FILE_PATH_FORMAT "../file/%d.pcd"
#define PCD_FILE_READ_ERROR_FORMAT "Couldn't read file %d.pcd \n"
int pcd_id_target = 1, pcd_id_source = 2;
char pcd_file_path_1[200], pcd_file_path_2[200], read_pcd_error[200];

//icp transformation info
Eigen::Matrix<float,4,4> transformation_matrix;

//callback para
bool next_iteration = false;

//打印旋转矩阵和平移矩阵
void print4x4Matrix(const Eigen::Matrix<float,4,4>& matrix); 
//KeyboardEvent callback
void KeyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,void* nothing);

/*
 * 主函数
 * show icp
 */
int main(int argc, char* argv[])
{
  /**********创建指向点云的指针 指针类型为pcl::PointCloud<pcl::PointXYZ>::Ptr，同时创建两个点云对象。**********/
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_pcd_p(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_pcd_reserve_p(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_pcd_p(new pcl::PointCloud<pcl::PointXYZ>);

  
  /**********设置路径读取两张pcd文件**********///后来者逼近前者,保存source
  pcd_id_target = 1, pcd_id_source = 2;
  sprintf(pcd_file_path_1, PCD_FILE_PATH_FORMAT, pcd_id_target);
  sprintf(pcd_file_path_2, PCD_FILE_PATH_FORMAT, pcd_id_source);
  if(pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path_1, *target_pcd_p) == -1)//fail in read //读取第一张pcd文件，存到*source_pcd_p
  {
    sprintf(read_pcd_error, PCD_FILE_READ_ERROR_FORMAT, pcd_id_target);
    PCL_ERROR (read_pcd_error);
    return (-1);
  }
  if(pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path_2, *source_pcd_p) == -1)//fail in read //读取第二张pcd文件，存到*target_pcd_p
  {
    sprintf(read_pcd_error, PCD_FILE_READ_ERROR_FORMAT, pcd_id_source);
    PCL_ERROR (read_pcd_error);
    return (-1);
  }
  *source_pcd_reserve_p = *source_pcd_p;//save source_pcd
    /*
    void_RadiusOutlierRemoval(source_pcd_p, source_pcd_p);
    void_RadiusOutlierRemoval(target_pcd_p, target_pcd_p);
    void_voxel_grid(source_pcd_p, source_pcd_p);
    void_voxel_grid(target_pcd_p, target_pcd_p);
    */
  
  
  /**********icp配准**********/
  int iteration_counts = 1;
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaximumIterations(100); //setMaximumIterations
  icp.setInputSource(source_pcd_p);
  icp.setInputTarget(target_pcd_p);
  icp.setMaxCorrespondenceDistance(200);
  icp.align(*source_pcd_p);
  std::cout << "Applied " << iteration_counts << " ICP iteration(s)"<< std::endl;
  if (icp.hasConverged())//icp.hasConverged ()=1（true）输出变换矩阵的适合性评估
  {
	  std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
	  std::cout << "\nICP transformation " << iteration_counts << " : source_pcd -> target_pcd" << std::endl;
	  transformation_matrix = icp.getFinalTransformation();
	  print4x4Matrix(transformation_matrix);
  }
  else
  {
	  PCL_ERROR("\nICP has not converged.\n");
	  return (-1);
  }
  
  
  /**********点云可视化**********/
    //creat view window
  boost::shared_ptr<pcl::visualization::PCLVisualizer> 
    viewer_pcd(new pcl::visualization::PCLVisualizer ("Viewer_pcd"));
    //设置背景颜色全黑 
  viewer_pcd->setBackgroundColor(0,0,0);
    //设置目标点云
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> 
    target_color (target_pcd_p, 255, 0, 0); //red
  viewer_pcd.addPointCloud<pcl::PointXYZ>(target_pcd_p,target_color,"target_cloud");//add point addPointCloud
  viewer_pcd.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
						  1, "target_cloud"); 
    //设置备份源点云
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> 
    source_reserve_color (source_pcd_reserve_p, 0, 0, 255); //blue
  viewer_pcd.addPointCloud(source_pcd_reserve_p,source_reserve_color,"source_reserve_cloud");
  viewer_pcd.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
						  1, "source_reserve_cloud");
    //设置源点云
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> 
    source_color (source_pcd_p, 0, 255, 0); //green
  viewer_pcd.addPointCloud<pcl::PointXYZ>(source_pcd_p,source_color,"source_cloud");//add point addPointCloud
  viewer_pcd.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
						  1, "source_cloud"); 
    //add text box and setBackgroundColor
  viewer_pcd.addText("Blue: Source\nRed: Target; Green: ICP", 10, 15, 16, 
		  1.0, 1.0, 1.0, "color_info"); // white
    //update txt
  std::stringstream ss;
  ss << iteration_counts;
  std::string iterations_cnt = "ICP iterations = " + ss.str();
  viewer_pcd.addText(iterations_cnt, 10, 60, 16, 1.0, 1.0, 1.0, "iterations_cnt");
    //addCoordinateSystem
  viewer_pcd.addCoordinateSystem(1.0);//addCoordinateSystem  
  viewer_pcd.initCameraParameters();
  
  
  /*点击空格后迭代icp*/
  viewer_pcd.registerKeyboardCallback(&KeyboardEventOccurred, (void*)NULL);
  while(!viewer_pcd.wasStopped()) //if viewer_pcd was not stopped
  {
    viewer_pcd.spinOnce();
    if (next_iteration)
    {
      icp.align(*source_pcd_p);
      if (icp.hasConverged())
      {
	printf("\033[11A");  // Go up 11 lines in terminal output.
	printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
	std::cout << "\nICP transformation " << ++iteration_counts << " : source_pcd -> target_pcd" << std::endl;
	transformation_matrix *= icp.getFinalTransformation();  // WARNING /!\ This is not accurate!
	print4x4Matrix(transformation_matrix); //show transformation_matrix
	ss.str("");
	ss << iteration_counts;
	iterations_cnt = "ICP iterations = " + ss.str();
	viewer_pcd.updateText(iterations_cnt, 10, 60, 16, 1.0, 1.0, 1.0, "iterations_cnt");
	viewer_pcd.updatePointCloud(source_pcd_p, source_color);
      }
      else
      {
	PCL_ERROR("\nICP has not converged.\n");
	return (-1);
      }
    }
    next_iteration = false;
    //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  } 
  
  return 0; 
}


/* show matrix
 * void print4x4Matrix(const Eigen::Matrix<float,4,4> & matrix)
 * para matrix:const Eigen::Matrix<float,4,4> &
 */ 
void print4x4Matrix(const Eigen::Matrix<float,4,4>& matrix)
{
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}


/* KeyboardEvent callback
 * void KeyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,void* nothing)
 * para event: const pcl::visualization::KeyboardEvent&
 */
void KeyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,void* nothing)
{
	if (event.getKeySym() == "space" && event.keyDown())
		next_iteration = true;
}

