#include <iostream>
#include <string>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

using namespace std;
using namespace pcl;

//路径名设置
#define PCD_FILE_PATH_FORMAT "../used_file/%d.pcd"
#define PCD_FILE_READ_ERROR_FORMAT "Couldn't read file %d.pcd \n"

//show Matrix 4*4
void print4x4Matrix(const Eigen::Matrix<float,4,4>& matrix);

//size
#define MAX_FILES_GET 10


/*
 * 主函数
 * show icp
 */
int main(int argc, char* argv[])
{
  /**********创建指向点云的指针 指针类型为pcl::PointCloud<pcl::PointXYZ>::Ptr，同时创建两个点云对象。**********/
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_pcd_p(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_pcd_p(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_pcd_p(new pcl::PointCloud<pcl::PointXYZ>);;
 
  
  /**********icp配准**********/
    //icp对象
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaxCorrespondenceDistance(10);
  std::vector< Eigen::Matrix<float,4,4> > icp_tansfor_matrix_buf;
  Eigen::Matrix<float,4,4> transformation_matrix;
  
  
  /**********设置路径读取两张pcd文件**********///后来者逼近前者,保存source
  int pcd_id_target = 1, pcd_id_source = 2;
  char pcd_file_path_1[200], pcd_file_path_2[200], read_pcd_error[200];
  std::cout << "Start append point cloude...." << std::endl << std::endl;
  for(int i = 1; i < MAX_FILES_GET; i++)
  {
      //set file path
    pcd_id_target = i;
    pcd_id_source = i + 1;
    sprintf(pcd_file_path_1, PCD_FILE_PATH_FORMAT, pcd_id_target);
    sprintf(pcd_file_path_2, PCD_FILE_PATH_FORMAT, pcd_id_source);
     
      //read target pcd file read source pcd file
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path_1, *target_pcd_p) == -1)//fail in read //读取第一张pcd文件，存到*source_pcd_p
    {
      sprintf(read_pcd_error, PCD_FILE_READ_ERROR_FORMAT, pcd_id_target);
      PCL_ERROR (read_pcd_error);
      return (-1);
    }
    if(1 == i)
      *output_pcd_p += *target_pcd_p;
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path_2, *source_pcd_p) == -1)//fail in read //读取第二张pcd文件，存到*target_pcd_p
    {
      sprintf(read_pcd_error, PCD_FILE_READ_ERROR_FORMAT, pcd_id_source);
      PCL_ERROR (read_pcd_error);
      return (-1);
    }
      
      //set up icp target and source 
    icp.setInputSource(source_pcd_p);
    icp.setInputTarget(target_pcd_p);
    icp.align(*source_pcd_p);
    if (icp.hasConverged())//icp.hasConverged ()=1（true）输出变换矩阵的适合性评估
    {
      std::cout << "\nICP " << i + 1 << " has converged, score is " << icp.getFitnessScore() << std::endl;
      transformation_matrix = icp.getFinalTransformation();
      icp_tansfor_matrix_buf.push_back(transformation_matrix);
      print4x4Matrix(transformation_matrix);
    }
    else
    {
      PCL_ERROR("\nICP has not converged.\n");
      return (-1);
    }
    
      //append
    *output_pcd_p += *source_pcd_p;
    std::cout << "\nAppended PCD file " << i + 1 << std::endl;
  }
  
  
  /**********点云可视化**********/
    //creat view window
  pcl::visualization::PCLVisualizer viewer_pcd("Viewer_pcd");
    //设置背景颜色全黑 
  viewer_pcd.setBackgroundColor(0,0,0);
    //设置目标点云颜色
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> 
    output_color (output_pcd_p, 255, 0, 0); //red
  viewer_pcd.addPointCloud<pcl::PointXYZ>(output_pcd_p,output_color,"output_cloud");//add point addPointCloud
  viewer_pcd.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"output_cloud"); 
  viewer_pcd.addCoordinateSystem(1.0);//addCoordinateSystem  
  viewer_pcd.initCameraParameters();


  /**********save output pcd file**********/
  pcl::io::savePCDFileASCII ("../output_appended.pcd", *output_pcd_p);


  /**********show output point cloud**********/
  while(!(viewer_pcd.wasStopped())) //if viewer_pcd was not stopped
  {
    viewer_pcd.spinOnce();
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
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
