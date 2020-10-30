#cmake版本要求
cmake_minimum_required(VERSION 3.1)

#编译要求
add_compile_options(-std=c++11)

#项目名称
project(v2t_homework_3)

#设置编译
set(CMAKE_BUILD_TYPE "Debug")

#package
find_package(PCL 1.2 REQUIRED)

#头文件
include_directories(.)
include_directories(${PCL_INCLUDE_DIRS})

#链接路径
link_directories(${PCL_LIBRARY_DIRS})

#定义
add_definitions(${PCL_DEFINITIONS})

#生成库文件
#add_library(r_c_filter_shared SHARED r_v_filter.cpp)

#message
message(STATUS "pcl_include_dirs : ${PCL_INCLUDE_DIRS}")
message(STATUS "pcl_library_dirs : ${PCL_LIBRARY_DIRS}")
message(STATUS "pcl_defintions : ${PCL_DEFINITIONS}")
message(STATUS "pcl_librarys : ${PCL_LIBRARIES}")

#可执行文件
add_executable(pc_icp pc_icp.cpp)

#链接
target_link_libraries (pc_icp ${PCL_LIBRARIES})




