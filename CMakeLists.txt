cmake_minimum_required ( VERSION 2.6 FATAL_ERROR)  
project(tf_pcl_algorithm)
                 
find_package(PCL 1.7 REQUIRED)  

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARIES_DIRS})
add_definitions(${PCL_DEFINITIONS})

################ tf_write_pcl ################
add_executable(tf_write_pcl ./tf_write_pcl/write_pcl.cpp)
target_link_libraries(tf_write_pcl ${PCL_LIBRARIES})

################ tf_read_pcl ################
add_executable(tf_read_pcl ./tf_read_pcl/read_pcl.cpp)
target_link_libraries(tf_read_pcl ${PCL_LIBRARIES})

################ tf_tree_pcl ################
add_executable(tf_tree_pcl ./tf_tree_pcl/tree_pcl.cpp)
target_link_libraries(tf_tree_pcl ${PCL_LIBRARIES})

################ tf_normal_pcl ################
add_executable(tf_normal_pcl ./tf_normal_pcl/normal_pcl.cpp)
target_link_libraries(tf_normal_pcl ${PCL_LIBRARIES})

################ tf_icp_pcl ################
add_executable(tf_icp_pcl ./tf_icp_pcl/icp_pcl.cpp)
target_link_libraries(tf_icp_pcl ${PCL_LIBRARIES})