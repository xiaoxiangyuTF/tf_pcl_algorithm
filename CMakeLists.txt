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

################ tf_filter_pcl ################
add_executable(tf_filter_pcl ./tf_filter_pcl/filter_pcl.cpp)
target_link_libraries(tf_filter_pcl ${PCL_LIBRARIES})