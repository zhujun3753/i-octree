cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ioctree_demo)


add_compile_options(-std=c++14 )
set(CMAKE_CXX_FLAGS "-std=c++14 -pthread -O3") 
add_definitions(-w)
find_package(PCL 1.8 REQUIRED)


include_directories(
    ${PCL_INCLUDE_DIRS}
    ikd-Tree/ikd-Tree
    octree2
)


add_executable(time_compare examples/time_compare.cpp ikd-Tree/ikd_Tree.cpp)
target_link_libraries(time_compare ${PCL_LIBRARIES} )

