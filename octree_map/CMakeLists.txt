cmake_minimum_required (VERSION 3.1 FATAL_ERROR)

project (iOctreepy)


set(CMAKE_PREFIX_PATH /home/xxx/anaconda3/envs/xxx/lib/python3.x/site-packages/torch/share/cmake)
set(CMAKE_CXX_STANDARD 17)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
find_package(Torch REQUIRED)
add_definitions(-w)

#* ioctree
find_package(Eigen3)
include_directories(
    .
    ${EIGEN3_INCLUDE_DIR}
    )

add_library(octree_map SHARED octree_map.cpp)
target_link_libraries(octree_map ${TORCH_LIBRARIES} )

