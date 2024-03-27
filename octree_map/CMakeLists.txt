cmake_minimum_required (VERSION 3.1 FATAL_ERROR)

project (iOctreepy)


set(CMAKE_PREFIX_PATH /home/xxx/anaconda3/envs/xxx/lib/python3.x/site-packages/torch/share/cmake)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_COMPILER  "/usr/local/cuda/bin/nvcc")
set(CUDA_HOME /usr/local/cuda)
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda")
# set(CMAKE_CXX_STANDARD 14)
set(TORCH_CUDA_ARCH_LIST "8.6")
link_directories(/opt/intel/oneapi/mkl/2022.2.1/lib/intel64)
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-O3 --use_fast_math  --maxrregcount=128 --ptxas-options=-v -std=c++14 --compiler-options -Wall -gencode arch=compute_86,code=sm_86 --expt-relaxed-constexpr)

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

