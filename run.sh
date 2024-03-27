# mkdir build
# cd build
# cmake ..  -Wno-dev 
# make

# cd ..
# ./build/time_compare

cd octree_map
# Update the `CMAKE_PREFIX_PATH` variable in `CMakeLists.txt` to reflect your own path settings.!!!!
bash build.sh

cd ..
python demo.py