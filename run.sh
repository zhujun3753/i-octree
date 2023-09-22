mkdir build
cd build
cmake ..  -Wno-dev 
make

cd ..
# # ./build/example1 data/scan_001_points.dat
./build/time_compare

