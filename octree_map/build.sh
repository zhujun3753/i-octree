
if ! test -d build ;then
    mkdir build
fi
cd build
cmake ..
make -j
cd ..