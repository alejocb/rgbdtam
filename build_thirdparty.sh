cd ThirdParty/DBoW2
rm -rf build
rm -rf lib
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j


cd ../../g2o
rm -rf build
rm -rf lib
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j


cd ../../segment
rm segment
make


