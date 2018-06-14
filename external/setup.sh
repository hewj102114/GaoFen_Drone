#download cmake - we need v3.9+ which is not out of box in Ubuntu 16.04
if [[ ! -d "cmake_build/bin" ]]; then
    echo "Downloading cmake..."
    wget https://cmake.org/files/v3.10/cmake-3.10.2.tar.gz \
        -O cmake.tar.gz
    tar -xzf cmake.tar.gz
    rm cmake.tar.gz
    rm -rf ./cmake_build
    mv ./cmake-3.10.2 ./cmake_build
    pushd cmake_build
    ./bootstrap
    make
    popd
fi


CMAKE="$(readlink -f cmake_build/bin/cmake)"
export CC="gcc"
export CXX="g++"

build_dir=build


if [[ -f "./build/CMakeCache.txt" ]]; then
    rm "./build/CMakeCache.txt"
fi
if [[ -d "./build/CMakeFiles" ]]; then
    rm -rf "./build/CMakeFiles"
fi

if [[ ! -d $build_dir ]]; then
    mkdir $build_dir   
fi

cd $build_dir
"$CMAKE" ../ -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX=../output
make
make install

cd ../
cp -r output/include/* ../src/airsim-ros/include
cp -r eigen3/* ../src/airsim-ros/include
cp -r output/lib/*.a ../src/airsim-ros/lib