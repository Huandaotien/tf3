# Requirements:
- Ubuntu 20.04.6 LTS / 22.04 LTS

# Build tf3 project:
- Build and install tf3 library:
    ```
    cd tf3
    mkdir -p build
    cd build
    cmake .. -DBUILD_SHARED_LIBS=ON -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_INSTALL_PREFIX=/usr/local
    make -j$(nproc)
    make install
    ```