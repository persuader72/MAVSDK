git clone https://github.com/mavlink/MAVSDK.git --recurse-submodules -b v1.4.4
cmake -DCMAKE_BUILD_TYPE=Debug -DBUILD_MAVSDK_SERVER=OFF -DBUILD_TESTS=OFF -DMAVLINK_DIALECT=ardupilotmega -DCMAKE_INSTALL_PREFIX=install -Bbuild/default -H.
cmake --build build/default -j8 --target install


tools/generate_from_protos.sh && tools/run-docker.sh tools/fix_style.sh .