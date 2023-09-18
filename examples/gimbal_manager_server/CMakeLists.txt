cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(gimbal_manager_server)

add_executable(gimbal_manager_server
    gimbal_manager_server.cpp
)

add_executable(gimbal_manager_client
    gimbal_manager_client.cpp
)

find_package(MAVSDK REQUIRED)

target_link_libraries(gimbal_manager_server
    MAVSDK::mavsdk
)

target_link_libraries(gimbal_manager_client
    MAVSDK::mavsdk
)

if(NOT MSVC)
    add_compile_options(gimbal_manager_server PRIVATE -Wall -Wextra)
else()
    add_compile_options(gimbal_manager_server PRIVATE -WX -W2)
endif()
