cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(remote_id)

find_package(MAVSDK REQUIRED)

add_executable(remote_id
    remote_id.cpp
)

target_link_libraries(remote_id
    MAVSDK::mavsdk
)

if(NOT MSVC)
    add_compile_options(remote_id PRIVATE -Wall -Wextra)
else()
    add_compile_options(remote_id PRIVATE -WX -W2)
endif()
