cmake_minimum_required(VERSION 3.13)
project(pibot)

set(CMAKE_CXX_STANDARD 20)


#set(CMAKE_PREFIX_PATH "/usr/lib/cmake")

#message(DEBUG ${CMAKE_PREFIX_PATH})

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system chrono regex)
find_package(nlohmann_json REQUIRED)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
)

add_subdirectory(test)
add_subdirectory(third_party)
