cmake_minimum_required(VERSION 3.13)
project(pibot)

set(CMAKE_CXX_STANDARD 20)

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wno-psabi")

find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache) # Less useful to do it for linking, see edit2
endif(CCACHE_FOUND)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system chrono regex)
find_package(nlohmann_json REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIR}
)

add_subdirectory(test)
add_subdirectory(third_party)
