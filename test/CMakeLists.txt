
include_directories(
        ${PROJECT_SOURCE_DIR}/third_party/moteus
        ${PROJECT_SOURCE_DIR}/third_party/BNO055_driver-master
)

link_libraries(
        bno055
        ${Boost_LIBRARIES}
        pthread
        nlohmann_json::nlohmann_json
        zmq
)

add_executable(test_imu test_imu.cpp ../src/Imu.cpp)

add_executable(test_asio_can test_asio_can.cpp)

#add_executable(test_bno55_asio test_bno55_asio.cpp)

add_executable(test_moteus_query test_moteus_query.cpp)

add_executable(test_robot
        test_robot.cpp
        ../src/Config.cpp
        ../src/Controller.cpp
        ../src/Imu.cpp
        )

target_compile_definitions(test_robot PUBLIC PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
target_compile_definitions(test_robot PUBLIC USE_ZMQ=1)

add_executable(test_joystick_asio test_joystick_asio.cpp)
