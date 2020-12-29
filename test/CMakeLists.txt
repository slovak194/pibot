
include_directories(
        ${PROJECT_SOURCE_DIR}/third_party/moteus
        ${PROJECT_SOURCE_DIR}/third_party/BNO055_driver-master
)

link_libraries(
        bno055
        ${Boost_LIBRARIES}
        pthread
        nlohmann_json::nlohmann_json
)

add_executable(test_bno55 test_bno55.cpp)

add_executable(test_asio_can test_asio_can.cpp)

add_executable(test_bno55_asio test_bno55_asio.cpp)

add_executable(test_moteus_query test_moteus_query.cpp)

add_executable(test_robot test_robot.cpp ../src/Config.cpp)

target_compile_definitions(test_robot PUBLIC PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

add_executable(test_joystick_asio test_joystick_asio.cpp)

