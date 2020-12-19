find_package(Boost REQUIRED COMPONENTS system chrono)

add_executable(test_bno55 test_bno55.cpp)
target_include_directories(test_bno55 PUBLIC ${PROJECT_SOURCE_DIR}/third_party/BNO055_driver-master)
target_link_libraries(test_bno55 PUBLIC bno055)

add_executable(test_asio_can test_asio_can.cpp)
target_link_libraries(test_asio_can PUBLIC ${Boost_LIBRARIES} pthread)

add_executable(test_bno55_asio test_bno55_asio.cpp)
target_include_directories(test_bno55_asio PUBLIC ${PROJECT_SOURCE_DIR}/third_party/BNO055_driver-master)
target_link_libraries(test_bno55_asio PUBLIC bno055 ${Boost_LIBRARIES} pthread)
