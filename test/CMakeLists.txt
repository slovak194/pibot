add_executable(test_bno55 test_bno55.cpp)

target_include_directories(test_bno55 PUBLIC ${PROJECT_SOURCE_DIR}/third_party/BNO055_driver-master)
target_link_libraries(test_bno55 PUBLIC bno055)

