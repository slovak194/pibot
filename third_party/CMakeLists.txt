set_source_files_properties(./BNO055_driver-master/bno055.c PROPERTIES LANGUAGE CXX )
set_source_files_properties(./BNO055_driver-master/bno055.h PROPERTIES LANGUAGE CXX )
set_source_files_properties(BNO055_driver-master/bno055_interface.c PROPERTIES LANGUAGE CXX )
set_source_files_properties(BNO055_driver-master/bno055_interface.h PROPERTIES LANGUAGE CXX )

add_library(bno055
        ./BNO055_driver-master/bno055.c
        BNO055_driver-master/bno055_interface.c
#        ./BNO055_driver-master/bno055_support.c
        )

target_link_libraries(bno055 PUBLIC pigpio)

target_compile_definitions(bno055 PUBLIC BNO055_API=1)
