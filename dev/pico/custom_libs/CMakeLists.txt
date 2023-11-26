add_library(pins pins.h)
set_target_properties(pins PROPERTIES LINKER_LANGUAGE CXX)

add_library(ir_sensors ir_sensors.h)
set_target_properties(ir_sensors PROPERTIES LINKER_LANGUAGE CXX)

add_library(odom odom.h)
set_target_properties(odom PROPERTIES LINKER_LANGUAGE CXX)