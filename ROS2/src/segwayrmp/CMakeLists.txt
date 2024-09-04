cmake_minimum_required(VERSION 3.5)
project(segwayrmp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpic  -Wl,-R${PROJECT_SOURCE_DIR}/lib")
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -fpic")
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


message("-- Segway Robtics Notice: This external lib and include path can be changed by users.")

#This setting can be changed by users.
set(EXTERNAL_INCLUDE_PATH ${CMAKE_SOURCE_DIR}/../../../LibAPI/include)
message("-- Segway Robtics Notice: The external inlude path is " ${EXTERNAL_INCLUDE_PATH})

#This setting can be changed by users.
set(EXTERNAL_LIB_PATH ${CMAKE_SOURCE_DIR}/../../../LibAPI/lib)
message("-- Segway Robtics Notice: The external lib path is " ${EXTERNAL_LIB_PATH})


if(${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "aarch64")
    message("-- Segway Robtics Notice: This is aarch64 cpu")
    set(EXTERNAL_LIB ${EXTERNAL_LIB_PATH}/libctrl_arm64-v8a.so)
elseif(${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "x86_64")
	message("-- Segway Robtics Notice: This is x86_64 cpu")
  set(EXTERNAL_LIB ${EXTERNAL_LIB_PATH}/libctrl_x86_64.so)
else()
  message("-- Segway Robotics Notice: The lib_xxx.so is only supported to x86_64 and aarch64,sorry!")
  return()
endif()
message( "-- Segway Robtics Notice: EXTERNAL_LIB name is " ${EXTERNAL_LIB} )

include_directories(
  include
  ${rclcpp_components_INCLUDE_DIRS}
)

ament_auto_add_library(segway_chassis_component SHARED src/robot.cpp)
target_compile_definitions(segway_chassis_component
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
target_link_libraries(segway_chassis_component
  ${EXTERNAL_LIB}
)
rclcpp_components_register_node(segway_chassis_component PLUGIN "robot::Chassis" EXECUTABLE SmartCar)

# add_executable(drive_segway_sample
#   tools/drive_segway_sample.cpp
# )
# target_include_directories(drive_segway_sample PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
# ament_target_dependencies(drive_segway_sample
#   "rclcpp" "std_msgs" "nav_msgs" "sensor_msgs" "geometry_msgs" "segway_msgs" "tf2" "tf2_ros"
# ) 

install(
  FILES
  ${EXTERNAL_LIB}
  DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
