# Optional arguments to be passed into file
# ADDITIONAL_INCLUDE_DIRS           string  Additional include directories to add to every build target (PX4 uses this).
# BUILD_MAVLINK_INTERFACE_PLUGIN    bool    Build mavlink_interface_plugin (requires mav dependency).
# BUILD_OCTOMAP_PLUGIN              bool    Build the optical map plugin (requires Octomap).
# BUILD_OPTICAL_FLOW_PLUGIN         bool    Build the optical flow plugin (requires OpenCV).
# MAVLINK_HEADER_DIR                string  Location of MAVLink header files. If not provided, this CMakeLists.txt file will
#                                               search the default locations (e.g. ROS) for them. This variable is only required
#                                               if BUILD_MAVLINK_INTERFACE_PLUGIN=TRUE.
# NO_ROS                            bool    Build without any ROS dependencies.

cmake_minimum_required(VERSION 2.8.3)
project(rotors_gazebo_plugins)
#
#if ( ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR} )
#  message( FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt." )
#endif()

#message(STATUS "rotors_gazebo_plugins CMakeLists.txt called.")


# =============================================================================================== #
# ========================== SET DEFAULTS FOR PASSED-IN VARIABLES =============================== #
# =============================================================================================== #

if(NOT DEFINED BUILD_MAVLINK_INTERFACE_PLUGIN)
  message(STATUS "BUILD_MAVLINK_INTERFACE_PLUGIN variable not provided, setting to FALSE.")
  set(BUILD_MAVLINK_INTERFACE_PLUGIN FALSE)
endif()

if(NOT DEFINED BUILD_OCTOMAP_PLUGIN)
  message(STATUS "BUILD_OCTOMAP_PLUGIN variable not provided, setting to FALSE.")
  set(BUILD_OCTOMAP_PLUGIN FALSE)
endif()

if(NOT DEFINED BUILD_OPTICAL_FLOW_PLUGIN)
  message(STATUS "BUILD_OPTICAL_FLOW_PLUGIN variable not provided, setting to FALSE.")
  set(BUILD_OPTICAL_FLOW_PLUGIN FALSE)
endif()

if(NOT DEFINED NO_ROS)
  message(STATUS "NO_ROS variable not provided, setting to FALSE.")
  set(NO_ROS FALSE)
endif()

# Add any additional include directories as specified by the calling process (either user or another CMake file).
# ASL: Doesn't use this, catkin manages the mav_comm dependency
# PX4: Provides include directory for mav_msgs_rotors, so that "mav_msgs_rotors/default_topics.h" can be found and used.
include_directories(${ADDITIONAL_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
message(STATUS "CMAKE_BINARY_DIR = ${CMAKE_BINARY_DIR}")

# Initialise a list which will keep track of all targets
# that need to be installed.
set(targets_to_install "")

# =============================================================================================== #
# ======================================== STATUS MESSAGES ====================================== #
# =============================================================================================== #

message(STATUS "ADDITIONAL_INCLUDE_DIRS = ${ADDITIONAL_INCLUDE_DIRS}")

if(BUILD_OCTOMAP_PLUGIN)
  message(STATUS "BUILD_OCTOMAP_PLUGIN = TRUE, building gazebo_octomap_plugin.")
else ()
  message(STATUS "BUILD_OCTOMAP_PLUGIN = FALSE, NOT building gazebo_octomap_plugin.")
endif ()

if(BUILD_OPTICAL_FLOW_PLUGIN)
  message(STATUS "BUILD_OPTICAL_FLOW_PLUGIN = TRUE, building gazebo_optical_flow_plugin.")
else ()
  message(STATUS "BUILD_OPTICAL_FLOW_PLUGIN = FALSE, NOT building gazebo_optical_flow_plugin.")
endif ()

if(NO_ROS)
  message(STATUS "NO_ROS = TRUE, building rotors_gazebo_plugins WITHOUT any ROS dependancies.")
else()
  message(STATUS "NO_ROS = FALSE, building rotors_gazebo_plugins WITH ROS dependancies.")
endif()



# Specify C++11 standard
add_definitions(-std=c++11)

# Provides a compiler flag notifying the preprocessor about
# the MAVLink Interface plugin build status
add_definitions(
  -DMAVLINK_INTERFACE=${BUILD_MAVLINK_INTERFACE_PLUGIN}
)

# To enable assertions when compiled in release mode.
add_definitions(-DROS_ASSERT_ENABLED)

if (NOT NO_ROS)
  find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    cv_bridge
    geometry_msgs
    mav_msgs_rotors
    octomap_msgs
    octomap_ros
    rosbag
    roscpp
    rotors_comm
    rotors_control
    std_srvs
    tf
  )
endif()


message(STATUS "CMAKE_BINARY_DIR = ${CMAKE_BINARY_DIR}")

find_package(Eigen REQUIRED)
find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# if (BUILD_MAVLINK_INTERFACE_PLUGIN)
#   include_directories(${mavros_msgs_INCLUDE_DIRS})
#   include_directories(${libmavconn_INCLUDE_DIRS})
# endif()

# ============================================== #
# =================== PROTOBUF ================= #
# ============================================== #

set(PROTOBUF_IMPORT_DIRS "")
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()
message(STATUS "PROTOBUF_IMPORT_DIRS = " ${PROTOBUF_IMPORT_DIRS})

# protobuf required for publishing/subscribing to Gazebo
# messages
# WARNING: THIS MUST BE RUN AFTER PROTOBUF_IMPORT_DIRS IS SETUP
find_package(Protobuf REQUIRED)
pkg_check_modules(PROTOBUF protobuf)

# Protobuf version check
if ("${PROTOBUF_VERSION}" VERSION_LESS "2.5.0")
  message(FATAL_ERROR "protobuf version: ${PROTOBUF_VERSION} not compatible, must be >= 2.5.0")
endif()

# After running this block of code,
# we should have something similar to
# GAZEBO_MSG_INCLUDE_DIRS = /usr/include/gazebo-7/gazebo/msgs
set(GAZEBO_MSG_INCLUDE_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
  endif()
endforeach()

# Get lists of all .proto files in the msgs directory
file(GLOB msgs msgs/*.proto)

# Finally, generate the .cpp files from the .proto files
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})

# Create a shared library of protobuf messages (.so extension on Linux platforms)
add_library(mav_msgs_rotors SHARED ${PROTO_SRCS})
target_link_libraries(mav_msgs_rotors ${PROTOBUF_LIBRARY} gazebo_msgs)

# This causes mav_msgs_rotors to be linked with every created library in this file from this
# point forward.
# NOTE: This is deprecated, should be using target_link_libraries instead
link_libraries(mav_msgs_rotors)


# ============================================== #
# ==================== CATKIN ================== #
# ============================================== #

if (NOT NO_ROS)
  catkin_package(
    INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS} ${CATKIN_DEVEL_PREFIX}/include/${PACKAGE_NAME}
    LIBRARIES rotors_gazebo_motor_model rotors_gazebo_controller_interface
    CATKIN_DEPENDS cv_bridge geometry_msgs mav_msgs_rotors octomap_msgs octomap_ros rosbag roscpp rotors_comm rotors_control std_srvs tf
    DEPENDS eigen gazebo octomap opencv
    #CFG_EXTRAS rotors_gazebo_plugins.cmake
  )
endif()


# Including GAZEBO_MSG_INCLUDE_DIRS here allows the .cpp files generated by custom .proto files to find
# the .cpp/hpp files generated by the built-in Gazebo .proto files
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_MSG_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})

# =============================================================================================== #
# ========================================= USER LIBRARIES ====================================== #
# =============================================================================================== #

# SORTED IN ALPHABETICAL ORDER (by "plugin" name, keep it this way!)

#set(GAZEBO_LIBRARIES "BulletSoftBodyBulletDynamicsBulletCollisionLinearMath/usr/lib/x86_64-linux-gnu/libSimTKsimbody.so/usr/lib/x86_64-linux-gnu/libSimTKmath.so/usr/lib/x86_64-linux-gnu/libSimTKcommon.so/usr/lib/x86_64-linux-gnu/libblas.so/usr/lib/x86_64-linux-gnu/liblapack.so/usr/lib/x86_64-linux-gnu/libblas.sopthreadrtdlm/usr/lib/x86_64-linux-gnu/libgazebo.so/usr/lib/x86_64-linux-gnu/libgazebo_client.so/usr/lib/x86_64-linux-gnu/libgazebo_gui.so/usr/lib/x86_64-linux-gnu/libgazebo_sensors.so/usr/lib/x86_64-linux-gnu/libgazebo_rendering.so/usr/lib/x86_64-linux-gnu/libgazebo_physics.so/usr/lib/x86_64-linux-gnu/libgazebo_ode.so/usr/lib/x86_64-linux-gnu/libgazebo_transport.so/usr/lib/x86_64-linux-gnu/libgazebo_msgs.so/usr/lib/x86_64-linux-gnu/libgazebo_util.so/usr/lib/x86_64-linux-gnu/libgazebo_common.so/usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so/usr/lib/x86_64-linux-gnu/libgazebo_opcode.so/usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so/usr/lib/x86_64-linux-gnu/libboost_thread.so/usr/lib/x86_64-linux-gnu/libboost_signals.so/usr/lib/x86_64-linux-gnu/libboost_system.so/usr/lib/x86_64-linux-gnu/libboost_filesystem.so/usr/lib/x86_64-linux-gnu/libboost_program_options.so/usr/lib/x86_64-linux-gnu/libboost_regex.so/usr/lib/x86_64-linux-gnu/libboost_iostreams.so/usr/lib/x86_64-linux-gnu/libboost_date_time.so/usr/lib/x86_64-linux-gnu/libboost_chrono.so/usr/lib/x86_64-linux-gnu/libboost_atomic.so/usr/lib/x86_64-linux-gnu/libprotobuf.so-lpthread/usr/lib/x86_64-linux-gnu/libsdformat.so/usr/lib/x86_64-linux-gnu/libOgreMain.sodebug/usr/lib/x86_64-linux-gnu/libOgreMain.so/usr/lib/x86_64-linux-gnu/libboost_thread.so/usr/lib/x86_64-linux-gnu/libboost_date_time.so/usr/lib/x86_64-linux-gnu/libboost_system.so/usr/lib/x86_64-linux-gnu/libboost_atomic.so/usr/lib/x86_64-linux-gnu/libboost_chrono.so/usr/lib/x86_64-linux-gnu/libboost_chrono.sooptimized/usr/lib/x86_64-linux-gnu/libOgreTerrain.sodebug/usr/lib/x86_64-linux-gnu/libOgreTerrain.sooptimized/usr/lib/x86_64-linux-gnu/libOgrePaging.sodebug/usr/lib/x86_64-linux-gnu/libOgrePaging.so")
#set(GAZEBO_LIBRARIES
#    /usr/lib/x86_64-linux-gnu/libgazebo.so
#    /usr/lib/x86_64-linux-gnu/libgazebo_client.so
#    /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
#    /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
#    /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
#    /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
#    /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
#    /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
#    /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
#    /usr/lib/x86_64-linux-gnu/libgazebo_util.so
#    /usr/lib/x86_64-linux-gnu/libgazebo_common.so
#    /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
#    /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
#    /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
#    /usr/lib/x86_64-linux-gnu/libgazebo_math.so
#    /usr/lib/x86_64-linux-gnu/libgazebo_ccd.so
#    /usr/lib/x86_64-linux-gnu/libboost_thread.so
#    /usr/lib/x86_64-linux-gnu/libboost_signals.so
#    /usr/lib/x86_64-linux-gnu/libboost_system.so
#    /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
#    /usr/lib/x86_64-linux-gnu/libboost_program_options.so
#    /usr/lib/x86_64-linux-gnu/libboost_regex.so
#    /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
#    /usr/lib/x86_64-linux-gnu/libboost_date_time.so
#    /usr/lib/x86_64-linux-gnu/libboost_chrono.so
#    /usr/lib/x86_64-linux-gnu/libboost_atomic.so
#    /usr/lib/x86_64-linux-gnu/libpthread.so
#    /usr/lib/x86_64-linux-gnu/libprotobuf.so
#    /usr/lib/x86_64-linux-gnu/libprotobuf.so
#    -lpthread
#    /usr/lib/x86_64-linux-gnu/libsdformat.so
#    /usr/lib/x86_64-linux-gnu/libignition-math2.so
#    /usr/lib/x86_64-linux-gnu/libOgreMain.so
#    /usr/lib/x86_64-linux-gnu/libOgreMain.so
#    /usr/lib/x86_64-linux-gnu/libboost_thread.so
#    /usr/lib/x86_64-linux-gnu/libboost_date_time.so
#    /usr/lib/x86_64-linux-gnu/libboost_system.so
#    /usr/lib/x86_64-linux-gnu/libboost_atomic.so
#    /usr/lib/x86_64-linux-gnu/libboost_chrono.so
#    /usr/lib/x86_64-linux-gnu/libpthread.so
#    /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
#    /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
#    /usr/lib/x86_64-linux-gnu/libOgrePaging.so
#    /usr/lib/x86_64-linux-gnu/libOgrePaging.so
#    /usr/lib/x86_64-linux-gnu/libignition-math2.so
#    )

set(GAZEBO_LIBRARIES "")

find_library(GAZEBO_LIBRARIES
        NAMES
                libgazebo.so
                libgazebo_client.so
                libgazebo_gui.so
                libgazebo_sensors.so
                libgazebo_rendering.so
                libgazebo_physics.so
                libgazebo_ode.so
                libgazebo_transport.so
                libgazebo_msgs.so
                libgazebo_util.so
                libgazebo_common.so
                libgazebo_gimpact.so
                libgazebo_opcode.so
                libgazebo_opende_ou.so
                libgazebo_math.so
                libgazebo_ccd.so
                libboost_thread.so
                libboost_signals.so
                libboost_system.so
                libboost_filesystem.so
                libboost_program_options.so
                libboost_regex.so
                libboost_iostreams.so
                libboost_date_time.so
                libboost_chrono.so
                libboost_atomic.so
                libpthread.so
                libprotobuf.so
                libprotobuf.so
                libpthread.so
                libsdformat.so
                libignition-math2.so
                libOgreMain.so
                libOgreMain.so
                libboost_thread.so
                libboost_date_time.so
                libboost_system.so
                libboost_atomic.so
                libboost_chrono.so
                libpthread.so
                libOgreTerrain.so
                libOgreTerrain.so
                libOgrePaging.so
                libOgrePaging.so
                libignition-math2.so
        PATHS
                /usr/lib/x86_64-linux-gnu
)
#message(STATUS "++++++++++++++++++++++++++++++++++++ CATKIN LIBS:" ${catkin_LIBRARIES})
#set(catkin_LIBRARIES "/opt/ros/melodic/lib/libcv_bridge.so/usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0/usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0/usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0octomap_ros/opt/ros/melodic/lib/liboctomap.so/opt/ros/melodic/lib/liboctomath.so/opt/ros/melodic/lib/librosbag.so/opt/ros/melodic/lib/librosbag_storage.so/opt/ros/melodic/lib/libclass_loader.so/usr/lib/libPocoFoundation.so/usr/lib/x86_64-linux-gnu/libdl.so/opt/ros/melodic/lib/libroslib.so/opt/ros/melodic/lib/librospack.so/usr/lib/x86_64-linux-gnu/libpython2.7.so/usr/lib/x86_64-linux-gnu/libboost_program_options.so/usr/lib/x86_64-linux-gnu/libtinyxml2.so/opt/ros/melodic/lib/libroslz4.so/usr/lib/x86_64-linux-gnu/liblz4.so/opt/ros/melodic/lib/libtopic_tools.solee_position_controllerroll_pitch_yawrate_thrust_controller/opt/ros/melodic/lib/libtf.so/opt/ros/melodic/lib/libtf2_ros.so/opt/ros/melodic/lib/libactionlib.so/opt/ros/melodic/lib/libmessage_filters.so/opt/ros/melodic/lib/libroscpp.so/usr/lib/x86_64-linux-gnu/libboost_filesystem.so/usr/lib/x86_64-linux-gnu/libboost_signals.so/opt/ros/melodic/lib/libxmlrpcpp.so/opt/ros/melodic/lib/libtf2.so/opt/ros/melodic/lib/libroscpp_serialization.so/opt/ros/melodic/lib/librosconsole.so/opt/ros/melodic/lib/librosconsole_log4cxx.so/opt/ros/melodic/lib/librosconsole_backend_interface.so/usr/lib/x86_64-linux-gnu/liblog4cxx.so/usr/lib/x86_64-linux-gnu/libboost_regex.so/opt/ros/melodic/lib/librostime.so/opt/ros/melodic/lib/libcpp_common.so/usr/lib/x86_64-linux-gnu/libboost_system.so/usr/lib/x86_64-linux-gnu/libboost_thread.so/usr/lib/x86_64-linux-gnu/libboost_chrono.so/usr/lib/x86_64-linux-gnu/libboost_date_time.so/usr/lib/x86_64-linux-gnu/libboost_atomic.so/usr/lib/x86_64-linux-gnu/libpthread.so/usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4")

#========================================= BAG PLUGIN ===========================================//
# Entire GazeboBagPlugin is a heavy ROS dependency, and so rather than passing messages to
# GazeboRosInterfacePlugin, this entire library is only included if ROS is present.
if (NOT NO_ROS)
  add_library(rotors_gazebo_bag_plugin SHARED src/gazebo_bag_plugin.cpp)
  target_link_libraries(rotors_gazebo_bag_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
  add_dependencies(rotors_gazebo_bag_plugin ${catkin_EXPORTED_TARGETS})
  list(APPEND targets_to_install rotors_gazebo_bag_plugin)
endif()

#================================= CONTROLLER INTERFACE PLUGIN ==================================//
add_library(rotors_gazebo_controller_interface SHARED src/gazebo_controller_interface.cpp)
target_link_libraries(rotors_gazebo_controller_interface ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
if (NOT NO_ROS)
  add_dependencies(rotors_gazebo_controller_interface ${catkin_EXPORTED_TARGETS})
endif()
list(APPEND targets_to_install rotors_gazebo_controller_interface)

#=================================== GEOTAGGED IMAGES PLUGIN ====================================//

message(STATUS "GAZEBO ${gazebo_VERSION_MAJOR}.${gazebo_VERSION_MINOR}.${gazebo_VERSION_PATCH}")
message(STATUS "${GAZEBO_VERSION_MAJOR}")

if(${gazebo_VERSION_MAJOR} GREATER 4)
  add_library(gazebo_geotagged_images_plugin SHARED src/external/gazebo_geotagged_images_plugin.cpp)
  target_link_libraries(gazebo_geotagged_images_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
  if (NOT NO_ROS)
    add_dependencies(gazebo_geotagged_images_plugin ${catkin_EXPORTED_TARGETS})
  endif()
  list(APPEND targets_to_install gazebo_geotagged_images_plugin)
else()
  message(STATUS "Gazebo version is less than 5, not building gazebo_geotagged_images_plugin.cpp.")
endif()

#========================================= GPS PLUGIN ===========================================//
add_library(rotors_gazebo_gps_plugin SHARED src/gazebo_gps_plugin.cpp)
target_link_libraries(rotors_gazebo_gps_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
if (NOT NO_ROS)
  add_dependencies(rotors_gazebo_gps_plugin ${catkin_EXPORTED_TARGETS})
endif()
list(APPEND targets_to_install rotors_gazebo_gps_plugin)

#========================================= IMU PLUGIN ===========================================//
add_library(rotors_gazebo_imu_plugin SHARED src/gazebo_imu_plugin.cpp)
target_link_libraries(rotors_gazebo_imu_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
if (NOT NO_ROS)
  add_dependencies(rotors_gazebo_imu_plugin ${catkin_EXPORTED_TARGETS})
endif()
list(APPEND targets_to_install rotors_gazebo_imu_plugin)

#========================================= TRAJECTORY PLUGIN ===========================================//
add_library(gazebo_plot_trajectory SHARED src/gazebo_plot_trajectory.cpp)
target_link_libraries(gazebo_plot_trajectory ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
if (NOT NO_ROS)
  add_dependencies(gazebo_plot_trajectory ${catkin_EXPORTED_TARGETS})
endif()
list(APPEND targets_to_install gazebo_plot_trajectory)

#======================================== LIDAR PLUGIN ==========================================//
if(${gazebo_VERSION_MAJOR} GREATER 4)
  add_library(rotors_gazebo_lidar_plugin SHARED src/external/gazebo_lidar_plugin.cpp)
  target_link_libraries(rotors_gazebo_lidar_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
  if (NOT NO_ROS)
    add_dependencies(rotors_gazebo_lidar_plugin ${catkin_EXPORTED_TARGETS})
  endif()
  list(APPEND targets_to_install rotors_gazebo_lidar_plugin)
else()
  message(STATUS "Gazebo version is less than 5, not building gazebo_lidar_plugin.cpp.")
endif()

#===================================== MAGNETOMETER PLUGIN ======================================//
add_library(rotors_gazebo_magnetometer_plugin SHARED src/gazebo_magnetometer_plugin.cpp)
target_link_libraries(rotors_gazebo_magnetometer_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
if (NOT NO_ROS)
  add_dependencies(rotors_gazebo_magnetometer_plugin ${catkin_EXPORTED_TARGETS})
endif()
list(APPEND targets_to_install rotors_gazebo_magnetometer_plugin)

#================================= MAVLINK INTERFACE PLUGIN =====================================//

if(BUILD_MAVLINK_INTERFACE_PLUGIN)
  message(STATUS "BUILD_MAVLINK_INTERFACE_PLUGIN = TRUE, adding mavros dependency and building mavlink_interface_plugin.")
else ()
  message(STATUS "BUILD_MAVLINK_INTERFACE_PLUGIN = FALSE, NOT adding mavros dependency and NOT building mavlink_interface_plugin.")
endif ()

if (BUILD_MAVLINK_INTERFACE_PLUGIN)

  # We need the MAVLink headers.
  set(MAVLINK_HEADERS_FOUND FALSE)
  
  # First, check to see if MAVLink headers were passed in as variable
  if(NOT ${MAVLINK_HEADER_DIR} STREQUAL "")
  
    message(STATUS "MAVLINK_HEADER_DIR provided as '${MAVLINK_HEADER_DIR}'.")
    include_directories(${MAVLINK_HEADER_DIR})
    set(MAVLINK_HEADERS_FOUND TRUE)
    
  else()
  
    message(STATUS "MAVLINK_HEADER_DIR not provided, falling back to looking at default paths...")
    # If ROS indigo is installed, we should be able to find them at the path below
    if(EXISTS /opt/ros/indigo/include/mavlink/v1.0/)
      message(STATUS "Found MAVLink headers at '/opt/ros/indigo/include/mavlink/v1.0/'.")
      include_directories("/opt/ros/indigo/include/mavlink/v1.0/")
      set(MAVLINK_HEADERS_FOUND TRUE)
    endif()
    
  endif()
  
  if(NOT MAVLINK_HEADERS_FOUND)
    message(FATAL_ERROR "MAVLink headers were not found. They are required for building MavlinkInterfacePlugin.")
  endif()

  # Note that this library includes TWO .cpp files.
  add_library(rotors_gazebo_mavlink_interface SHARED src/gazebo_mavlink_interface.cpp src/geo_mag_declination.cpp)
  target_link_libraries(rotors_gazebo_mavlink_interface ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${mav_msgs_rotors})
  #add_dependencies(rotors_gazebo_mavlink_interface ${catkin_EXPORTED_TARGETS} ${mavros_EXPORTED_TARGETS} ${mavros_msgs_EXPORTED_TARGETS})
  list(APPEND targets_to_install rotors_gazebo_mavlink_interface)
endif()

#==================================== MOTOR MODEL PLUGIN ========================================//
add_library(rotors_gazebo_motor_model SHARED src/gazebo_motor_model.cpp)
target_link_libraries(rotors_gazebo_motor_model ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
if (NOT NO_ROS)
  add_dependencies(rotors_gazebo_motor_model ${catkin_EXPORTED_TARGETS})
endif()
list(APPEND targets_to_install rotors_gazebo_motor_model)

#==================================== MULTIROTOR BASE PLUGIN ====================================//
add_library(rotors_gazebo_multirotor_base_plugin SHARED src/gazebo_multirotor_base_plugin.cpp)
target_link_libraries(rotors_gazebo_multirotor_base_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
if (NOT NO_ROS)
  add_dependencies(rotors_gazebo_multirotor_base_plugin ${catkin_EXPORTED_TARGETS})
endif()
list(APPEND targets_to_install rotors_gazebo_multirotor_base_plugin)

#====================================== OCTOMAP PLUGIN ==========================================//

# Conditionally built since it requires Octomap as a dependency
# ASL uses this, PX4 does not
if(BUILD_OCTOMAP_PLUGIN)
  find_package(octomap REQUIRED)
  add_library(rotors_gazebo_octomap_plugin SHARED src/gazebo_octomap_plugin.cpp)
  target_link_libraries(rotors_gazebo_octomap_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
  if (NOT NO_ROS)
    add_dependencies(rotors_gazebo_octomap_plugin ${catkin_EXPORTED_TARGETS})
  endif()
  list(APPEND targets_to_install rotors_gazebo_octomap_plugin)
endif()

#======================================= ODOMETRY PLUGIN ========================================//
add_library(rotors_gazebo_odometry_plugin SHARED src/gazebo_odometry_plugin.cpp)
target_link_libraries(rotors_gazebo_odometry_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${OpenCV_LIBRARIES})
if (NOT NO_ROS)
  add_dependencies(rotors_gazebo_odometry_plugin ${catkin_EXPORTED_TARGETS})
endif()
list(APPEND targets_to_install rotors_gazebo_odometry_plugin)

#===================================== OPTICAL FLOW PLUGIN ======================================//
# Since the optical flow plugin depends on external code (PX4/OpticalFlow), this is
# only conditionally built
if(BUILD_OPTICAL_FLOW_PLUGIN)

  # Check to make sure the correct Gazebo version is available.
  if(${gazebo_VERSION_MAJOR} LESS 5)
    message(FATAL_ERROR "Gazebo version needs to be >= v5.x. You specified BUILD_OPTICAL_FLOW_PLUGIN=TRUE, but Gazebo version was less than v5.x.")
  endif()

  find_package(Boost 1.40.0 REQUIRED timer)

  # OpticalFlow needs to be built and installed by the calling script, as it is very
  # unlikely that this package already exists on the users computer.
  # In PX4 this is done with a call to ExternalProject_Add().
  find_package(OpticalFlow REQUIRED)

  add_library(rotors_gazebo_optical_flow_plugin SHARED src/external/gazebo_optical_flow_plugin.cpp)
  target_include_directories(rotors_gazebo_optical_flow_plugin PUBLIC ${OpticalFlow_INCLUDE_DIRS})
  target_link_libraries(rotors_gazebo_optical_flow_plugin
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpticalFlow_LIBRARIES})
  if (NOT NO_ROS)
    add_dependencies(rotors_gazebo_optical_flow_plugin ${catkin_EXPORTED_TARGETS})
  endif()
  list(APPEND targets_to_install rotors_gazebo_optical_flow_plugin)
endif()

#===================================== ROS INTERFACE PLUGIN =====================================//
# This entire plugin is only built if ROS is a dependency
if (NOT NO_ROS)
  add_library(rotors_gazebo_ros_interface_plugin SHARED src/gazebo_ros_interface_plugin.cpp)
  target_link_libraries(rotors_gazebo_ros_interface_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
  add_dependencies(rotors_gazebo_ros_interface_plugin ${catkin_EXPORTED_TARGETS})
  list(APPEND targets_to_install rotors_gazebo_ros_interface_plugin)
endif()

#========================================= WIND PLUGIN ==========================================//
add_library(rotors_gazebo_wind_plugin SHARED src/gazebo_wind_plugin.cpp)
target_link_libraries(rotors_gazebo_wind_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
if (NOT NO_ROS)
  add_dependencies(rotors_gazebo_wind_plugin ${catkin_EXPORTED_TARGETS})
endif()
list(APPEND targets_to_install rotors_gazebo_wind_plugin)

#========================================= WIND GUST PLUGIN ==========================================//
add_library(rotors_gazebo_wind_gust_plugin SHARED src/gazebo_wind_gust_plugin.cpp)
target_link_libraries(rotors_gazebo_wind_gust_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
if (NOT NO_ROS)
  add_dependencies(rotors_gazebo_wind_gust_plugin ${catkin_EXPORTED_TARGETS})
endif()
list(APPEND targets_to_install rotors_gazebo_wind_gust_plugin)

#========================================= RANDOM POSITION PLUGIN ==========================================//
add_library(rotors_gazebo_random_position_plugin SHARED src/gazebo_random_position_plugin.cpp)
target_link_libraries(rotors_gazebo_random_position_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
if (NOT NO_ROS)
  add_dependencies(rotors_gazebo_random_position_plugin ${catkin_EXPORTED_TARGETS})
endif()
list(APPEND targets_to_install rotors_gazebo_random_position_plugin)

#========================================= ANGULAR VELOCITY PLUGIN ==========================================//
add_library(rotors_gazebo_angular_vel_plugin SHARED src/gazebo_angular_vel_plugin.cpp)
target_link_libraries(rotors_gazebo_angular_vel_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
if (NOT NO_ROS)
  add_dependencies(rotors_gazebo_angular_vel_plugin ${catkin_EXPORTED_TARGETS})
endif()
list(APPEND targets_to_install rotors_gazebo_angular_vel_plugin)

# =============================================================================================== #
# ======================================= EXTERNAL LIBRARIES ==================================== #
# =============================================================================================== #

# liftdrag_plugin is provided by Gazebo, but not guaranteed to be on system.
# Naming has not been changed to match rotors convetion, due to this being an external
# code soure.
# Linux is not consistent with plugin availability, even on Gazebo 7
#if("${GAZEBO_VERSION}" VERSION_LESS "7.0")
if(${gazebo_VERSION_MAJOR} GREATER 4)
  add_library(LiftDragPlugin SHARED src/liftdrag_plugin/liftdrag_plugin.cpp)
  target_link_libraries(LiftDragPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
  if (NOT NO_ROS)
    add_dependencies(LiftDragPlugin ${catkin_EXPORTED_TARGETS})
  endif()
  list(APPEND targets_to_install LiftDragPlugin)
else()
  message(STATUS "Gazebo version is less than 5, not building liftdrag_plugin.cpp.")
endif()



message(STATUS "CMAKE_INSTALL_PREFIX = ${CMAKE_INSTALL_PREFIX}")
if (NOT NO_ROS)
  set(BIN_DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  set(LIB_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
else()
  set(BIN_DESTINATION ${CMAKE_BINARY_DIR})
  set(LIB_DESTINATION ${CMAKE_BINARY_DIR})
endif()


# Install all targets that were defined above (some conditionally) and added to the
# variable targets_to_install
install(
  TARGETS
    ${targets_to_install}
  DESTINATION ${BIN_DESTINATION}
  LIBRARY DESTINATION ${LIB_DESTINATION}
)

# Copy over all converted protobuf files (*.pb.h and *.pb.c) from the src directory to
# the devel directory. This is required so that you can include the protobuf header files from other C++ files.
if (NOT NO_ROS)
  file(COPY ${CMAKE_CURRENT_BINARY_DIR}/ DESTINATION ${CATKIN_DEVEL_PREFIX}/include/${PACKAGE_NAME} FILES_MATCHING PATTERN "*.pb.*")
endif()

#message(FATAL_ERROR "Reached EOF.")
