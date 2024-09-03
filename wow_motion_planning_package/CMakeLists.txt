cmake_minimum_required(VERSION 3.5)
project(wow_motion_planning_package)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  tf2_geometry_msgs
  tf2_ros
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wow_motion_planning_package
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    tf2_geometry_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/wow_motion_planning_package.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/wow_motion_planning_package_node.cpp)


add_executable(motion_planning_pipeline_tutorial src/motion_planning_pipeline_tutorial.cpp)
target_link_libraries(motion_planning_pipeline_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS motion_planning_pipeline_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(path_planning src/path_planning.cpp)
target_link_libraries(path_planning ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS path_planning DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(get_trajectory src/get_trajectory.cpp)
target_link_libraries(get_trajectory ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS get_trajectory DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})




if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  # Default to installing in SOEM source directory
  set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_LIST_DIR}/src/EtherCAT_SOEM/install)
endif()

set(SOEM_INCLUDE_INSTALL_DIR include/soem)
set(SOEM_LIB_INSTALL_DIR lib)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(BUILD_TESTS TRUE)
else()
  message(STATUS "SOEM: not building tests when built as dependency")
  set(BUILD_TESTS FALSE)
endif()

if(WIN32)
  set(OS "win32")
  include_directories(oshw/win32/wpcap/Include)
  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    link_directories(${CMAKE_CURRENT_LIST_DIR}/src/EtherCAT_SOEM/oshw/win32/wpcap/Lib/x64)
  elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
    link_directories(${CMAKE_CURRENT_LIST_DIR}/src/EtherCAT_SOEM/oshw/win32/wpcap/Lib)
  endif()
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /D _CRT_SECURE_NO_WARNINGS")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  /WX")
  set(OS_LIBS wpcap.lib Packet.lib Ws2_32.lib Winmm.lib)
elseif(UNIX AND NOT APPLE)
  set(OS "linux")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
  set(OS_LIBS pthread rt)
elseif(APPLE)
  # This must come *before* linux or MacOSX will identify as Unix.
  set(OS "macosx")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
  set(OS_LIBS pthread pcap)
elseif(${CMAKE_SYSTEM_NAME} MATCHES "rt-kernel")
  set(OS "rtk")
  message(STATUS "ARCH is ${ARCH}")
  message(STATUS "BSP is ${BSP}")
  include_directories(oshw/${OS}/${ARCH})
  file(GLOB OSHW_EXTRA_SOURCES oshw/${OS}/${ARCH}/*.c)
  set(OSHW_SOURCES "${OS_HW_SOURCES} ${OSHW_ARCHSOURCES}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-but-set-variable")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-function")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-format")
  set(OS_LIBS "-Wl,--start-group -l${BSP} -l${ARCH} -lkern -ldev -lsio -lblock -lfs -lusb -llwip -leth -li2c -lrtc -lcan -lnand -lspi -lnor -lpwm -ladc -ltrace -lc -lm -Wl,--end-group")
elseif(${CMAKE_SYSTEM_NAME} MATCHES "rtems")
  message(STATUS "Building for RTEMS")
  set(OS "rtems")
  set(SOEM_LIB_INSTALL_DIR ${LIB_DIR})
  set(BUILD_TESTS FALSE)
endif()

message(STATUS "OS is ${OS}")

file(GLOB SOEM_SOURCES src/EtherCAT_SOEM/soem/*.c)
file(GLOB OSAL_SOURCES src/EtherCAT_SOEM/osal/${OS}/*.c)
file(GLOB OSHW_SOURCES src/EtherCAT_SOEM/oshw/${OS}/*.c)

file(GLOB SOEM_HEADERS src/EtherCAT_SOEM/soem/*.h)
file(GLOB OSAL_HEADERS src/EtherCAT_SOEM/osal/osal.h osal/${OS}/*.h)
file(GLOB OSHW_HEADERS src/EtherCAT_SOEM/oshw/${OS}/*.h)

add_library(soem STATIC
  ${SOEM_SOURCES}
  ${OSAL_SOURCES}
  ${OSHW_SOURCES}
  ${OSHW_EXTRA_SOURCES})
target_link_libraries(soem ${OS_LIBS})

target_include_directories(soem PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/soem>
  $<INSTALL_INTERFACE:include/soem>)

target_include_directories(soem PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/osal>
  $<INSTALL_INTERFACE:include/soem>)

target_include_directories(soem PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/osal/${OS}>
  $<INSTALL_INTERFACE:include/soem>)

target_include_directories(soem
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/oshw/${OS}>
  $<INSTALL_INTERFACE:include/soem>
  )


# add_subdirectory(src)


# add_compile_options(-std=c++11)
# set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")  #设置c++的编译选项
# set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99")  #设置c的编译选项

# set(EXTRA_LIBS -pthread libunitree_legged_sdk_arm64.so lcm xlsxwriter
#     libUnitreeMotorSDK_Arm64.so)

# set(CMAKE_CXX_FLAGS "-O3")

# set(SOURCES src/app/motor_control.c
#             src/app/motor_control.h
#             src/app/math_ops.c
#             src/app/math_ops.h
#             src/app/transmit.c
#             src/app/transmit.h
#             src/app/config.h
#             src/app/joystick.c
#             src/app/joystick.h
#             src/app/analysis_data.c
#             src/app/analysis_data.h
#             src/app/yesense_decoder.c
#             src/app/yesense_decoder.h
#             src/app/rt_spi.c
#             src/app/rt_spi.h
# )


include_directories(src/EtherCAT_Motor)
include_directories(src/EtherCAT_SOEM/soem)
include_directories(src/EtherCAT_SOEM/osal)
include_directories(src/EtherCAT_SOEM/osal/${OS})
include_directories(src/EtherCAT_SOEM/oshw/${OS})


# Set the C standard
set(CMAKE_C_STANDARD 11)

# Add source files
set(SOURCES
    src/EtherCAT_Motor/EtherCAT_Motor.c
    src/EtherCAT_Motor/EtherCAT_Timer.c
)

# Add include directories
include_directories(
    src/EtherCAT_Motor
)


# Add the executable
add_executable(low_level src/low_level.cpp ${SOURCES})
# Link the SOEM library and necessary system libraries
target_link_libraries(low_level soem pthread rt m ${EXTRA_LIBS} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Add the executable
add_executable(low_level_new src/low_level_new.cpp ${SOURCES})
# Link the SOEM library and necessary system libraries
target_link_libraries(low_level_new soem pthread rt m ${EXTRA_LIBS} ${catkin_LIBRARIES} ${Boost_LIBRARIES})


# Add the executable
add_executable(low_level_no_info src/low_level_no_info.cpp ${SOURCES})
# Link the SOEM library and necessary system libraries
target_link_libraries(low_level_no_info soem pthread rt m ${EXTRA_LIBS} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Add the executable
add_executable(get_joint_state src/get_joint_state.cpp ${SOURCES})
# Link the SOEM library and necessary system libraries
target_link_libraries(get_joint_state soem pthread rt m ${EXTRA_LIBS} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Add the executable
add_executable(low_level_ik src/low_level_ik.cpp ${SOURCES})
# Link the SOEM library and necessary system libraries
target_link_libraries(low_level_ik soem pthread rt m ${EXTRA_LIBS} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

find_package(pinocchio REQUIRED)
set(CMAKE_CXX_STANDARD 17)  # 设置 C++17

# add_executable(left_control_ik src/left_control_ik.cpp)
# target_link_libraries(left_control_ik PUBLIC pinocchio::pinocchio ${EXTRA_LIBS} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# add_executable(right_control_ik src/right_control_ik.cpp)
# target_link_libraries(right_control_ik PUBLIC pinocchio::pinocchio ${EXTRA_LIBS} ${catkin_LIBRARIES} ${Boost_LIBRARIES})


add_executable(combine_control_ik src/combine_control_ik.cpp)
target_link_libraries(combine_control_ik PUBLIC pinocchio::pinocchio ${EXTRA_LIBS} ${catkin_LIBRARIES} ${Boost_LIBRARIES})