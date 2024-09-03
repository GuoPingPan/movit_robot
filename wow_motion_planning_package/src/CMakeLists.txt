cmake_minimum_required(VERSION 3.5)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules")
project(SOEM)

if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  # Default to installing in SOEM source directory
  set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install)
endif()

set(CMAKE_CXX_STANDARD 14)


set(SOEM_INCLUDE_INSTALL_DIR include/soem)
set(SOEM_LIB_INSTALL_DIR lib)
set(BUILD_TESTS TRUE)



file(GLOB SOEM_SOURCES soem/*.c)
file(GLOB OSAL_SOURCES osal/${OS}/*.c)
file(GLOB OSHW_SOURCES oshw/${OS}/*.c)

file(GLOB SOEM_HEADERS soem/*.h)
file(GLOB OSAL_HEADERS osal/osal.h osal/${OS}/*.h)
file(GLOB OSHW_HEADERS oshw/${OS}/*.h)

include_directories(soem)
include_directories(osal)
include_directories(osal/${OS})
include_directories(oshw/${OS})

include_directories(include)
link_directories(lib)

# add_library(soem STATIC
add_library(soem SHARED
  ${SOEM_SOURCES}
  ${OSAL_SOURCES}
  ${OSHW_SOURCES}
  ${OSHW_EXTRA_SOURCES})
target_link_libraries(soem ${OS_LIBS})

# 添加LibSerial库的头文件路径
include_directories(/usr/include/libserial)
# 添加LibSerial库的库文件路径
link_directories(/usr/lib/aarch64-linux-gnu)

message("LIB_DIR: ${SOEM_LIB_INSTALL_DIR}")

# install(TARGETS soem DESTINATION ${SOEM_LIB_INSTALL_DIR})
# install(FILES
#   ${SOEM_HEADERS}
#   ${OSAL_HEADERS}
#   ${OSHW_HEADERS}
#   DESTINATION ${SOEM_INCLUDE_INSTALL_DIR})

# if(BUILD_TESTS) 
add_subdirectory(app)
# endif()
