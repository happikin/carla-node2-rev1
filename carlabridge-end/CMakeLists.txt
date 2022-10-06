set(CMAKE_CXX_FLAGS "-w")
project(opendds_devguide_messenger CXX)
cmake_minimum_required(VERSION 3.3)

find_package(Boost REQUIRED System)
find_package(OpenDDS REQUIRED)
find_package(OpenCV REQUIRED)
# Make sure the MPC-generated headers are gone so the CMake build will use the
# right ones. This is not needed in a real project.

# IDL TypeSupport Library
add_library(carladata_idl)
OPENDDS_TARGET_SOURCES(carladata_idl PUBLIC "CarlaData.idl")
target_link_libraries(carladata_idl PUBLIC OpenDDS::Dcps)

set(CARLA_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libcarla-install/lib)
set(CARLA_LIBS ${CARLA_LIB_DIR}/libcarla_client.a ${CARLA_LIB_DIR}/librpc.a ${CARLA_LIB_DIR}/libRecast.a ${CARLA_LIB_DIR}/libDetour.a ${CARLA_LIB_DIR}/libDetourCrowd.a ${CARLA_LIB_DIR}/libboost_filesystem.a ${CARLA_LIB_DIR}/libboost_system.a)
set(CARLA_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libcarla-install/include ${CMAKE_CURRENT_SOURCE_DIR}/libcarla-install/include/system)

include_directories(include ${CARLA_INC_DIR})

set(opendds_libs
  OpenDDS::Dcps # Core OpenDDS Library
  OpenDDS::InfoRepoDiscovery OpenDDS::Tcp # For run_test.pl
  OpenDDS::Rtps OpenDDS::Rtps_Udp # For run_test.pl --rtps
  carladata_idl
)

add_executable(node1
  ../src/node1.cpp
  ../src/VehicleOdometryListener.cpp
)
target_link_libraries(node1 ${opendds_libs} ${OpenCV_LIBS})

add_executable(node2
  ../src/node2.cpp
  ../src/RadarSensorListener.cpp
  ../src/ImageSensorListener.cpp
)
target_link_libraries(node2 ${opendds_libs} ${OpenCV_LIBS})

add_executable(carlabridge
  ../src/carlabridge.cpp
  ../src/VehicleOdometryListener.cpp
)
target_link_libraries(carlabridge ${opendds_libs} ${CARLA_LIBS} ${BOOST_LIBRARIES} ${OpenCV_LIBS} pthread)