#Andrew Burt - a.burt@ucl.ac.uk

cmake_minimum_required(VERSION 2.8)
project(treeseg)

SET(CMAKE_CXX_FLAGS "-std=c++0x")
find_package(PCL 1.7 REQUIRED)

#include_directories(${PCL_INCLUDE_DIRS} /data/TLS/tools/treeseg/include)
#link_directories(/data/TLS/tools/treeseg/lib)
include_directories(${PCL_INCLUDE_DIRS} /Users/ucfaab0/OneDrive/UCL/tools/treeseg/include)
link_directories(/Users/ucfaab0/OneDrive/UCL/tools/treeseg/lib)

IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

	add_executable(rxp2pcd rxp2pcd.cpp)
	target_link_libraries(rxp2pcd pthread scanlib-mt riboost_system-mt)

	add_executable(rxp2tree rxp2tree.cpp)
	target_link_libraries(rxp2tree pthread scanlib-mt riboost_system-mt ${PCL_LIBRARIES})

ENDIF()

add_executable(cloudresults cloudresults.cpp)
target_link_libraries(cloudresults ${PCL_LIBRARIES})

add_executable(downsample downsample.cpp)
target_link_libraries(downsample ${PCL_LIBRARIES})

add_executable(getclusters getclusters.cpp)
target_link_libraries(getclusters ${PCL_LIBRARIES})

add_executable(getvolume getvolume.cpp)
target_link_libraries(getvolume ${PCL_LIBRARIES})

add_executable(getslice getslice.cpp)
target_link_libraries(getslice ${PCL_LIBRARIES})

add_executable(isolatetree isolatetree.cpp)
target_link_libraries(isolatetree ${PCL_LIBRARIES})

add_executable(nearestneighbour nearestneighbour.cpp)
target_link_libraries(nearestneighbour ${PCL_LIBRARIES})

add_executable(pcdASCII2binary pcdASCII2binary.cpp)
target_link_libraries(pcdASCII2binary ${PCL_LIBRARIES})

add_executable(pcdbinary2ASCII pcdbinary2ASCII.cpp)
target_link_libraries(pcdbinary2ASCII ${PCL_LIBRARIES})

add_executable(plotcoords plotcoords.cpp)
target_link_libraries(plotcoords)

add_executable(pruneleaves pruneleaves.cpp)
target_link_libraries(pruneleaves ${PCL_LIBRARIES})

add_executable(segmentstems segmentstems.cpp)
target_link_libraries(segmentstems ${PCL_LIBRARIES})
