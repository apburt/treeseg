cmake_minimum_required(VERSION 2.8)
project(treeseg)

set(CMAKE_CXX_FLAGS "-std=c++0x")

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ../include)
link_directories(../lib)

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	if(EXISTS "../lib/libscanifc-mt.so")
		add_executable(plotcoords plotcoords.cpp)
		target_link_libraries(plotcoords)
		add_executable(rxp2pcd rxp2pcd.cpp)
		target_link_libraries(rxp2pcd pthread scanlib-mt riboost_system-mt)
	endif()
else()
	message(WARNING "rxp2pcd not building because either RIEGL libs/headers not found, or not a compatible OS.")
endif()

add_library(treeseg SHARED treeseg.cpp)
target_link_libraries(treeseg ${PCL_LIBRARIES})

add_executable(nearestneighbour nearestneighbour.cpp)
target_link_libraries(nearestneighbour treeseg ${PCL_LIBRARIES})

add_executable(downsample downsample.cpp)
target_link_libraries(downsample treeseg ${PCL_LIBRARIES})

add_executable(getdemslice getdemslice.cpp)
target_link_libraries(getdemslice treeseg ${PCL_LIBRARIES})

add_executable(findstems findstems.cpp)
target_link_libraries(findstems treeseg ${PCL_LIBRARIES})

add_executable(segmentstem segmentstem.cpp)
target_link_libraries(segmentstem treeseg ${PCL_LIBRARIES})

add_executable(getcrownvolume getcrownvolume.cpp)
target_link_libraries(getcrownvolume treeseg ${PCL_LIBRARIES})

add_executable(segmentcrown segmentcrown.cpp)
target_link_libraries(segmentcrown treeseg ${PCL_LIBRARIES})

add_executable(xyz2pcd xyz2pcd.cpp)
target_link_libraries(xyz2pcd ${PCL_LIBRARIES})

add_executable(pcd2xyz pcd2xyz.cpp)
target_link_libraries(pcd2xyz ${PCL_LIBRARIES})
