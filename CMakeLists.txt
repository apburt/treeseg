cmake_minimum_required(VERSION 3.10)
project(treeseg)

find_package(PCL 1.9 REQUIRED)
find_package(Armadillo 9.6 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${ARMADILLO_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/include/riegl)
link_directories(${CMAKE_SOURCE_DIR}/lib ${CMAKE_SOURCE_DIR}/lib/riegl)

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	if(EXISTS "${CMAKE_SOURCE_DIR}/lib/riegl/libscanifc-mt.so")
		add_executable(rxp2pcd ${CMAKE_SOURCE_DIR}/src/rxp2pcd.cpp)
		target_link_libraries(rxp2pcd scanifc-mt)

		add_executable(rxp2tree ${CMAKE_SOURCE_DIR}/src/rxp2tree.cpp)
		target_link_libraries(rxp2tree ${PCL_LIBRARIES} scanifc-mt)
	endif()
	else()
		message(WARNING "rxp executables not building because either RIEGL libs/headers not found, or not a compatible OS.")
endif()

add_library(treeseg SHARED ${CMAKE_SOURCE_DIR}/src/treeseg.cpp)
target_link_libraries(treeseg ${PCL_LIBRARIES})

add_library(leafsep SHARED ${CMAKE_SOURCE_DIR}/src/leafsep.cpp)
target_link_libraries(leafsep treeseg ${PCL_LIBRARIES} ${ARMADILLO_LIBRARIES})

add_executable(plotcoords ${CMAKE_SOURCE_DIR}/src/plotcoords.cpp)
target_link_libraries(plotcoords)

add_executable(nearestneighbour ${CMAKE_SOURCE_DIR}/src/nearestneighbour.cpp)
target_link_libraries(nearestneighbour treeseg ${PCL_LIBRARIES})

add_executable(downsample ${CMAKE_SOURCE_DIR}/src/downsample.cpp)
target_link_libraries(downsample treeseg ${PCL_LIBRARIES})

add_executable(thin ${CMAKE_SOURCE_DIR}/src/thin.cpp)
target_link_libraries(thin treeseg ${PCL_LIBRARIES})

add_executable(getdemslice ${CMAKE_SOURCE_DIR}/src/getdemslice.cpp)
target_link_libraries(getdemslice treeseg ${PCL_LIBRARIES})

add_executable(findstems ${CMAKE_SOURCE_DIR}/src/findstems.cpp)
target_link_libraries(findstems treeseg ${PCL_LIBRARIES})

add_executable(segmentstem ${CMAKE_SOURCE_DIR}/src/segmentstem.cpp)
target_link_libraries(segmentstem treeseg ${PCL_LIBRARIES})

add_executable(getcrownvolume ${CMAKE_SOURCE_DIR}/src/getcrownvolume.cpp)
target_link_libraries(getcrownvolume treeseg ${PCL_LIBRARIES})

add_executable(segmentcrown ${CMAKE_SOURCE_DIR}/src/segmentcrown.cpp)
target_link_libraries(segmentcrown treeseg leafsep ${PCL_LIBRARIES})

add_executable(xyz2pcd ${CMAKE_SOURCE_DIR}/src/xyz2pcd.cpp)
target_link_libraries(xyz2pcd treeseg ${PCL_LIBRARIES})

add_executable(pcd2xyz ${CMAKE_SOURCE_DIR}/src/pcd2xyz.cpp)
target_link_libraries(pcd2xyz treeseg ${PCL_LIBRARIES})

add_executable(sepwoodleaf ${CMAKE_SOURCE_DIR}/src/sepwoodleaf.cpp)
target_link_libraries(sepwoodleaf treeseg leafsep ${PCL_LIBRARIES})
