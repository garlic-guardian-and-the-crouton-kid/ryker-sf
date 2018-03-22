project( rykers-sf_ )
cmake_minimum_required(VERSION 2.8.10)
set(CMAKE_CXX_STANDARD 14)
find_package(CGAL QUIET COMPONENTS Core )
find_package( OpenCV QUIET )
find_package( GDAL QUIET )
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package( sba )

if ( NOT CGAL_FOUND )
  message(STATUS "This program requires the CGAL library, and will not be compiled.")
elseif ( NOT OpenCV_FOUND )
  message(STATUS "This program requires the OpenCV library, and will not be compiled")
elseif ( NOT GDAL_FOUND )
  message(STATUS "This program requires the GDAL library, and will not be compiled")
elseif ( NOT SBA_FOUND )
  message(STATUS "This program requires the SBA library, and will not be compiled")
else()
  include( ${CGAL_USE_FILE} )
  include_directories("${GDAL_INCLUDE_DIR}")
  include_directories( ${OpenCV_INCLUDE_DIRS} )
  include_directories( ${SBA_INCLUDE_DIRS} )

  set(SOURCES
		src/image_metadata.cc
		src/overlapping_image_set.cc
		src/partition.cc
		src/point_match.cc
		src/point_set.cc
		src/triangulate.cc
		src/main.cc)
  
  add_executable(reconstruct_3d ${SOURCES})
  target_include_directories(reconstruct_3d PRIVATE include)
  target_link_libraries(reconstruct_3d ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} ${OpenCV_LIBS} ${GDAL_LIBRARY} ${SBA_LIBS})
endif()

