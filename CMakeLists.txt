project( rykers-sf_ )
cmake_minimum_required(VERSION 2.8.10)
set(CMAKE_CXX_STANDARD 14)
find_package(CGAL QUIET COMPONENTS Core )
find_package( OpenCV QUIET )

if ( NOT CGAL_FOUND )
  message(STATUS "This program requires the CGAL library, and will not be compiled.")
elseif ( NOT OPENCV_FOUND )
  message(STATUS "This program requires the OpenCV library, and will not be compiled")
else()
  include( ${CGAL_USE_FILE} )
  include_directories( ${OpenCV_INCLUDE_DIRS} )

  set(SOURCES src/partition.cc src/main.cc src/point_match.cc)

  add_executable(reconstruct_3d ${SOURCES})
  target_include_directories(reconstruct_3d PRIVATE include)
  target_link_libraries(reconstruct_3d ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} ${OpenCV_LIBS})

  add_executable(point_matching_tests src/point_match.cc)
  target_include_directories(point_matching_tests PRIVATE include)
  target_link_libraries(point_matching_tests ${OpenCV_LIBS})
  
endif()

