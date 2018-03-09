# Created by the script cgal_create_cmake_script
# This is the CMake script for compiling a CGAL application.
project( rykers-sf_ )
cmake_minimum_required(VERSION 2.8.10)
set(CMAKE_CXX_STANDARD 14)
find_package(CGAL QUIET COMPONENTS Core )
if ( CGAL_FOUND )
  include( ${CGAL_USE_FILE} )

  set(SOURCES src/partition.cc src/main.cc)

  add_executable(reconstruct_3d ${SOURCES})
  target_include_directories(reconstruct_3d PRIVATE include)
  target_link_libraries(reconstruct_3d ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})
else()
    message(STATUS "This program requires the CGAL library, and will not be compiled.")
endif()

