######################################################################
# Find script for SBA
#
# Assumes that environment variable SBAROOT is set to the root of the oal install 
#
# Input Environment Variables:
# SBAROOT                  : Root of SBA Install
#
# Output Variables:
# -----------------
# SBA_FOUND                : TRUE if search succeded
# SBA_INCLUDE_DIRS         : include paths for oal
# SBA_LIB                  : library if found
# 
######################################################################

set(SBA_ROOT_DIR $ENV{SBAROOT})
set(SBA_SRC_DIR "${SBA_ROOT_DIR}")
set(SBA_DIR "${SBA_SRC_DIR}")

set(SBA_INCLUDE_DIRS ${SBA_DIR} ${SBA_SRC_DIR} ${SBA_ROOT_DIR})

set(SBA_LIB SBA-NOTFOUND) # if we don't do this find_library caches the results
find_library(SBA_LIB sba PATHS ${SBA_ROOT_DIR} )

#if not found search in the sba root directory for libsba.a 
if(NOT SBA_LIB)
	find_library(SBA_LIB sba PATHS ${SBA_ROOT_DIR})
endif(NOT SBA_LIB)

# if still not found print error message
if(SBA_LIB)
  set(SBA_FOUND true)
else()
  message("WARNING: Could not find library ${SBA_LIB}\nDid you set SBAROOT environment variable?")
  set(SBA_FOUND false)
endif()

