########################################################################
# Build stuff
########################################################################
cmake_minimum_required(VERSION 2.8.12)
project(linrad_extio_SoapySDR CXX)

# list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
find_package(SoapySDR "0.8.0" REQUIRED)
message(STATUS "SOAPY_SDR_INCLUDE_DIRS - ${SOAPY_SDR_INCLUDE_DIR}")
message(STATUS "SOAPY_SDR_LIBRARIES - ${SOAPY_SDR_LIBRARY}")
include_directories(${SOAPY_SDR_INCLUDE_DIR})
SET(OTHER_LIBRARIES ${SOAPY_SDR_LIBRARY} ${OTHER_LIBRARIES})
ADD_DEFINITIONS(
    -DUSE_SOAPY_SDR=1
)

macro(use_cxx11)
  if (CMAKE_VERSION VERSION_LESS "3.1")
    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
    endif ()
  else ()
    set (CMAKE_CXX_STANDARD 11)
  endif ()
endmacro(use_cxx11)

use_cxx11()

add_subdirectory(src)

