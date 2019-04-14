########################################################################
# Build stuff
########################################################################

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

macro(use_cxx11)
  if (CMAKE_VERSION VERSION_LESS "3.1")
    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
    endif ()
  else ()
    set (CMAKE_CXX_STANDARD 11)
  endif ()
endmacro(use_cxx11)

#enable c++11 features
#if(CMAKE_COMPILER_IS_GNUCXX)
#
#    #C++11 is a required language feature for this project
#    include(CheckCXXCompilerFlag)
#    CHECK_CXX_COMPILER_FLAG("-std=c++11" HAS_STD_CXX11)
#    if(HAS_STD_CXX11)
#        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#    else(HAS_STD_CXX11)
#        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#    endif()

#    #disable warnings for unused parameters
#    add_definitions(-Wno-unused-parameter)

#endif(CMAKE_COMPILER_IS_GNUCXX)

add_library(ExtIO_linrad-soapy MODULE ExtIO_linrad-soapy.h ExtIO_linrad-soapy.cpp)
target_link_libraries(ExtIO_linrad-soapy ${OTHER_LIBRARIES})
set_target_properties(ExtIO_linrad-soapy PROPERTIES DEBUG_POSTFIX "") #same name in debug mode

find_package(Threads REQUIRED)
if(THREADS_HAVE_PTHREAD_ARG)
  target_compile_options(ExtIO_linrad-soapy PUBLIC "-pthread")
endif()
if(CMAKE_THREAD_LIBS_INIT)
  target_link_libraries(ExtIO_linrad-soapy "${CMAKE_THREAD_LIBS_INIT}")
endif()
