########################################################################
# Build stuff
########################################################################

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_library(ExtIO_linrad-soapy MODULE ExtIO_linrad-soapy.h ExtIO_linrad-soapy.cpp
    ../SoapyRemote/server/ServerListener.cpp
    ../SoapyRemote/server/ClientHandler.cpp
    ../SoapyRemote/server/LogForwarding.cpp
    ../SoapyRemote/server/ServerStreamData.cpp)

target_link_libraries(ExtIO_linrad-soapy PRIVATE SoapySDR SoapySDRRemoteCommon)
message(STATUS "SOAPY_SERVER_SOURCES: ${SOAPY_SERVER_SOURCES}")
target_include_directories(ExtIO_linrad-soapy PRIVATE ../SoapyRemote/server)
target_include_directories(ExtIO_linrad-soapy PRIVATE ../SoapyRemote/common)

# target_link_libraries(ExtIO_linrad-soapy ${OTHER_LIBRARIES})
# set_target_properties(ExtIO_linrad-soapy PROPERTIES DEBUG_POSTFIX "") #same name in debug mode

find_package(Threads REQUIRED)
message(STATUS "CMAKE_THREAD_LIBS_INIT: ${CMAKE_THREAD_LIBS_INIT}")
if(THREADS_HAVE_PTHREAD_ARG)
  target_compile_options(ExtIO_linrad-soapy PUBLIC "-pthread")
endif()
if(CMAKE_THREAD_LIBS_INIT)
    target_link_libraries(ExtIO_linrad-soapy PUBLIC "${CMAKE_THREAD_LIBS_INIT}")
endif()

########################################################################
# Thread config support
########################################################################
find_library(
    RT_LIBRARIES
    NAMES rt
    PATHS /usr/lib /usr/lib64
)

if (RT_LIBRARIES)
    target_link_libraries(ExtIO_linrad-soapy PRIVATE ${RT_LIBRARIES})
endif()

if(WIN32)
    target_sources(ExtIO_linrad-soapy PRIVATE ../SoapyRemote/server/ThreadPrioWindows.cpp)
elseif(UNIX)
    target_sources(ExtIO_linrad-soapy PRIVATE ../SoapyRemote/server/ThreadPrioUnix.cpp)
endif()
