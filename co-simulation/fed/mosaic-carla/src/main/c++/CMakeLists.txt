cmake_minimum_required(VERSION 3.16)
include(CMakePrintHelpers)
include(GNUInstallDirs)

project(carla-jni)

find_package(Java REQUIRED)
find_package(JNI REQUIRED)
if (JNI_FOUND)
    message (STATUS "JNI_INCLUDE_DIRS=${JNI_INCLUDE_DIRS}")
    message (STATUS "JNI_LIBRARIES=${JNI_LIBRARIES}")
endif()
include(UseJava)

FIND_PACKAGE( Boost 1.71 COMPONENTS thread filesystem REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

add_library(carla-jni SHARED
    libcarla-jni.cpp)
set_target_properties(carla-jni PROPERTIES SUFFIX ".so")

add_executable(libcarla-test
	libcarla-test.cpp)

include_directories(
    ${JNI_INCLUDE_DIRS}
    include
    carla)

target_link_libraries(carla-jni
    ${CMAKE_SOURCE_DIR}/lib/libcarla_client.a
    ${CMAKE_SOURCE_DIR}/lib/libboost_atomic.a
    ${CMAKE_SOURCE_DIR}/lib/libboost_filesystem.a
    ${CMAKE_SOURCE_DIR}/lib/libboost_numpy38.a
    ${CMAKE_SOURCE_DIR}/lib/libboost_program_options.a
    ${CMAKE_SOURCE_DIR}/lib/libboost_system.a
    ${CMAKE_SOURCE_DIR}/lib/libDetour.a
    ${CMAKE_SOURCE_DIR}/lib/libDetourCrowd.a
    ${CMAKE_SOURCE_DIR}/lib/libDetourTileCache.a
    ${CMAKE_SOURCE_DIR}/lib/libRecast.a
    ${CMAKE_SOURCE_DIR}/lib/librpc.a
    ${JNI_LIBRARIES})

target_link_libraries(libcarla-test
    ${CMAKE_SOURCE_DIR}/lib/libcarla_client.a
    ${CMAKE_SOURCE_DIR}/lib/libboost_atomic.a
    ${CMAKE_SOURCE_DIR}/lib/libboost_filesystem.a
    ${CMAKE_SOURCE_DIR}/lib/libboost_numpy38.a
    ${CMAKE_SOURCE_DIR}/lib/libboost_program_options.a
    ${CMAKE_SOURCE_DIR}/lib/libboost_system.a
    ${CMAKE_SOURCE_DIR}/lib/libDetour.a
    ${CMAKE_SOURCE_DIR}/lib/libDetourCrowd.a
    ${CMAKE_SOURCE_DIR}/lib/libDetourTileCache.a
    ${CMAKE_SOURCE_DIR}/lib/libRecast.a
    ${CMAKE_SOURCE_DIR}/lib/librpc.a
    ${JNI_LIBRARIES})

install(TARGETS carla-jni
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

