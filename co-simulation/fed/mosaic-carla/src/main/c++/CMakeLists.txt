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

add_library(carla-jni SHARED
    libcarla-jni.cpp)
set_target_properties(carla-jni PROPERTIES SUFFIX ".so")

include_directories(
    ${JNI_INCLUDE_DIRS}
    include
)

#ADD_LIBRARY(libcarla_client STATIC IMPORTED)
#SET_TARGET_PROPERTIES(libcarla_client PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/libcarla_client.a)

#TARGET_LINK_LIBRARIES(mytarget A boost_unit_test_framework C)
cmake_print_variables(CMAKE_SOURCE_DIR)

target_link_libraries(carla-jni
    ${CMAKE_SOURCE_DIR}/libcarla_client.a
    ${JNI_LIBRARIES}
)

install(TARGETS carla-jni
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
