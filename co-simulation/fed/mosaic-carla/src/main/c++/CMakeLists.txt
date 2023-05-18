project(libcarla-jni)

find_package(Java REQUIRED)
find_package(JNI REQUIRED)
if (JNI_FOUND)
    message (STATUS "JNI_INCLUDE_DIRS=${JNI_INCLUDE_DIRS}")
    message (STATUS "JNI_LIBRARIES=${JNI_LIBRARIES}")
endif()
include(UseJava)

add_library(libcarla-jni 
    libcarla-jni.cpp)

target_include_directories(libcarla-jni
    ${JNI_INCLUDE_DIRS}
    include
)

target_link_libraries(libcarla-jni
    libcarla-clinet.a
    ${JNI_LIBRARIES}
)

set_target_properties(libcarla-jni PROPERTIES SUFFIX ".so")