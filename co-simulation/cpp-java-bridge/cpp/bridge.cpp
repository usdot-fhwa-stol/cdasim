#include <iostream>
#include "pure_cpp_.h"
#include <jni.h>
#include <../example_com_MyBridgeClass.h>
#include "carla_files.h"

JNIEXPORT jint JNICALL Java_example_com_MyBridgeClass_subtract(JNIEnv *env, jobject obj, jint a, jint b) {
    jint result = pure_cpp::subtract_cpp(a,b);
    std::cout << "C++ Subtract: " << result << std::endl;
    return result;
}

JNIEXPORT jint JNICALL Java_example_com_MyBridgeClass_sum(JNIEnv *env, jobject obj, jint a, jint b) {
    jint sum_ = pure_cpp::sum_cpp(a, b);
    std::cout << "C++ Sum: " << sum_ << std::endl;
    return sum_;
}

JNIEXPORT jint JNICALL Java_example_com_MyBridgeClass_dryTest(JNIEnv *env, jobject obj) {
    int is_success = dryTest();
    std::cout << "C++ CARLAs Integration Success: " << is_success << std::endl;
    return is_success;
}
