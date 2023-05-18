#include "include/org_eclipse_mosaic_fed_carla_libcarla_CarlaClient.h"
#include <boost/asio.hpp>
#include "carla/client/Client.h"
#include "carla/Time.h"


std::unique_ptr<carla::client::Client> client_instance = nullptr;
bool connected = false;
long current_sim_timestep_ns = 0;
bool running = false;
JNIEXPORT void JNICALL Java_org_eclipse_mosaic_fed_carla_libcarla_CarlaClient_setSynchronous_1
  (JNIEnv *, jobject){}

/*
 * Class:     org_eclipse_mosaic_fed_carla_libcarla_CarlaClient
 * Method:    connect_
 * Signature: (Ljava/lang/String{}I)Z
 */
JNIEXPORT jboolean JNICALL Java_org_eclipse_mosaic_fed_carla_libcarla_CarlaClient_connect_1
  (JNIEnv *, jobject, jstring, jint){
    carla::client::Client target_client{"127.0.0.1", 1516, 0};
    target_client.GetWorld().Tick(carla::time_duration::milliseconds(50));
  }

/*
 * Class:     org_eclipse_mosaic_fed_carla_libcarla_CarlaClient
 * Method:    loadWorld_
 * Signature: (Ljava/lang/String{}Z)Z
 */
JNIEXPORT jboolean JNICALL Java_org_eclipse_mosaic_fed_carla_libcarla_CarlaClient_loadWorld_1
  (JNIEnv *, jobject, jstring, jboolean){}

/*
 * Class:     org_eclipse_mosaic_fed_carla_libcarla_CarlaClient
 * Method:    reloadWorld_
 * Signature: (Z)Z
 */
JNIEXPORT jboolean JNICALL Java_org_eclipse_mosaic_fed_carla_libcarla_CarlaClient_reloadWorld_1
  (JNIEnv *, jobject, jboolean){}

/*
 * Class:     org_eclipse_mosaic_fed_carla_libcarla_CarlaClient
 * Method:    getActor_
 * Signature: (I)[B
 */
JNIEXPORT jbyteArray JNICALL Java_org_eclipse_mosaic_fed_carla_libcarla_CarlaClient_getActor_1
  (JNIEnv *, jobject, jint){}

/*
 * Class:     org_eclipse_mosaic_fed_carla_libcarla_CarlaClient
 * Method:    getActorIds_
 * Signature: ()[I
 */
JNIEXPORT jintArray JNICALL Java_org_eclipse_mosaic_fed_carla_libcarla_CarlaClient_getActorIds_1
  (JNIEnv *, jobject){}

/*
 * Class:     org_eclipse_mosaic_fed_carla_libcarla_CarlaClient
 * Method:    getManagedActors_
 * Signature: ()[I
 */
JNIEXPORT jintArray JNICALL Java_org_eclipse_mosaic_fed_carla_libcarla_CarlaClient_getManagedActors_1
  (JNIEnv *, jobject){}

/*
 * Class:     org_eclipse_mosaic_fed_carla_libcarla_CarlaClient
 * Method:    createActor_
 * Signature: ()Z
 */
JNIEXPORT jboolean JNICALL Java_org_eclipse_mosaic_fed_carla_libcarla_CarlaClient_createActor_1
  (JNIEnv *, jobject){}

/*
 * Class:     org_eclipse_mosaic_fed_carla_libcarla_CarlaClient
 * Method:    getCurrentTimestep_
 * Signature: ()J
 */
JNIEXPORT jlong JNICALL Java_org_eclipse_mosaic_fed_carla_libcarla_CarlaClient_getCurrentTimestep_1
  (JNIEnv *, jobject){}

/*
 * Class:     org_eclipse_mosaic_fed_carla_libcarla_CarlaClient
 * Method:    applySettings_
 * Signature: ()Z
 */
JNIEXPORT jboolean JNICALL Java_org_eclipse_mosaic_fed_carla_libcarla_CarlaClient_applySettings_1
  (JNIEnv *, jobject){}

/*
 * Class:     org_eclipse_mosaic_fed_carla_libcarla_CarlaClient
 * Method:    loadTransform_
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_org_eclipse_mosaic_fed_carla_libcarla_CarlaClient_loadTransform_1
  (JNIEnv *, jobject){}

/*
 * Class:     org_eclipse_mosaic_fed_carla_libcarla_CarlaClient
 * Method:    getClientVersion_
 * Signature: ()Ljava/lang/String{}
 */
JNIEXPORT jstring JNICALL Java_org_eclipse_mosaic_fed_carla_libcarla_CarlaClient_getClientVersion_1
  (JNIEnv *env, jobject self){
    char msg[5] =  "test";
    jstring output;
    output = env->NewStringUTF(msg);
    return output;
  }

/*
 * Class:     org_eclipse_mosaic_fed_carla_libcarla_CarlaClient
 * Method:    getServerVersion_
 * Signature: ()Ljava/lang/String{}
 */
JNIEXPORT jstring JNICALL Java_org_eclipse_mosaic_fed_carla_libcarla_CarlaClient_getServerVersion_1
  (JNIEnv *, jobject){}