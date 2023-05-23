/*
 * Copyright (C) 2023 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.eclipse.mosaic.fed.carla.libcarla;

public class CarlaClient {

    static {
        System.loadLibrary("carla-jni");
    }

    public String getClientVersion() {
        libpython_clj2.java_api.importModule("carla_mosaic_bridge.py");
        libpython_clj2.java_api.call()
        return getClientVersion_();
    }

    private native void init_();

    private native void tick_();

    private native void setSynchronous_();

    private native boolean connect_(String serverIp, int serverPort);

    private native boolean loadWorld_(String mapName, boolean resetSettings);

    private native boolean reloadWorld_(boolean resetSettings);

    private native byte[] getActor_(int actorId);

    private native int[] getActorIds_();

    private native int[] getManagedActors_();

    private native boolean createActor_();

    private native long getCurrentTimestep_();

    private native boolean applySettings_();

    private native void loadTransform_();

    private native String getClientVersion_();

    private native String getServerVersion_();
}
