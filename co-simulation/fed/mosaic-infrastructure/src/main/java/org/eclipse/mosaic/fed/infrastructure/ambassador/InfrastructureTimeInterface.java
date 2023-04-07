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

package org.eclipse.mosaic.fed.infrastructure.ambassador;

import java.io.IOException;

import org.slf4j.Logger;

import com.google.gson.Gson;

public class InfrastructureTimeInterface {
    private InfrastructureInstanceManager manager;
    // set to false on init release
    private boolean await_infrastructure_advance_request = false;

    protected Logger log;

    public InfrastructureTimeInterface(InfrastructureInstanceManager manager) {
        this.manager = manager;
    }

    /**
     * This function implements the encoding of a json string consist of timestep
     * and seq
     * 
     * @param message This is the class that gets encoded, which include the data of
     *                timestep and seq
     * @return return_json: encoded json string
     */
    public byte[] encodeTimeMessage(InfrastructureTimeMessage message) {
        Gson gson = new Gson();
        String json = gson.toJson(message);
        byte[] returnJson = json.getBytes();
        return returnJson;
    }

    /**
     * This function is used to send out encoded timestep update to all registered
     * instances the manager has on the managed instances map
     * 
     * @param message This time message is used to store current seq and timestep
     *                from the ambassador side
     * @throws IOException
     */
    public void onTimeStepUpdate(InfrastructureTimeMessage message) throws IOException {
        if (this.manager.getManagedInstances().size() == 0) {
            log.info("There are no registered instances");
        }

        for (InfrastructureInstance currentInstance : manager.getManagedInstances().values()) {
            currentInstance.sendTimeSyncMsgs(encodeTimeMessage(message));
        }
    }
}