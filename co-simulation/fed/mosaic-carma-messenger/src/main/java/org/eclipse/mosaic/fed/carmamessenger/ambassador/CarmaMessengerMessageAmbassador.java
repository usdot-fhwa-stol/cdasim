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
package org.eclipse.mosaic.fed.carmamessenger.ambassador;

import org.eclipse.mosaic.fed.carma.configuration.CarmaConfiguration;
import org.eclipse.mosaic.lib.util.CommonMessageAmbassador;
import org.eclipse.mosaic.lib.util.objects.ObjectInstantiation;
import org.eclipse.mosaic.rti.api.parameters.AmbassadorParameter;

import gov.dot.fhwa.saxton.CarmaV2xMessageReceiver;

public class CarmaMessengerMessageAmbassador extends CommonMessageAmbassador<CarmaMessengerInstanceManager, CarmaMessengerRegistrationReceiver, CarmaMessengerRegistrationMessage>{
    /**
     * Simulation time.
     */
    long currentSimulationTime;

    /**
     * CarmaMessageAmbassador configuration file.
     */
    CarmaConfiguration carmaMessengerConfiguration;

    private CarmaMessengerRegistrationReceiver carmaMessengerRegistrationReceiver;
    private Thread registrationRxBackgroundThread;
    private CarmaV2xMessageReceiver v2xMessageReceiver;
    private Thread v2xRxBackgroundThread;
    private CarmaMessengerInstanceManager carmaMessengerInstanceManager = new CarmaMessengerInstanceManager();
    private int timeSyncSeq = 0;


    /**
     * Create a new {@link CarmaMessengerMessageAmbassador} object.
     *
     * @param ambassadorParameter includes parameters for the
     *                            CarmaMessageAmbassador.
     */
    public CarmaMessengerMessageAmbassador(AmbassadorParameter ambassadorParameter) {
        super(ambassadorParameter, CarmaMessengerRegistrationMessage.class);

        try {
            // Read the CARMA message ambassador configuration file
            carmaMessengerConfiguration = new ObjectInstantiation<>(CarmaConfiguration.class, log)
                    .readFile(ambassadorParameter.configuration);
        } catch (InstantiationException e) {
            log.error("Configuration object could not be instantiated: ", e);
        }

        log.info("The update interval of CARMA message ambassador is " + carmaMessengerConfiguration.updateInterval + " .");

        // Check the CARMA update interval
        if (carmaMessengerConfiguration.updateInterval <= 0) {
            throw new RuntimeException("Invalid update interval for CARMA message ambassador, should be >0.");
        }
        log.info("CARMA message ambassador is generated.");
    }

}
