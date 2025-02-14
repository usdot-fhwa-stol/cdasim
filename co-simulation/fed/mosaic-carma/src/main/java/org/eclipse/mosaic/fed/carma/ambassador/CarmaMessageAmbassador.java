/*
 * Copyright (c) 2021 Old Dominion University. All rights reserved.
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 */

package org.eclipse.mosaic.fed.carma.ambassador;

import org.eclipse.mosaic.fed.carma.configuration.CarmaConfiguration;
import org.eclipse.mosaic.lib.CommonUtil.ambassador.CommonMessageAmbassador;
import org.eclipse.mosaic.lib.util.objects.ObjectInstantiation;
import org.eclipse.mosaic.rti.api.AbstractFederateAmbassador;
import org.eclipse.mosaic.rti.api.InternalFederateException;
import org.eclipse.mosaic.rti.api.parameters.AmbassadorParameter;

import gov.dot.fhwa.saxton.CarmaV2xMessageReceiver;
/**
 * Implementation of a {@link AbstractFederateAmbassador} for CARMA message
 * ambassador.
 */
public class CarmaMessageAmbassador extends CommonMessageAmbassador<CarmaInstanceManager, CarmaRegistrationReceiver, CarmaRegistrationMessage, CarmaConfiguration> {

    /**
     * CarmaMessageAmbassador configuration file.
     */
    CarmaConfiguration carmaConfiguration;

    private CarmaRegistrationReceiver carmaRegistrationReceiver;
    private Thread registrationRxBackgroundThread;
    private CarmaV2xMessageReceiver v2xMessageReceiver;
    private Thread v2xRxBackgroundThread;
    private static CarmaInstanceManager carmaInstanceManager = new CarmaInstanceManager();
    private int timeSyncSeq = 0;
        
    
    
    /**
     * Create a new {@link CarmaMessageAmbassador} object.
     *
     * @param ambassadorParameter includes parameters for the
     *                            CarmaMessageAmbassador.
     */
    public CarmaMessageAmbassador(AmbassadorParameter ambassadorParameter) {
        super(ambassadorParameter, carmaInstanceManager, CarmaRegistrationMessage.class, CarmaConfiguration.class);

        try {
        // Read the CARMA message ambassador configuration file
            carmaConfiguration = new ObjectInstantiation<>(CarmaConfiguration.class, log)
                    .readFile(ambassadorParameter.configuration);
        } catch (InstantiationException e) {
            log.error("Configuration object could not be instantiated: ", e);
        }

    log.info("The update interval of CARMA message ambassador is " + carmaConfiguration.updateInterval + " .");

    // Check the CARMA update interval
    if (carmaConfiguration.updateInterval <= 0) {
        throw new RuntimeException("Invalid update interval for CARMA message ambassador, should be >0.");
    }
    log.info("CARMA message ambassador is generated.");
    }

    @Override
    public synchronized void processTimeAdvanceGrant(long time) throws InternalFederateException {
        if (time < currentSimulationTime) {
            // process time advance only if time is equal or greater than the next
            // simulation time step
            return;
        }
        super.processTimeAdvanceGrant(time);
    }
}
