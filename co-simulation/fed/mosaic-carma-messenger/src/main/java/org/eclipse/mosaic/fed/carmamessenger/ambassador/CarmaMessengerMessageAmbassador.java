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

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.eclipse.mosaic.interactions.application.MsgerRequesetTrafficEvent;
import org.eclipse.mosaic.interactions.application.MsgerResponseTrafficEvent;
import org.eclipse.mosaic.lib.CommonUtil.ambassador.CommonMessageAmbassador;
import org.eclipse.mosaic.lib.CommonUtil.configuration.CommonConfiguration;
import org.eclipse.mosaic.lib.util.objects.ObjectInstantiation;
import org.eclipse.mosaic.rti.api.Interaction;
import org.eclipse.mosaic.rti.api.InternalFederateException;
import org.eclipse.mosaic.rti.api.parameters.AmbassadorParameter;

public class CarmaMessengerMessageAmbassador extends CommonMessageAmbassador<CarmaMessengerInstanceManager, CarmaMessengerRegistrationReceiver, CarmaMessengerRegistrationMessage, CommonConfiguration>{

    private static CarmaMessengerInstanceManager instanceManager;
    private CarmaMessengerBridgeRegistrationReceiver bridgeReceiver;
    private Thread BridgeRegistrationRxBackgroundThread;

    public CarmaMessengerMessageAmbassador(AmbassadorParameter ambassadorParameter) {
        // Use the static instanceManager if no specific one is provided
        this(ambassadorParameter, getInstanceManager());
    }

    /**
     * Create a new {@link CarmaMessengerMessageAmbassador} object.
     *
     * @param ambassadorParameter includes parameters for the
     *                            CarmaMessageAmbassador.
     */
    public CarmaMessengerMessageAmbassador(AmbassadorParameter ambassadorParameter, CarmaMessengerInstanceManager instanceManager) {
        super(ambassadorParameter, instanceManager, CarmaMessengerRegistrationMessage.class, CommonConfiguration.class);
        CarmaMessengerMessageAmbassador.instanceManager = instanceManager;
        try {
            // Read the CARMA message ambassador configuration file
            commonConfiguration = new ObjectInstantiation<>(CommonConfiguration.class, log)
                    .readFile(ambassadorParameter.configuration);
        } catch (InstantiationException e) {
            log.error("Configuration object could not be instantiated: ", e);
        }

        log.info("The update interval of CARMA message ambassador is " + commonConfiguration.updateInterval + " .");

        // Check the CARMA update interval
        if (commonConfiguration.updateInterval <= 0) {
            throw new RuntimeException("Invalid update interval for CARMA message ambassador, should be >0.");
        }
        log.info("CARMA message ambassador is generated.");
    }

    @Override
    public void initialize(long startTime, long endTime) throws InternalFederateException{
        super.initialize(startTime, endTime);
        bridgeReceiver = new CarmaMessengerBridgeRegistrationReceiver();
        bridgeReceiver.init();
        BridgeRegistrationRxBackgroundThread = new Thread(bridgeReceiver);
        BridgeRegistrationRxBackgroundThread.start();
    }

    @Override
    public synchronized void processTimeAdvanceGrant(long time) throws InternalFederateException {
        List<String> temp = new ArrayList<>();
        temp = this.commonInstanceManager.getVehicleIds();
        String parameterName = "";
        for(String id : temp){
            try {
                rti.triggerInteraction(new MsgerRequesetTrafficEvent(time, id, parameterName));
            } catch (Exception e) {
                log.error("error: " + e.getMessage());
            } 
        }
        try{
            List<CarmaMessengerBridgeRegistrationMessage> newBridgeRegistrations = bridgeReceiver.getReceivedMessages();
                for (CarmaMessengerBridgeRegistrationMessage reg : newBridgeRegistrations) {
                    commonInstanceManager.onMsgrNewRegistration(reg);
                    super.onDsrcRegistrationRequest(reg.getVehicleRole());
                }
        }catch(Exception e){
            log.error("error: " + e.getMessage());
        }
        super.processTimeAdvanceGrant(time);
    }

    @Override
    public void processInteraction(Interaction interaction) throws InternalFederateException {
        String type = interaction.getTypeId();
        long interactionTime = interaction.getTime();
        log.trace("Process interaction with type '{}' at time: {}", type, interactionTime);
        if (interaction.getTypeId().equals(MsgerResponseTrafficEvent.TYPE_ID)) {
            receiveMsgerResponseTrafficEventInteraction((MsgerResponseTrafficEvent) interaction);
        }
        else{
            super.processInteraction(interaction);
        }       
    }


    private void receiveMsgerResponseTrafficEventInteraction(MsgerResponseTrafficEvent interaction)
    {    
        try {
            this.commonInstanceManager.onDetectedTrafficEvents(interaction.getTrafficEvent());
        } catch (IOException e) {    
            log.error(e.getMessage());
        }   
    }

    public static CarmaMessengerInstanceManager getInstanceManager() {
        // Check if the instance manager is already initialized
        if (instanceManager == null) {
            // Initialize the instance manager if it's not yet created
            instanceManager = new CarmaMessengerInstanceManager();
        }
        return instanceManager;
    }
  
}
