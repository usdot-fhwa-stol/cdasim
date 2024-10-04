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
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.List;

import org.eclipse.mosaic.fed.application.ambassador.SimulationKernel;
import org.eclipse.mosaic.fed.carma.ambassador.CarmaMessageAmbassador;
import org.eclipse.mosaic.fed.carmamessenger.configuration.CarmaMessengerConfiguration;
import org.eclipse.mosaic.interactions.communication.V2xMessageTransmission;
import org.eclipse.mosaic.lib.misc.Tuple;
import org.eclipse.mosaic.lib.util.objects.ObjectInstantiation;
import org.eclipse.mosaic.rti.TIME;
import org.eclipse.mosaic.rti.api.IllegalValueException;
import org.eclipse.mosaic.rti.api.InternalFederateException;
import org.eclipse.mosaic.rti.api.parameters.AmbassadorParameter;

import gov.dot.fhwa.saxton.CarmaV2xMessage;
import gov.dot.fhwa.saxton.CarmaV2xMessageReceiver;
import gov.dot.fhwa.saxton.TimeSyncMessage;

public class CarmaMessengerMessageAmbassador extends CarmaMessageAmbassador{
    /**
     * Simulation time.
     */
    long currentSimulationTime;

    /**
     * CarmaMessageAmbassador configuration file.
     */
    CarmaMessengerConfiguration carmaMessengerConfiguration;

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
        super(ambassadorParameter);

        try {
            // Read the CARMA message ambassador configuration file
            carmaMessengerConfiguration = new ObjectInstantiation<>(CarmaMessengerConfiguration.class, log)
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

    /**
     * This method is called to tell the federate the start time and the end time.
     * 
     * @param startTime Start time of the simulation run in nano seconds.
     * @param endTime   End time of the simulation run in nano seconds.
     * @throws InternalFederateException Exception is thrown if an error is occurred
     *                                   while execute of a federate.
     */
    @Override
    public void initialize(long startTime, long endTime) throws InternalFederateException {
        currentSimulationTime = startTime;
        try {
            rti.requestAdvanceTime(currentSimulationTime, 0, (byte) 1);
        } catch (IllegalValueException e) {
            log.error("Error during advanceTime request", e);
            throw new InternalFederateException(e);
        }

        // Initialize listener socket and thread for CARMA Registration messages
        carmaMessengerRegistrationReceiver = new CarmaMessengerRegistrationReceiver();
        carmaMessengerRegistrationReceiver.init();
        registrationRxBackgroundThread = new Thread(carmaMessengerRegistrationReceiver);
        registrationRxBackgroundThread.start();

        // Initialize listener socket and thread for CARMA NS-3 Adapter messages
        v2xMessageReceiver = new CarmaV2xMessageReceiver();
        v2xMessageReceiver.init();
        v2xRxBackgroundThread = new Thread(v2xMessageReceiver);
        v2xRxBackgroundThread.start();
    }

        /**
     * This method is called by the AbstractFederateAmbassador when a time advance
     * has been granted by the RTI. Before this call is placed, any unprocessed
     * interaction is forwarded to the federate using the processInteraction method.
     *
     * @param time The timestamp towards which the federate can advance it local
     *             time.
     */
    @Override
    public synchronized void processTimeAdvanceGrant(long time) throws InternalFederateException {

        if (time < currentSimulationTime) {
            // process time advance only if time is equal or greater than the next
            // simulation time step
            return;
        }
        log.info("Carma message ambassador processing timestep to {}.", time);

        try {
            List<CarmaMessengerRegistrationMessage> newRegistrations = carmaMessengerRegistrationReceiver.getReceivedMessengerMessages();
            for (CarmaMessengerRegistrationMessage reg : newRegistrations) {
                carmaMessengerInstanceManager.onNewRegistration(reg);
                Method method = CarmaMessageAmbassador.class.getDeclaredMethod("onDsrcRegistrationRequest");
                method.setAccessible(true);
                method.invoke(this, reg.getCarlaVehicleRole());
            }
            // Set current simulation time to most recent time update
            currentSimulationTime = time;
            if (currentSimulationTime == 0) {
                // For the first timestep, clear the message receive queues.
                v2xMessageReceiver.getReceivedMessages(); // Automatically empties the queues.
            } else {
                List<Tuple<InetAddress, CarmaV2xMessage>> newMessages = v2xMessageReceiver.getReceivedMessages();
                for (Tuple<InetAddress, CarmaV2xMessage> msg : newMessages) {
                    V2xMessageTransmission msgInt = carmaMessengerInstanceManager.onV2XMessageTx(msg.getA(), msg.getB(), currentSimulationTime);
                    log.debug("Generated a message with ID: {}", msgInt.getMessageId());
                    SimulationKernel.SimulationKernel.getV2xMessageCache().putItem(currentSimulationTime, msgInt.getMessage());
                    rti.triggerInteraction(msgInt);
                }
            }
            // Time Syncmessage in nano seconds
            TimeSyncMessage timeSyncMessage = new TimeSyncMessage(currentSimulationTime, timeSyncSeq);
            carmaMessengerInstanceManager.onTimeStepUpdate(timeSyncMessage);
            // Increment time 
            currentSimulationTime += carmaMessengerConfiguration.updateInterval * TIME.MILLI_SECOND;
            timeSyncSeq += 1;
           
            rti.requestAdvanceTime(currentSimulationTime, 0, (byte) 2);
        } catch (IllegalValueException e) {
            log.error("Error during advanceTime(" + time + ")", e);
            throw new InternalFederateException(e);
        } catch (UnknownHostException e) {
            log.error("Error during advanceTime(" + time + ")", e);
            throw new InternalFederateException(e);
        } catch (IOException e) {
            log.error("Error during advanceTime(" + time + ")", e);
            throw new InternalFederateException(e);
        } catch (NoSuchMethodException e) {
            log.error("Error during advanceTime(" + time + ")", e);
            throw new InternalFederateException(e);
        } catch (IllegalAccessException e) {
            log.error("Error during advanceTime(" + time + ")", e);
            throw new InternalFederateException(e);
        } catch (IllegalArgumentException e) {
            log.error("Error during advanceTime(" + time + ")", e);
            throw new InternalFederateException(e);
        } catch (InvocationTargetException e) {
            log.error("Error during advanceTime(" + time + ")", e);
            throw new InternalFederateException(e);
        }
         
    }
}
