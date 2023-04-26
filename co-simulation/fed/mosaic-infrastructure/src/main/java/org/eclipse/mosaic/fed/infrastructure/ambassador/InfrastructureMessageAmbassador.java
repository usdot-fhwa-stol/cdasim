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

import gov.dot.fhwa.saxton.CarmaV2xMessage;
import gov.dot.fhwa.saxton.CarmaV2xMessageReceiver;
import org.eclipse.mosaic.fed.infrastructure.configuration.InfrastructureConfiguration;
import org.eclipse.mosaic.interactions.application.ExternalMessage;
import org.eclipse.mosaic.interactions.application.InfrastructureV2xMessageReception;
import org.eclipse.mosaic.interactions.communication.AdHocCommunicationConfiguration;
import org.eclipse.mosaic.interactions.communication.V2xMessageTransmission;
import org.eclipse.mosaic.interactions.mapping.RsuRegistration;
import org.eclipse.mosaic.lib.enums.AdHocChannel;
import org.eclipse.mosaic.lib.geo.GeoPoint;
import org.eclipse.mosaic.lib.misc.Tuple;
import org.eclipse.mosaic.lib.objects.communication.AdHocConfiguration;
import org.eclipse.mosaic.lib.objects.communication.InterfaceConfiguration;
import org.eclipse.mosaic.lib.util.objects.ObjectInstantiation;
import org.eclipse.mosaic.rti.TIME;
import org.eclipse.mosaic.rti.api.AbstractFederateAmbassador;
import org.eclipse.mosaic.rti.api.IllegalValueException;
import org.eclipse.mosaic.rti.api.Interaction;
import org.eclipse.mosaic.rti.api.InternalFederateException;
import org.eclipse.mosaic.rti.api.parameters.AmbassadorParameter;

import java.io.IOException;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Collections;
import java.util.List;

/**
 * Implementation of a {@link AbstractFederateAmbassador} for Infrastructure
 * message ambassador.
 */
public class InfrastructureMessageAmbassador extends AbstractFederateAmbassador {

    /**
     * Simulation time.
     */
    long currentSimulationTime;

    /**
     * InfrastructureMessageAmbassador configuration file.
     */
    InfrastructureConfiguration infrastructureConfiguration;

    private InfrastructureRegistrationReceiver infrastructureRegistrationReceiver;
    private Thread registrationRxBackgroundThread;

    private CarmaV2xMessageReceiver v2xMessageReceiver = new CarmaV2xMessageReceiver(1515);
    private Thread v2xMessageBackgroundThread;

    private InfrastructureInstanceManager infrastructureInstanceManager = new InfrastructureInstanceManager();
    private InfrastructureTimeInterface infrastructureTimeInterface = new InfrastructureTimeInterface(
            infrastructureInstanceManager);

    private int timeSyncSeq = 0;

    /**
     * Create a new {@link InfrastructureMessageAmbassador} object.
     *
     * @param ambassadorParameter includes parameters for the
     *                            InfrastructureMessageAmbassador.
     */
    public InfrastructureMessageAmbassador(AmbassadorParameter ambassadorParameter) {
        super(ambassadorParameter);

        // load configuration file
        try {
            infrastructureConfiguration = new ObjectInstantiation<>(InfrastructureConfiguration.class, log)
                    .readFile(ambassadorParameter.configuration);
        } catch (InstantiationException e) {
            log.error("Configuration object could not be instantiated: ", e);
        }

        log.info("The update interval of infrastructure message ambassador is {}.",
                infrastructureConfiguration.updateInterval);

        if (infrastructureConfiguration.updateInterval <= 0) {
            throw new RuntimeException("Invalid update interval for infrastructure message ambassador, should be >0.");
        }
        log.info("Infrastructure message ambassador is generated.");
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
        super.initialize(startTime, endTime);
        currentSimulationTime = startTime;

        infrastructureRegistrationReceiver = new InfrastructureRegistrationReceiver();
        infrastructureRegistrationReceiver.init();
        registrationRxBackgroundThread = new Thread(infrastructureRegistrationReceiver);
        registrationRxBackgroundThread.start();

        // TODO: Port 1517 assumed to be available. Need to double check.
        v2xMessageReceiver = new CarmaV2xMessageReceiver(1517);
        v2xMessageReceiver.init();
        v2xMessageBackgroundThread = new Thread(v2xMessageReceiver);
        v2xMessageBackgroundThread.start();

        try {
            rti.requestAdvanceTime(currentSimulationTime, 0, (byte) 1);
        } catch (IllegalValueException e) {
            log.error("Error during advanceTime request", e);
            throw new InternalFederateException(e);
        }
    }

    /**
     * This method processes the interactions.
     *
     * @param interaction The interaction that can be processed.
     * @throws InternalFederateException Exception is thrown if an error is occurred
     *                                   while execute of a federate.
     */
    @Override
    public void processInteraction(Interaction interaction) throws InternalFederateException {
        String type = interaction.getTypeId();
        long interactionTime = interaction.getTime();
        log.trace("Process interaction with type '{}' at time: {}", type, interactionTime);
        // Infrastructure message reception
        if (interaction.getTypeId().equals(InfrastructureV2xMessageReception.TYPE_ID)) {
            this.receiveInteraction((InfrastructureV2xMessageReception) interaction);
        }
    }

    /**
     * Extract external message from received
     * {@link InfrastructureV2xMessageReception} interaction.
     *
     * @param infrastructureV2xMessageReception Interaction indicates that the
     *                                          external message is received by a
     *                                          rsu.
     */
    private synchronized void receiveInteraction(InfrastructureV2xMessageReception infrastructureV2xMessageReception) {
        log.info(infrastructureV2xMessageReception.getReceiverID() + " received V2X messages at time: "
                + infrastructureV2xMessageReception.getTime() + ".");
        log.info("The received message is " + infrastructureV2xMessageReception.getMessage() + " .");
    }

    /**
     *
     * Creates an Ad-Hoc configuration object to represent the configuration of the
     * Ad-Hoc interface used for communication between the infrastructure instance
     * and other vehicles or components, and sends it to the RTI for exchange.
     *
     * @param reg the infrastructure registration message received from the RTI
     *
     *            Note: This function should be called after the
     *            onRsuRegistrationRequest
     * @throws UnknownHostException
     */
    private void onDsrcRegistrationRequest(String infrastructureId) throws UnknownHostException {
        // Create an InterfaceConfiguration object to represent the configuration of the
        // Ad-Hoc interface
        // TODO: Replace the IP address of the ad-hoc interface if necessary
        // TODO: Replace the subnet mask of the ad-hoc interface if necessary
        // TODO: Replace the transmit power of the ad-hoc interface (in dBm) if necessary
        // TODO: Replace the communication range of the ad-hoc interface (in meters) if necessary
        InterfaceConfiguration interfaceConfig = new InterfaceConfiguration.Builder(AdHocChannel.SCH1)
                .ip((Inet4Address) Inet4Address.getByName("192.168.0.1"))
                .subnet((Inet4Address) Inet4Address.getByName("255.255.255.0"))
                .power(50)
                .radius(100.0)
                .create();

        // Create an AdHocConfiguration object to associate the Ad-Hoc interface
        // configuration with the infrastructure instance's ID
        AdHocConfiguration adHocConfig = new AdHocConfiguration.Builder(infrastructureId)
                .addInterface(interfaceConfig)
                .create();

        // Create an AdHocCommunicationConfiguration object to specify the time and
        // Ad-Hoc configuration for exchange with another vehicle or component
        AdHocCommunicationConfiguration communicationConfig = new AdHocCommunicationConfiguration(currentSimulationTime,
                adHocConfig);

        try {
            // Trigger RTI interaction to MOSAIC to exchange the Ad-Hoc configuration
            this.rti.triggerInteraction(communicationConfig);
        } catch (InternalFederateException | IllegalValueException e) {
            // Log error message if there was an issue with the RTI interaction
            log.error(e.getMessage());
        }
    }


    /**
     * Performs registration of a new infrastructure instance and sets up
     * communication interfaces.
     *
     * @param reg The registration message of the new infrastructure instance.
     */
    private void onRsuRegistrationRequest(String infrastructureId, GeoPoint location) {

        // Register the new infrastructure instance to the RTI as an RSU
        RsuRegistration rsuRegistration = new RsuRegistration(currentSimulationTime, infrastructureId, "",
                Collections.emptyList(), location);
        try {
            // Trigger RTI interaction to MOSAIC
            this.rti.triggerInteraction(rsuRegistration);
        } catch (InternalFederateException | IllegalValueException e) {
            // Log error message if there was an issue with the RTI interaction
            log.error(e.getMessage());
        }
    }

    /**
     * This method is called by the AbstractFederateAmbassador when the RTI grants a
     * time advance to the federate. Any unprocessed interactions are forwarded to
     * the federate using the processInteraction method before this call is made.
     *
     * @param time The timestamp (in nanoseconds) indicating the time to which the federate can
     *             advance its local time.
     */
    @Override
    public synchronized void processTimeAdvanceGrant(long time) throws InternalFederateException {

        // Process the time advance only if the time is equal or greater than the next
        // simulation time step
        if (time < currentSimulationTime) {
            return;
        }
        try {

            // Handle any new infrastructure registration requests
            List<InfrastructureRegistrationMessage> newRegistrations = infrastructureRegistrationReceiver
                    .getReceivedMessages();
            for (InfrastructureRegistrationMessage reg : newRegistrations) {
                // Store new instance registration to infrastructure instance manager
                infrastructureInstanceManager.onNewRegistration(reg);
                // Process registration requests for RSUs and DSRCs
                onRsuRegistrationRequest(reg.getInfrastructureId(), reg.getLocation());
                onDsrcRegistrationRequest(reg.getInfrastructureId());
            }

            List<Tuple<InetAddress, CarmaV2xMessage>> newMessages = v2xMessageReceiver.getReceivedMessages();
            for (Tuple<InetAddress, CarmaV2xMessage> msg : newMessages) {
                V2xMessageTransmission msgInt = infrastructureInstanceManager.onV2XMessageTx(msg.getA(), msg.getB());
                this.rti.triggerInteraction(msgInt);
            }

            timeSyncSeq += 1;
            InfrastructureTimeMessage timeSyncMessage = new InfrastructureTimeMessage();
            timeSyncMessage.setSeq(timeSyncSeq);
            // nanoseconds to milliseconds for InfrastructureTimeMessage
            timeSyncMessage.setTimestep(currentSimulationTime/1000000);
            infrastructureTimeInterface.onTimeStepUpdate(timeSyncMessage);

            // TODO: Handle any queued V2X message receiver's received messages

            // Advance the simulation time
            currentSimulationTime += infrastructureConfiguration.updateInterval * TIME.MILLI_SECOND;

            // Request the next time advance from the RTI
            rti.requestAdvanceTime(currentSimulationTime, 0, (byte) 2);

            // Send an external message to the RSU
            String message = "External Message to RSU: RSU sent message at time: " + currentSimulationTime;
            ExternalMessage rsuMessage = new ExternalMessage(currentSimulationTime, message,
                    infrastructureConfiguration.senderRSUId);
            try {
                // Trigger RTI interaction to MOSAIC
                this.rti.triggerInteraction(rsuMessage);
            } catch (InternalFederateException | IllegalValueException e) {
                // Log an error message if there was an issue with the RTI interaction
                log.error(e.getMessage());
            }

        } catch (IllegalValueException e) {
            log.error("Error during advanceTime(" + time + ")", e);
            throw new InternalFederateException(e);
        } catch (IOException e1) {
            log.error("Error during updating timestep :" + e1.getMessage());
        }
    }

    /**
     * Return whether this federate is time constrained. Is set if the federate is
     * sensitive towards the correct ordering of events. The federate ambassador
     * will ensure that the message processing happens in time stamp order. If set
     * to false, interactions will be processed in receive order.
     *
     * @return {@code true} if this federate is time constrained, else
     *         {@code false}.
     */
    @Override
    public boolean isTimeConstrained() {
        return true;
    }

    /**
     * Return whether this federate is time regulating. Is set if the federate
     * influences other federates and can prevent them from advancing their local
     * time.
     *
     * @return {@code true} if this federate is time regulating, {@code false} else.
     */
    @Override
    public boolean isTimeRegulating() {
        return false;
    }

    /**
     * Test helper function to cleanup sockets and threads.
     */
    protected void close() {
        v2xMessageReceiver.stop();
        infrastructureRegistrationReceiver.stop();
    }
}
