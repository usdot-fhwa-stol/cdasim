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

package org.eclipse.mosaic.fed.infrastructure.ambassador;

import org.eclipse.mosaic.rti.TIME;
import org.eclipse.mosaic.rti.api.AbstractFederateAmbassador;
import org.eclipse.mosaic.rti.api.IllegalValueException;
import org.eclipse.mosaic.rti.api.Interaction;
import org.eclipse.mosaic.rti.api.InternalFederateException;
import org.eclipse.mosaic.rti.api.parameters.AmbassadorParameter;
import org.eclipse.mosaic.fed.infrastructure.configuration.InfrastructureConfiguration;
import org.eclipse.mosaic.lib.enums.AdHocChannel;
import org.eclipse.mosaic.lib.objects.communication.AdHocConfiguration;
import org.eclipse.mosaic.lib.objects.communication.InterfaceConfiguration;
import org.eclipse.mosaic.lib.util.objects.ObjectInstantiation;
import org.eclipse.mosaic.interactions.application.ExternalMessage;
import org.eclipse.mosaic.interactions.application.InfrastructureV2xMessageReception;
import org.eclipse.mosaic.interactions.communication.AdHocCommunicationConfiguration;
import org.eclipse.mosaic.interactions.mapping.RsuRegistration;
import org.eclipse.mosaic.fed.infrastructure.ambassador.InfrastructureRegistrationMessage;

import java.net.Inet4Address;
import java.net.InetAddress;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
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

    /**
     * The number of CARMA vehicles.
     */
    int numberOfInfrastructureInstances = 0;

    private InfrastructureRegistrationReceiver infrastructureRegistrationReceiver;
    private Thread registrationRxBackgroundThread;
    private InfrastructureTimeMessageReceiver infrastructureTimeMessageReceiver;
    private Thread v2xTimeRxBackgroundThread;
    private InfrastructureInstanceManager infrastructureInstanceManager = new InfrastructureInstanceManager();

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
        try {
            rti.requestAdvanceTime(currentSimulationTime, 0, (byte) 1);
        } catch (IllegalValueException e) {
            log.error("Error during advanceTime request", e);
            throw new InternalFederateException(e);
        }

        infrastructureRegistrationReceiver = new InfrastructureRegistrationReceiver();
        infrastructureRegistrationReceiver.init();
        registrationRxBackgroundThread = new Thread(infrastructureRegistrationReceiver);
        registrationRxBackgroundThread.start();
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

        try {
            
            // 
            List<InfrastructureRegistrationMessage> newRegistrations = infrastructureRegistrationReceiver
                    .getReceivedMessages();
            for (InfrastructureRegistrationMessage reg : newRegistrations) {

                // new instance registration to store to infrastructure instance manager
                infrastructureInstanceManager.onNewRegistration(reg);

                // register RSU, currently one infrastructure pair with one RSU, the ID will be the same as result
                // group and applications leave as null for now
                RsuRegistration rsuRegistration = new RsuRegistration(currentSimulationTime, reg.getInfrastructureId(),
                                                                      null, null,
                                                                      reg.getLocation());
                try {
                    // trigger RTI interaction to MOSAIC
                    this.rti.triggerInteraction(rsuRegistration);
                } catch (InternalFederateException | IllegalValueException e) {
                    log.error(e.getMessage());
                }

                //TODO actions to create DSRC parameter and then send to RTI interaction to MOSAIC

                // Create an AdHocConfiguration object to represent the configuration of the Ad-Hoc interface
                InterfaceConfiguration interfaceConfig = new InterfaceConfiguration.Builder(AdHocChannel.SCH1) 
                    .power(50) 
                    .radius(100.0) 
                    .secondChannel(AdHocChannel.SCH2) 
                    .create(); 

                AdHocConfiguration adHocConfig = new AdHocConfiguration.Builder(reg.getInfrastructureId())
                        .addInterface(interfaceConfig)
                        .create();

                AdHocCommunicationConfiguration communicationConfig = new AdHocCommunicationConfiguration(currentSimulationTime, adHocConfig);

                // Use the object to exchange the Ad-Hoc configuration with another vehicle or component
                try {
                    // trigger RTI interaction to MOSAIC
                    this.rti.triggerInteraction(communicationConfig);
                } catch (InternalFederateException | IllegalValueException e) {
                    log.error(e.getMessage());
                }
            }

            // TODO actions to do on queued Infrastructure time sync messages

            // TODO actions to do on queued v2x message receiver's received messages

            currentSimulationTime += infrastructureConfiguration.updateInterval * TIME.MILLI_SECOND;

            rti.requestAdvanceTime(currentSimulationTime, 0, (byte) 2);

            // send RSU external message
            String message = "External Message to RSU: RSU sent message at time: " + currentSimulationTime;
            ExternalMessage rsuMessage = new ExternalMessage(currentSimulationTime, message,
                    infrastructureConfiguration.senderRSUId);
            try {
                this.rti.triggerInteraction(rsuMessage);
            } catch (InternalFederateException | IllegalValueException e) {
                log.error(e.getMessage());
            }

        } catch (IllegalValueException e) {
            log.error("Error during advanceTime(" + time + ")", e);
            throw new InternalFederateException(e);
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
        return true;
    }
}
