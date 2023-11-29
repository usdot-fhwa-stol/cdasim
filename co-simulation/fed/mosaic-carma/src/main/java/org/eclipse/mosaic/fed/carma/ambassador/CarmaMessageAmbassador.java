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

import gov.dot.fhwa.saxton.CarmaV2xMessage;
import gov.dot.fhwa.saxton.CarmaV2xMessageReceiver;
import org.eclipse.mosaic.fed.application.ambassador.SimulationKernel;
import org.eclipse.mosaic.fed.carma.configuration.CarmaConfiguration;
import org.eclipse.mosaic.interactions.application.CarmaV2xMessageReception;
import org.eclipse.mosaic.interactions.communication.AdHocCommunicationConfiguration;
import org.eclipse.mosaic.interactions.communication.V2xMessageReception;
import org.eclipse.mosaic.interactions.communication.V2xMessageTransmission;
import org.eclipse.mosaic.interactions.mapping.advanced.ExternalVehicleRegistration;
import org.eclipse.mosaic.interactions.traffic.VehicleUpdates;
import org.eclipse.mosaic.lib.enums.AdHocChannel;
import org.eclipse.mosaic.lib.enums.DriveDirection;
import org.eclipse.mosaic.lib.geo.CartesianPoint;
import org.eclipse.mosaic.lib.geo.GeoPoint;
import org.eclipse.mosaic.lib.misc.Tuple;
import org.eclipse.mosaic.lib.objects.addressing.IpResolver;
import org.eclipse.mosaic.lib.objects.communication.AdHocConfiguration;
import org.eclipse.mosaic.lib.objects.communication.InterfaceConfiguration;
import org.eclipse.mosaic.lib.objects.road.IRoadPosition;
import org.eclipse.mosaic.lib.objects.road.SimpleRoadPosition;
import org.eclipse.mosaic.lib.objects.v2x.ExternalV2xMessage;
import org.eclipse.mosaic.lib.objects.v2x.V2xMessage;
import org.eclipse.mosaic.lib.objects.vehicle.*;
import org.eclipse.mosaic.lib.objects.vehicle.sensor.DistanceSensor;
import org.eclipse.mosaic.lib.objects.vehicle.sensor.RadarSensor;
import org.eclipse.mosaic.lib.util.objects.ObjectInstantiation;
import org.eclipse.mosaic.rti.TIME;
import org.eclipse.mosaic.rti.api.AbstractFederateAmbassador;
import org.eclipse.mosaic.rti.api.IllegalValueException;
import org.eclipse.mosaic.rti.api.Interaction;
import org.eclipse.mosaic.rti.api.InternalFederateException;
import org.eclipse.mosaic.rti.api.parameters.AmbassadorParameter;

import javax.xml.bind.DatatypeConverter;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Implementation of a {@link AbstractFederateAmbassador} for CARMA message
 * ambassador.
 */
public class CarmaMessageAmbassador extends AbstractFederateAmbassador {

    /**
     * Simulation time.
     */
    long currentSimulationTime;

    /**
     * CarmaMessageAmbassador configuration file.
     */
    CarmaConfiguration carmaConfiguration;

    private CarmaRegistrationReceiver carmaRegistrationReceiver;
    private Thread registrationRxBackgroundThread;
    private CarmaV2xMessageReceiver v2xMessageReceiver;
    private Thread v2xRxBackgroundThread;
    private CarmaInstanceManager carmaInstanceManager = new CarmaInstanceManager();

    /**
     * Create a new {@link CarmaMessageAmbassador} object.
     *
     * @param ambassadorParameter includes parameters for the
     *                            CarmaMessageAmbassador.
     */
    public CarmaMessageAmbassador(AmbassadorParameter ambassadorParameter) {
        super(ambassadorParameter);

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

        // Initialize listener socket and thread for CARMA Registration messages
        carmaRegistrationReceiver = new CarmaRegistrationReceiver();
        carmaRegistrationReceiver.init();
        registrationRxBackgroundThread = new Thread(carmaRegistrationReceiver);
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

        try {
            List<CarmaRegistrationMessage> newRegistrations = carmaRegistrationReceiver.getReceivedMessages();
            for (CarmaRegistrationMessage reg : newRegistrations) {
                carmaInstanceManager.onNewRegistration(reg);
                onDsrcRegistrationRequest(reg.getCarlaVehicleRole());
            }


            if (currentSimulationTime == 0) {
                // For the first timestep, clear the message receive queues.
                v2xMessageReceiver.getReceivedMessages(); // Automatically empties the queues.
            } else {
                List<Tuple<InetAddress, CarmaV2xMessage>> newMessages = v2xMessageReceiver.getReceivedMessages();
                for (Tuple<InetAddress, CarmaV2xMessage> msg : newMessages) {
                    V2xMessageTransmission msgInt = carmaInstanceManager.onV2XMessageTx(msg.getA(), msg.getB(), time);
                    SimulationKernel.SimulationKernel.getV2xMessageCache().putItem(currentSimulationTime, msgInt.getMessage());
                    rti.triggerInteraction(msgInt);
                }
            }

            currentSimulationTime += carmaConfiguration.updateInterval * TIME.MILLI_SECOND;

            rti.requestAdvanceTime(currentSimulationTime, 0, (byte) 2);
        } catch (IllegalValueException e) {
            log.error("Error during advanceTime(" + time + ")", e);
            throw new InternalFederateException(e);
        } catch (UnknownHostException e) {
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
        if (interaction.getTypeId().equals(V2xMessageReception.TYPE_ID)) {
            receiveV2xReceptionInteraction((V2xMessageReception) interaction);
        }
        if (interaction.getTypeId().equals(CarmaV2xMessageReception.TYPE_ID)) {
            receiveInteraction((CarmaV2xMessageReception) interaction);
        }
        if (interaction.getTypeId().equals(VehicleUpdates.TYPE_ID)) {
            receiveVehicleUpdateInteraction((VehicleUpdates) interaction);
        }
    }

    private synchronized void receiveVehicleUpdateInteraction(VehicleUpdates interaction) {
        carmaInstanceManager.onVehicleUpdates(interaction);
    }

    /**
     * Helper function to retrieve previously transmitted messages by ID from the buffer
     * @param id The id of the message to return
     * @return The {@link V2xMessage} object if the id exists in the buffer, null o.w.
     */
    private V2xMessage lookupV2xMsgIdInBuffer(int id) {
        return SimulationKernel.SimulationKernel.getV2xMessageCache().getItem(id);
    }

    /**
     * Callback to be invoked when the network simulator determines that a simulated radio has received a V2X message
     * @param interaction The v2x message receipt data
     */
    private synchronized void receiveV2xReceptionInteraction(V2xMessageReception interaction) {
        String carlaRoleName = interaction.getReceiverName();
        if (!carmaInstanceManager.checkIfRegistered(carlaRoleName)) {
            // Abort early as we only are concerned with CARMA Platform vehicles
            return;
        }
        log.info("Processing V2X message reception event for " + interaction.getReceiverName() + " of msg id " + interaction.getMessageId());

        int messageId = interaction.getMessageId();
        V2xMessage msg = lookupV2xMsgIdInBuffer(messageId);

        if (msg != null && msg instanceof ExternalV2xMessage) {
            ExternalV2xMessage msg2 = (ExternalV2xMessage) msg;
            carmaInstanceManager.onV2XMessageRx(DatatypeConverter.parseHexBinary(msg2.getMessage()), carlaRoleName);
            log.info("Sending V2X message reception event for " + interaction.getReceiverName() + " of msg id " + interaction.getMessageId() + " of size " + msg2.getPayLoad().getBytes().length);
        } else {
            log.warn("Message with id " + interaction.getMessageId() + " received by " + interaction.getReceiverName() + " is no longer in the message buffer to be retrieved! Message transmission failed!!!");
        }
    }

    private void onDsrcRegistrationRequest(String vehicleId) throws UnknownHostException {
        ExternalVehicleRegistration tempRegistration = new ExternalVehicleRegistration(
                currentSimulationTime,
                vehicleId,
                "carma",
                null,
                new VehicleType("carma"));

        try {
            // Trigger RTI interaction to MOSAIC to exchange the Ad-Hoc configuration
            this.rti.triggerInteraction(tempRegistration);
        } catch (InternalFederateException | IllegalValueException e) {
            // Log error message if there was an issue with the RTI interaction
            log.error(e.getMessage());
        }

        VehicleSignals tmpSignals  = new VehicleSignals(
                false,
                false,
                false,
                false,
                false);

        VehicleEmissions tmpEmissions = new VehicleEmissions(
                new Emissions(
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0
                ),
                new Emissions(
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0
                ));

        VehicleBatteryState tmpBattery = new VehicleBatteryState("", currentSimulationTime);
        IRoadPosition tmpPos = new SimpleRoadPosition("", 0, 0.0, 0.0);
        VehicleSensors tmpSensors = new VehicleSensors(
                new DistanceSensor(0.0,
                        0.0,
                        0.0,
                        0.0),
                new RadarSensor(0.0));
        VehicleConsumptions tmpConsumptions = new VehicleConsumptions(
                new Consumptions(0.0, 0.0),
                new Consumptions(0.0, 0.0));
        VehicleData tmpVehicle = new VehicleData.Builder(currentSimulationTime, vehicleId)
                .position(GeoPoint.ORIGO, CartesianPoint.ORIGO)
                .movement(0.0, 0.0, 0.0)
                .consumptions(tmpConsumptions)
                .emissions(tmpEmissions)
                .electric(tmpBattery)
                .laneArea("")
                .sensors(tmpSensors)
                .road(tmpPos)
                .signals(tmpSignals)
                .orientation(DriveDirection.FORWARD, 0.0, 0.0)
                .stopped(false)
                .route("")
                .create();
        VehicleUpdates tempUpdates = new VehicleUpdates(
                currentSimulationTime,
                new ArrayList<>(Arrays.asList(tmpVehicle)),
                new ArrayList<>(),
                new ArrayList<>());

        try {
            // Trigger RTI interaction to MOSAIC to exchange the Ad-Hoc configuration
            this.rti.triggerInteraction(tempUpdates);
        } catch (InternalFederateException | IllegalValueException e) {
            // Log error message if there was an issue with the RTI interaction
            log.error(e.getMessage());
        }


        // Create an InterfaceConfiguration object to represent the configuration of the
        // Ad-Hoc interface
        // TODO: Replace the transmit power of the ad-hoc interface (in dBm) if necessary
        // TODO: Replace the communication range of the ad-hoc interface (in meters) if necessary
        Inet4Address vehAddress = IpResolver.getSingleton().registerHost(vehicleId);
        log.info("Assigned registered comms device " + vehicleId + " with IP address " + vehAddress.toString());
        InterfaceConfiguration interfaceConfig = new InterfaceConfiguration.Builder(AdHocChannel.CCH)
                .ip(vehAddress)
                .subnet(IpResolver.getSingleton().getNetMask())
                .power(50)
                .radius(1000.0)
                .create();

        // Create an AdHocConfiguration object to associate the Ad-Hoc interface
        // configuration with the infrastructure instance's ID
        AdHocConfiguration adHocConfig = new AdHocConfiguration.Builder(vehicleId)
                .addInterface(interfaceConfig)
                .create();

        // Create an AdHocCommunicationConfiguration object to specify the time and
        // Ad-Hoc configuration for exchange with another vehicle or component
        AdHocCommunicationConfiguration communicationConfig = new AdHocCommunicationConfiguration(currentSimulationTime,
                adHocConfig);
        log.info("Communications comms device " + vehicleId + " with IP address " + vehAddress.toString() + " success!");
        try {
            // Trigger RTI interaction to MOSAIC to exchange the Ad-Hoc configuration
            this.rti.triggerInteraction(communicationConfig);
        } catch (InternalFederateException | IllegalValueException e) {
            // Log error message if there was an issue with the RTI interaction
            log.error(e.getMessage());
        }
    }
}
