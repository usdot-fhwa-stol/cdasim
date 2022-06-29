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

import org.eclipse.mosaic.rti.TIME;
import org.eclipse.mosaic.rti.api.AbstractFederateAmbassador;
import org.eclipse.mosaic.rti.api.IllegalValueException;
import org.eclipse.mosaic.rti.api.InternalFederateException;
import org.eclipse.mosaic.rti.api.parameters.AmbassadorParameter;

import org.eclipse.mosaic.fed.carma.configuration.CarmaConfiguration;
import org.eclipse.mosaic.fed.carma.configuration.CarmaVehicleConfiguration;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.eclipse.mosaic.rti.api.Interaction;
import org.eclipse.mosaic.lib.objects.vehicle.VehicleData;
import org.eclipse.mosaic.lib.objects.vehicle.VehicleDeparture;
import org.eclipse.mosaic.lib.util.objects.ObjectInstantiation;
import org.eclipse.mosaic.interactions.application.CarmaV2xMessageReception;
import org.eclipse.mosaic.interactions.application.ExternalMessage;
import org.eclipse.mosaic.interactions.traffic.VehicleUpdates;
import org.eclipse.mosaic.interactions.vehicle.VehicleFederateAssignment;
import org.eclipse.mosaic.lib.enums.DriveDirection;

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

    /**
     * List of vehicles that are controlled by CARMA platform.
     */
    private final HashMap<String, Boolean> carmaVehicleMap = new HashMap<>();

    /**
     * The number of CARMA vehicles.
     */
    int numberOfCarmaVehicle = 0;

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

        // Register CARMA vehicles
        for (CarmaVehicleConfiguration carmaVehicleConfiguration : carmaConfiguration.carmaVehicles) {
            VehicleDeparture vehicleDeparture = new VehicleDeparture.Builder(carmaVehicleConfiguration.routeID)
                    .departureLane(VehicleDeparture.LaneSelectionMode.BEST, carmaVehicleConfiguration.lane,
                            carmaVehicleConfiguration.position)
                    .departureSpeed(VehicleDeparture.DepartSpeedMode.MAXIMUM, carmaVehicleConfiguration.departSpeed)
                    .create();

            VehicleFederateAssignment carmaVehicleRegistration = new VehicleFederateAssignment(currentSimulationTime,
                    "carma_" + numberOfCarmaVehicle, "carma", 50, carmaVehicleConfiguration.vehicleType,
                    vehicleDeparture, carmaVehicleConfiguration.applications);

            numberOfCarmaVehicle++;

            try {
                this.rti.triggerInteraction(carmaVehicleRegistration);
                carmaVehicleMap.put(carmaVehicleRegistration.getVehicleId(), false);
            } catch (InternalFederateException | IllegalValueException e) {
                log.error(e.getMessage());
            }
        }
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
            // Update CARMA vehicle
            updateCarmaVehicles();

            currentSimulationTime += carmaConfiguration.updateInterval * TIME.MILLI_SECOND;

            rti.requestAdvanceTime(currentSimulationTime, 0, (byte) 2);

            // Send CARMA external messages
            String message = "Message to CARMA vehicles: Carma Vehicles update at time: " + currentSimulationTime;
            ExternalMessage externalMessage = new ExternalMessage(currentSimulationTime, message,
                    carmaConfiguration.senderCarmaVehicleId);
            try {
                this.rti.triggerInteraction(externalMessage);
            } catch (InternalFederateException | IllegalValueException e) {
                log.error(e.getMessage());
            }
            log.debug("trigger external message interaction at time: " + currentSimulationTime + " .");

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
        if (interaction.getTypeId().equals(CarmaV2xMessageReception.TYPE_ID)) {
            receiveInteraction((CarmaV2xMessageReception) interaction);
        }
    }

    /**
     * Extract external message from received {@link CarmaV2xMessageReception}
     * interaction.
     *
     * @param carmaV2xMessageReception Interaction indicates that the external
     *                                 message is received by a CARMA vehicle.
     * @throws InternalFederateException Exception if a invalid value is used.
     */
    private synchronized void receiveInteraction(CarmaV2xMessageReception carmaV2xMessageReception)
            throws InternalFederateException {
        log.info("CARMA vehicle: " + carmaV2xMessageReception.getReceiverID()
                + " received an external message at time: " + carmaV2xMessageReception.getTime() + ".");
        log.info("The received message is " + carmaV2xMessageReception.getMessage() + " .");

    }

    /**
     * Update CARMA vehicles position.
     */
    private final void updateCarmaVehicles() {

        List<VehicleData> addedVehicle = new ArrayList<>();
        List<VehicleData> updatedVehicle = new ArrayList<>();
        List<String> removedNames = new ArrayList<>();

        for (String vehicleName : carmaVehicleMap.keySet()) {
            final int firstUnderscorePosition = vehicleName.indexOf('_');
            final int vehicleNumber = Integer.parseInt(vehicleName.substring(firstUnderscorePosition + 1));

            VehicleData vehicleData = new VehicleData.Builder(currentSimulationTime, vehicleName)
                    .position(carmaConfiguration.carmaVehicles.get(vehicleNumber).geoPosition,
                            carmaConfiguration.carmaVehicles.get(vehicleNumber).projectedPosition)
                    .orientation(DriveDirection.UNAVAILABLE,
                            carmaConfiguration.carmaVehicles.get(vehicleNumber).heading,
                            carmaConfiguration.carmaVehicles.get(vehicleNumber).slope)
                    .create();

            if (!carmaVehicleMap.get(vehicleName)) {
                // update the new added CARMA vehicles
                addedVehicle.add(vehicleData);
                carmaVehicleMap.put(vehicleName, true);
            } else {
                // update the registed CARMA vehicles
                updatedVehicle.add(vehicleData);
            }
        }

        VehicleUpdates vehicleUpdates = new VehicleUpdates(currentSimulationTime, addedVehicle, updatedVehicle,
                removedNames);

        // trigger VehicleUpdates interaction
        try {
            this.rti.triggerInteraction(vehicleUpdates);
        } catch (InternalFederateException | IllegalValueException e) {
            log.error(e.getMessage());
        }
    }
}
