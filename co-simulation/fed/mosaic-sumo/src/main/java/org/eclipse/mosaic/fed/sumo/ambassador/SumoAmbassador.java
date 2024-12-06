/*
 * Copyright (c) 2020 Fraunhofer FOKUS and others. All rights reserved.
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contact: mosaic@fokus.fraunhofer.de
 */

package org.eclipse.mosaic.fed.sumo.ambassador;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

import org.apache.commons.lang3.StringUtils;
import org.eclipse.mosaic.fed.sumo.traci.commands.VehicleGetParameter;
import org.eclipse.mosaic.fed.sumo.util.SumoVehicleClassMapping;
import org.eclipse.mosaic.fed.sumo.util.SumoVehicleTypesWriter;
import org.eclipse.mosaic.interactions.application.CarlaTraciRequest;
import org.eclipse.mosaic.interactions.application.CarlaTraciResponse;
import org.eclipse.mosaic.interactions.application.MsgerRequestTrafficEvent;
import org.eclipse.mosaic.interactions.application.MsgerResponseTrafficEvent;
import org.eclipse.mosaic.interactions.application.SimulationStep;
import org.eclipse.mosaic.interactions.application.SimulationStepResponse;
import org.eclipse.mosaic.interactions.mapping.VehicleRegistration;
import org.eclipse.mosaic.interactions.mapping.advanced.ScenarioVehicleRegistration;
import org.eclipse.mosaic.interactions.traffic.VehicleRoutesInitialization;
import org.eclipse.mosaic.interactions.traffic.VehicleTypesInitialization;
import org.eclipse.mosaic.interactions.traffic.VehicleUpdates;
import org.eclipse.mosaic.interactions.vehicle.VehicleRouteRegistration;
import org.eclipse.mosaic.lib.enums.VehicleClass;
import org.eclipse.mosaic.lib.math.MathUtils;
import org.eclipse.mosaic.lib.objects.mapping.VehicleMapping;
import org.eclipse.mosaic.lib.objects.trafficevent.MsgerTrafficEvent;
import org.eclipse.mosaic.lib.objects.vehicle.VehicleRoute;
import org.eclipse.mosaic.lib.objects.vehicle.VehicleType;
import org.eclipse.mosaic.rti.api.FederateAmbassador;
import org.eclipse.mosaic.rti.api.IllegalValueException;
import org.eclipse.mosaic.rti.api.Interaction;
import org.eclipse.mosaic.rti.api.InternalFederateException;
import org.eclipse.mosaic.rti.api.parameters.AmbassadorParameter;

/**
 * Implementation of a {@link AbstractSumoAmbassador} for the traffic simulator
 * SUMO. It allows to control the progress of the traffic simulation and
 * publishes {@link VehicleUpdates}.
 *
 * @see FederateAmbassador
 * @see VehicleUpdates
 */
public class SumoAmbassador extends AbstractSumoAmbassador {

    /**
     * Caches the {@link VehicleRoutesInitialization}-interaction until the
     * {@link VehicleTypesInitialization}-interaction is received.
     */
    private VehicleRoutesInitialization cachedVehicleRoutesInitialization;

    /**
     * Caches the {@link VehicleTypesInitialization}-interaction if it is received
     * before the {@link VehicleRoutesInitialization}-interaction.
     */
    private VehicleTypesInitialization cachedVehicleTypesInitialization;

    /**
     * Cached {@link VehicleRegistration}-interactions, which will clear vehicles if
     * they can be added and try again next time step if it couldn't be emitted.
     */
    private final List<VehicleRegistration> notYetAddedVehicles = new ArrayList<>();

    /**
     * Cached {@link VehicleRegistration}-interactions, for vehicles, that haven't
     * been subscribed to yet.
     */
    private final List<VehicleRegistration> notYetSubscribedVehicles = new ArrayList<>();

    /**
     * Set containing all vehicles, that have been added using the SUMO route file.
     */
    private final Set<String> vehiclesAddedViaRouteFile = new HashSet<>();

    /**
     * Set containing all vehicles, that have been added from the RTI e.g. using the
     * Mapping file.
     */
    private final Set<String> vehiclesAddedViaRti = new HashSet<>();

    /**
     * Constructor for {@link SumoAmbassador}.
     *
     * @param ambassadorParameter parameters for the ambassador containing its' id
     *                            and configuration
     */
    public SumoAmbassador(AmbassadorParameter ambassadorParameter) {
        super(ambassadorParameter);
    }

    @Override
    public void initialize(long startTime, long endTime) throws InternalFederateException {
        super.initialize(startTime, endTime);
    }

    /**
     * This method processes the interaction.
     *
     * @param interaction The interaction that can be processed.
     * @throws InternalFederateException if an interaction could not be received
     *                                   properly
     */
    @Override
    public synchronized void processInteraction(Interaction interaction) throws InternalFederateException {
        // Init and VehicleRegistration are processed directly...
        if (interaction.getTypeId().equals(VehicleRoutesInitialization.TYPE_ID)) {
            this.receiveInteraction((VehicleRoutesInitialization) interaction);
        } else if (interaction.getTypeId().equals(VehicleTypesInitialization.TYPE_ID)) {
            this.receiveInteraction((VehicleTypesInitialization) interaction);
        } else if (interaction.getTypeId().equals(VehicleRegistration.TYPE_ID)) {
            this.receiveInteraction((VehicleRegistration) interaction);
        } else if (interaction.getTypeId().equals(CarlaTraciRequest.TYPE_ID)) {
            this.receiveInteraction((CarlaTraciRequest) interaction);
        } else if (interaction.getTypeId().equals(SimulationStep.TYPE_ID)) {
            this.receiveInteraction((SimulationStep) interaction);
        } else if (interaction.getTypeId().equals(MsgerRequestTrafficEvent.TYPE_ID)){
            this.receiveInteraction((MsgerRequestTrafficEvent)interaction);
        } else {
            // ... everything else is saved for later
            super.processInteraction(interaction);
        }
    }

    /**
     * This processes all other types of interactions as part of
     * {@link #processTimeAdvanceGrant(long)}.
     *
     * @param interaction The interaction to process.
     * @param time        The time of the processed interaction.
     * @throws InternalFederateException Exception is thrown if the interaction time
     *                                   is not correct.
     */
    @Override
    protected void processInteractionAdvanced(Interaction interaction, long time) throws InternalFederateException {
        // make sure the interaction is not in the future
        if (interaction.getTime() > time) {
            throw new InternalFederateException(
                    "Interaction time lies in the future:" + interaction.getTime() + " current time:" + time);
        }

        if (interaction.getTypeId().equals(VehicleRouteRegistration.TYPE_ID)) {
            receiveInteraction((VehicleRouteRegistration) interaction);
        } else {
            super.processInteractionAdvanced(interaction, time);
        }
    }

    private void receiveInteraction(MsgerRequestTrafficEvent interaction) throws InternalFederateException {
        
        VehicleGetParameter veh = new VehicleGetParameter();
        String trafficEvent  = "";
        Set<String> vehicleData = traci.getSimulationControl().getKnownVehicles();
        log.debug("Vehicles list: {}", vehicleData.toString());
        try {
            log.debug("Vehicle info: {}", interaction.toString());
            if(!vehicleData.contains(interaction.vehicleId())){return;}
            trafficEvent = veh.execute(traci, interaction.vehicleId(), interaction.getParameterName());
        
            if (trafficEvent.equals("")) {
                log.debug("No traffic event found");
                rti.triggerInteraction(new MsgerResponseTrafficEvent(interaction.getTime(), null));
            } else{
                String[] parameters = trafficEvent.split(";");
                float[] floatParameters = new float[parameters.length];
                for (int i =0; i< parameters.length; i++){
                    floatParameters[i] = Float.parseFloat(parameters[i]);
                }
                if (parameters.length != 4){
                    log.error("Received payload error");
                    rti.triggerInteraction(new MsgerResponseTrafficEvent(interaction.getTime(), null));
                }
                else{
                    MsgerTrafficEvent config = new MsgerTrafficEvent(interaction.vehicleId(), floatParameters[0], floatParameters[1], floatParameters[2], floatParameters[3]);
                    log.debug("Response traffic event generated: {}",config.toString());
                    rti.triggerInteraction(new MsgerResponseTrafficEvent(interaction.getTime(), config));
                }
            }
        } catch (Exception e) {
            log.error("Unexpected error: " + e.getMessage());
        }
    }
        

    /**
     * Handles the {@link VehicleRegistration}-registration and adds the vehicle to
     * the current simulation.
     *
     * @param interaction {@link VehicleRegistration} containing the vehicle
     *                    definition.
     */
    private void receiveInteraction(VehicleRegistration interaction) {
        VehicleMapping vehicleMapping = interaction.getMapping();
        String vehicleId = vehicleMapping.getName();
        String logMessage;
        boolean isVehicleAddedViaRti = !vehiclesAddedViaRouteFile.contains(vehicleMapping.getName());
        if (isVehicleAddedViaRti) {
            vehiclesAddedViaRti.add(vehicleMapping.getName());
            notYetAddedVehicles.add(interaction);
            logMessage = "VehicleRegistration from RTI \"{}\" received at simulation time {} ns";
        } else { // still subscribe to vehicles with apps
            logMessage = "VehicleRegistration for SUMO vehicle \"{}\" received at simulation time {} ns";
        }
        log.info(logMessage, vehicleId, interaction.getTime());

        // now prepare vehicles to subscribe to
        boolean subscribeToVehicle = sumoConfig.subscribeToAllVehicles || vehicleMapping.hasApplication();
        if (subscribeToVehicle) {
            notYetSubscribedVehicles.add(interaction);
        } else {
            log.debug("Won't subscribe to vehicle \"{}\".", vehicleId);
        }
    }

    /**
     * This processes a {@link VehicleRouteRegistration} that have been dynamically
     * created.
     *
     * @param interaction Interaction containing information about an added route.
     */
    private void receiveInteraction(VehicleRouteRegistration interaction) throws InternalFederateException {
        VehicleRoute newRoute = interaction.getRoute();
        propagateRouteIfAbsent(newRoute.getId(), newRoute);
    }

    /**
     * Extract data from the {@link VehicleRoutesInitialization} to SUMO.
     *
     * @param interaction interaction containing vehicle departures and pre
     *                    calculated routes for change route requests.
     * @throws InternalFederateException if something goes wrong in
     *                                   startSumoLocal(), initTraci(),
     *                                   completeRoutes() or readRouteFromTraci()
     */
    private void receiveInteraction(VehicleRoutesInitialization interaction) throws InternalFederateException {
        log.debug("Received VehicleRoutesInitialization: {}", interaction.getTime());

        cachedVehicleRoutesInitialization = interaction;
        if (sumoReadyToStart()) {
            sumoStartupProcedure();
        }
    }

    /**
     * Extract data from the {@link VehicleTypesInitialization} and forward to SUMO.
     *
     * @param interaction interaction containing vehicle types
     * @throws InternalFederateException if something goes wrong in
     *                                   startSumoLocal(), initTraci() or
     *                                   completeRoutes()
     */
    private void receiveInteraction(VehicleTypesInitialization interaction) throws InternalFederateException {
        log.debug("Received VehicleTypesInitialization");

        cachedVehicleTypesInitialization = interaction;
        if (sumoReadyToStart()) {
            sumoStartupProcedure();
        }
    }

    private boolean sumoReadyToStart() {
        return descriptor != null && cachedVehicleRoutesInitialization != null
                && cachedVehicleTypesInitialization != null;
    }

    private void sumoStartupProcedure() throws InternalFederateException {
        writeTypesFromRti(cachedVehicleTypesInitialization);
        startSumoLocal();
        initTraci();
        readInitialRoutesFromTraci();
        addInitialRoutes();
    }

    /**
     * Read Routes priorly defined in Sumo route-file to later make them available
     * to the rest of the simulations using {@link VehicleRouteRegistration}.
     *
     * @throws InternalFederateException if Traci connection couldn't be established
     */
    private void readInitialRoutesFromTraci() throws InternalFederateException {
        for (String id : traci.getRouteControl().getRouteIds()) {
            if (!routeCache.containsKey(id)) {
                VehicleRoute route = readRouteFromTraci(id);
                routeCache.put(route.getId(), route);
            }
        }
    }

    /**
     * Passes on initial routes to SUMO.
     *
     * @throws InternalFederateException if there was a problem with traci
     */
    private void addInitialRoutes() throws InternalFederateException {
        for (Map.Entry<String, VehicleRoute> routeEntry : cachedVehicleRoutesInitialization.getRoutes().entrySet()) {
            propagateRouteIfAbsent(routeEntry.getKey(), routeEntry.getValue());
        }
    }

    /**
     * Vehicles of the {@link #notYetSubscribedVehicles} list will be added to
     * simulation by this function or cached again for the next time advance.
     *
     * @param time Current system time
     * @throws InternalFederateException if vehicle couldn't be added
     */
    @Override
    protected void flushNotYetAddedVehicles(long time) throws InternalFederateException {
        // if not yet a last advance time was set, sumo is not ready
        if (time < 0) {
            return;
        }
        log.debug("traci getDepartedVehicles list: {}", traci.getSimulationControl().getDepartedVehicles().toString());
        // first check if there were new vehicles added via SUMO
        propagateSumoVehiclesToRti();
        log.debug("After add vehicle to vehiclesAddedViaRouteFile: {}", vehiclesAddedViaRouteFile.toString());
        // now add all vehicles, that were received from RTI
        addNotYetAddedVehicles(time);
        // now subscribe to all relevant vehicles
        subscribeToNotYetSubscribedVehicles(time);

    }

    private void propagateSumoVehiclesToRti() throws InternalFederateException {
        final List<String> departedVehicles = traci.getSimulationControl().getDepartedVehicles();
        String vehicleTypeId;
        for (String vehicleId : departedVehicles) {
            if (vehiclesAddedViaRti.contains(vehicleId)) { // only handle route file vehicles here
                continue;
            }
            vehiclesAddedViaRouteFile.add(vehicleId);          
            vehicleTypeId = traci.getVehicleControl().getVehicleTypeId(vehicleId);
            try {
                rti.triggerInteraction(new ScenarioVehicleRegistration(this.nextTimeStep, vehicleId, vehicleTypeId));
            } catch (IllegalValueException e) {
                throw new InternalFederateException(e);
            }
            if (sumoConfig.subscribeToAllVehicles) {
                traci.getSimulationControl().subscribeForVehicle(vehicleId, this.nextTimeStep, this.getEndTime());
            }
        }
    }

    private void addNotYetAddedVehicles(long time) throws InternalFederateException {
        for (Iterator<VehicleRegistration> iterator = notYetAddedVehicles.iterator(); iterator.hasNext();) {
            VehicleRegistration interaction = iterator.next();

            String vehicleId = interaction.getMapping().getName();
            String vehicleType = interaction.getMapping().getVehicleType().getName();
            String routeId = interaction.getDeparture().getRouteId();
            String departPos = String.format(Locale.ENGLISH, "%.2f", interaction.getDeparture().getDeparturePos());
            String departSpeed = extractDepartureSpeed(interaction);
            String laneId = extractDepartureLane(interaction);

            try {
                if (interaction.getTime() <= time) {
                    log.info(
                            "Adding new vehicle \"{}\" at simulation time {} ns (type={}, routeId={}, laneId={}, departPos={})",
                            vehicleId, interaction.getTime(), vehicleType, routeId, laneId, departPos);

                    if (!routeCache.containsKey(routeId)) {
                        throw new IllegalArgumentException("Unknown route " + routeId
                                + " for vehicle with departure time " + interaction.getTime());
                    }

                    traci.getSimulationControl().addVehicle(vehicleId, routeId, vehicleType, laneId, departPos,
                            departSpeed);

                    applyChangesInVehicleTypeForVehicle(vehicleId, interaction.getMapping().getVehicleType(),
                            cachedVehicleTypesInitialization.getTypes().get(vehicleType));
                    iterator.remove();
                }
            } catch (InternalFederateException e) {
                log.warn("Vehicle with id: " + vehicleId + " could not be added.(" + e.getClass().getCanonicalName()
                        + ")", e);
                if (sumoConfig.exitOnInsertionError) {
                    throw e;
                }
                iterator.remove();
            }
        }
    }

    private void subscribeToNotYetSubscribedVehicles(long time) throws InternalFederateException {
        for (Iterator<VehicleRegistration> iterator = notYetSubscribedVehicles.iterator(); iterator.hasNext();) {
            VehicleRegistration currentVehicleRegistration = iterator.next();
            String vehicleId = currentVehicleRegistration.getMapping().getName();
            try {
                // always subscribe to vehicles, that are came from SUMO and are in
                // notYetSubscribedVehicles-list
                if (vehiclesAddedViaRouteFile.contains(vehicleId) || currentVehicleRegistration.getTime() <= time) {
                    traci.getSimulationControl().subscribeForVehicle(vehicleId, currentVehicleRegistration.getTime(),
                            this.getEndTime());
                    iterator.remove();
                }
            } catch (InternalFederateException e) {
                log.warn("Couldn't subscribe to vehicle {}.", vehicleId);
                if (sumoConfig.exitOnInsertionError) {
                    throw e;
                }
                iterator.remove();
            }
        }

    }

    private void applyChangesInVehicleTypeForVehicle(String vehicleId, VehicleType actualVehicleType,
            VehicleType baseVehicleType) throws InternalFederateException {
        if (!MathUtils.isFuzzyEqual(actualVehicleType.getTau(), baseVehicleType.getTau())) {
            double minReactionTime = sumoConfig.updateInterval / 1000d;
            traci.getVehicleControl().setReactionTime(vehicleId,
                    Math.max(minReactionTime, actualVehicleType.getTau() + sumoConfig.timeGapOffset));
        }
        if (!MathUtils.isFuzzyEqual(actualVehicleType.getMaxSpeed(), baseVehicleType.getMaxSpeed())) {
            traci.getVehicleControl().setMaxSpeed(vehicleId, actualVehicleType.getMaxSpeed());
        }
        if (!MathUtils.isFuzzyEqual(actualVehicleType.getAccel(), baseVehicleType.getAccel())) {
            traci.getVehicleControl().setMaxAcceleration(vehicleId, actualVehicleType.getAccel());
        }
        if (!MathUtils.isFuzzyEqual(actualVehicleType.getDecel(), baseVehicleType.getDecel())) {
            traci.getVehicleControl().setMaxDeceleration(vehicleId, actualVehicleType.getDecel());
        }
        if (!MathUtils.isFuzzyEqual(actualVehicleType.getMinGap(), baseVehicleType.getMinGap())) {
            traci.getVehicleControl().setMinimumGap(vehicleId, actualVehicleType.getMinGap());
        }
        if (!MathUtils.isFuzzyEqual(actualVehicleType.getLength(), baseVehicleType.getLength())) {
            traci.getVehicleControl().setVehicleLength(vehicleId, actualVehicleType.getLength());
        }
        if (!MathUtils.isFuzzyEqual(actualVehicleType.getSpeedFactor(), baseVehicleType.getSpeedFactor())) {
            traci.getVehicleControl().setSpeedFactor(vehicleId, actualVehicleType.getSpeedFactor());
        }
    }

    private String extractDepartureSpeed(VehicleRegistration interaction) {
        switch (interaction.getDeparture().getDepartSpeedMode()) {
            case PRECISE:
                return String.format(Locale.ENGLISH, "%.2f", interaction.getDeparture().getDepartSpeed());
            case RANDOM:
                return "random";
            case MAXIMUM:
            default:
                return "max";
        }
    }

    private String extractDepartureLane(VehicleRegistration interaction) {
        switch (interaction.getDeparture().getLaneSelectionMode()) {
            case RANDOM:
                return "random";
            case FREE:
                return "free";
            case ALLOWED:
                return "allowed";
            case BEST:
                return "best";
            case FIRST:
                return "first";
            case HIGHWAY:
                return isTruckOrTrailer(interaction.getMapping().getVehicleType().getVehicleClass()) ? "first" : "best";
            default:
                int extractedLaneId = interaction.getDeparture().getDepartureLane();
                return extractedLaneId >= 0 ? Integer.toString(extractedLaneId) : "best";
        }
    }

    private boolean isTruckOrTrailer(VehicleClass vehicleClass) {
        return SumoVehicleClassMapping.toSumo(vehicleClass).equals("truck")
                || SumoVehicleClassMapping.toSumo(vehicleClass).equals("trailer");
    }

    private void propagateRouteIfAbsent(String routeId, VehicleRoute route) throws InternalFederateException {
        // if the route is already known (because it is defined in a route-file) don't
        // add route
        if (routeCache.containsKey(routeId)) {
            log.warn("Couldn't add Route {}, because it is already known to SUMO.", routeId);
        } else {
            routeCache.put(routeId, route);
            traci.getRouteControl().addRoute(routeId, route.getConnectionIds());
        }
    }

    /**
     * Writes a new SUMO additional-file based on the registered vehicle types.
     *
     * @param typesInit Interaction contains predefined vehicle types.
     */
    private void writeTypesFromRti(VehicleTypesInitialization typesInit) {
        File dir = new File(descriptor.getHost().workingDirectory, descriptor.getId());
        String subDir = new File(sumoConfig.sumoConfigurationFile).getParent();
        if (StringUtils.isNotBlank(subDir)) {
            dir = new File(dir, subDir);
        }
        SumoVehicleTypesWriter sumoVehicleTypesWriter = new SumoVehicleTypesWriter(dir, sumoConfig);
        // stores the *.add.xml file to the working directory. this file is required for
        // SUMO to run
        sumoVehicleTypesWriter.addVehicleTypes(typesInit.getTypes()).store();
    }

    /**
     * This processes a {@link CarlarTraciRequest} that have been dynamically
     * created.
     *
     * @param interaction Interaction containing information about carla request.
     */
    private void receiveInteraction(CarlaTraciRequest interaction) throws InternalFederateException {
        try {
            sumoCarlaCoSimulation = true;
            traci.getOut().write(interaction.getCommand());
            byte[] returnedMessage = new byte[65535];
            int len = traci.getIn().read(returnedMessage);
            byte[] messageToCarla = Arrays.copyOfRange(returnedMessage, 0, len);

            // trigger a CarlaTraciResponse interaction
            rti.triggerInteraction(new CarlaTraciResponse(interaction.getTime(), len, messageToCarla));

        } catch (Exception e) {
            log.error("error occurs during process carla request interaction: " + e.getMessage());
        }
    }

    /**
     * This processes a {@link SimulationStep} that have been dynamically created.
     *
     * @param interaction Interaction containing information about traci simulation
     *                    step from carla.
     */
    private void receiveInteraction(SimulationStep interaction) throws InternalFederateException {
        try {
            byte[] simulationStepResponse = new byte[] { 0, 0, 0, 15, 7, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

            log.info("sumo received carla simulation step at time: " + interaction.getTime() + " at sumo time: "
                    + this.nextTimeStep);
            // trigger a simulation step response
            rti.triggerInteraction(new SimulationStepResponse(interaction.getTime(), simulationStepResponse.length,
                    simulationStepResponse));
            // set the simulation step flag
            receivedSimulationStep = true;

        } catch (

        Exception e) {
            log.error("error occurs during process simulation step interaction: " + e.getMessage());
        }
    }
}
