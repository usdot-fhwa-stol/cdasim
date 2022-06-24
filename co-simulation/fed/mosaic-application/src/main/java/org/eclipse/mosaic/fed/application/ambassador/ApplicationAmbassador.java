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

package org.eclipse.mosaic.fed.application.ambassador;

import org.eclipse.mosaic.fed.application.ambassador.eventresources.RemoveVehicles;
import org.eclipse.mosaic.fed.application.ambassador.navigation.CentralNavigationComponent;
import org.eclipse.mosaic.fed.application.ambassador.simulation.AbstractSimulationUnit;
import org.eclipse.mosaic.fed.application.ambassador.simulation.TrafficLightGroupUnit;
import org.eclipse.mosaic.fed.application.ambassador.simulation.TrafficManagementCenterUnit;
import org.eclipse.mosaic.fed.application.ambassador.simulation.communication.ReceivedV2xMessage;
import org.eclipse.mosaic.fed.application.ambassador.util.EventNicenessPriorityRegister;
import org.eclipse.mosaic.fed.application.app.api.MosaicApplication;
import org.eclipse.mosaic.fed.application.app.api.TrafficSignAwareApplication;
import org.eclipse.mosaic.fed.application.app.api.os.OperatingSystem;
import org.eclipse.mosaic.fed.application.config.CApplicationAmbassador;
import org.eclipse.mosaic.interactions.application.ApplicationInteraction;
import org.eclipse.mosaic.interactions.application.CarlaV2xMessageReception;
import org.eclipse.mosaic.interactions.application.CarmaV2xMessageReception;
import org.eclipse.mosaic.interactions.application.InfrastructureV2xMessageReception;
import org.eclipse.mosaic.interactions.application.ExternalMessage;
import org.eclipse.mosaic.interactions.application.SumoTraciResponse;
import org.eclipse.mosaic.interactions.communication.V2xFullMessageReception;
import org.eclipse.mosaic.interactions.communication.V2xMessageAcknowledgement;
import org.eclipse.mosaic.interactions.communication.V2xMessageReception;
import org.eclipse.mosaic.interactions.electricity.ChargingDenialResponse;
import org.eclipse.mosaic.interactions.electricity.ChargingStationUpdates;
import org.eclipse.mosaic.interactions.electricity.VehicleElectricityUpdates;
import org.eclipse.mosaic.interactions.environment.EnvironmentSensorUpdates;
import org.eclipse.mosaic.interactions.mapping.ChargingStationRegistration;
import org.eclipse.mosaic.interactions.mapping.RsuRegistration;
import org.eclipse.mosaic.interactions.mapping.ServerRegistration;
import org.eclipse.mosaic.interactions.mapping.TmcRegistration;
import org.eclipse.mosaic.interactions.mapping.TrafficLightRegistration;
import org.eclipse.mosaic.interactions.mapping.VehicleRegistration;
import org.eclipse.mosaic.interactions.mapping.advanced.RoutelessVehicleRegistration;
import org.eclipse.mosaic.interactions.traffic.TrafficDetectorUpdates;
import org.eclipse.mosaic.interactions.traffic.TrafficLightUpdates;
import org.eclipse.mosaic.interactions.traffic.VehicleRoutesInitialization;
import org.eclipse.mosaic.interactions.traffic.VehicleTypesInitialization;
import org.eclipse.mosaic.interactions.traffic.VehicleUpdates;
import org.eclipse.mosaic.interactions.trafficsigns.VehicleSeenTrafficSignsUpdate;
import org.eclipse.mosaic.interactions.vehicle.VehicleRouteRegistration;
import org.eclipse.mosaic.lib.objects.environment.EnvironmentEvent;
import org.eclipse.mosaic.lib.objects.traffic.InductionLoopInfo;
import org.eclipse.mosaic.lib.objects.traffic.LaneAreaDetectorInfo;
import org.eclipse.mosaic.lib.objects.trafficlight.TrafficLightGroupInfo;
import org.eclipse.mosaic.lib.objects.v2x.ExternalV2xMessage;
import org.eclipse.mosaic.lib.objects.v2x.V2xMessage;
import org.eclipse.mosaic.lib.objects.v2x.etsi.EtsiPayloadConfiguration;
import org.eclipse.mosaic.lib.objects.vehicle.VehicleBatteryState;
import org.eclipse.mosaic.lib.objects.vehicle.VehicleData;
import org.eclipse.mosaic.lib.objects.vehicle.VehicleDeparture;
import org.eclipse.mosaic.lib.util.FileUtils;
import org.eclipse.mosaic.lib.util.objects.ObjectInstantiation;
import org.eclipse.mosaic.lib.util.scheduling.DefaultEventScheduler;
import org.eclipse.mosaic.lib.util.scheduling.Event;
import org.eclipse.mosaic.lib.util.scheduling.EventManager;
import org.eclipse.mosaic.lib.util.scheduling.EventScheduler;
import org.eclipse.mosaic.lib.util.scheduling.MultiThreadedEventScheduler;
import org.eclipse.mosaic.rti.TIME;
import org.eclipse.mosaic.rti.api.AbstractFederateAmbassador;
import org.eclipse.mosaic.rti.api.IllegalValueException;
import org.eclipse.mosaic.rti.api.Interaction;
import org.eclipse.mosaic.rti.api.InternalFederateException;
import org.eclipse.mosaic.rti.api.parameters.AmbassadorParameter;

import edu.umd.cs.findbugs.annotations.SuppressWarnings;
import org.apache.commons.lang3.Validate;

import java.io.File;
import java.net.MalformedURLException;
import java.net.URL;
import java.net.URLClassLoader;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import javax.annotation.Nonnull;

/**
 * Main class for the application simulator.
 */
@SuppressWarnings("ST_WRITE_TO_STATIC_FROM_INSTANCE_METHOD")
public class ApplicationAmbassador extends AbstractFederateAmbassador implements EventManager {

    private final EventScheduler eventScheduler;

    private final Map<String, VehicleRegistration> vehicleRegistrations = new HashMap<>();

    /**
     * Constructor for {@link ApplicationAmbassador}. This will load the
     * configuration, initialize the {@link SimulationKernel}, initialize the
     * {@link CentralNavigationComponent} and add all Application jar-files.
     *
     * @param ambassadorParameter parameters for ambassador configuration
     */
    public ApplicationAmbassador(AmbassadorParameter ambassadorParameter) {
        super(ambassadorParameter);

        CApplicationAmbassador ambassadorConfig;
        try {
            SimulationKernel.SimulationKernel.setConfigurationPath(ambassadorParameter.configuration.getParentFile());
            // try to read the configuration from the configuration file
            ambassadorConfig = new ObjectInstantiation<>(CApplicationAmbassador.class)
                    .readFile(ambassadorParameter.configuration);

            Validate.isTrue(ambassadorConfig.eventSchedulerThreads > 0,
                    "Number of eventSchedulerThreads must be greater than zero.");

            if (ambassadorConfig.eventSchedulerThreads == 1) {
                eventScheduler = new DefaultEventScheduler();
            } else {
                eventScheduler = new MultiThreadedEventScheduler(ambassadorConfig.eventSchedulerThreads);
            }

            SimulationKernel.SimulationKernel.setConfiguration(ambassadorConfig);
            EtsiPayloadConfiguration.setPayloadConfiguration(new EtsiPayloadConfiguration(
                    ambassadorConfig.encodePayloads, ambassadorConfig.minimalPayloadLength));

        } catch (InstantiationException e) {
            log.error(ErrorRegister.CONFIGURATION_CouldNotReadFromFile.toString(), e);
            throw new RuntimeException(e);
        }
        SimulationKernel.SimulationKernel.setEventManager(this);

        if (SimulationKernel.SimulationKernel.navigation == null) {
            // set the CNC (central navigation component)
            CentralNavigationComponent cnc = new CentralNavigationComponent(ambassadorParameter,
                    ambassadorConfig.navigationConfiguration);
            SimulationKernel.SimulationKernel.setCentralNavigationComponent(cnc);
        }

        // add all application jar files
        addJarFiles();
    }

    private void addJarFiles() {

        File configurationPath = SimulationKernel.SimulationKernel.getConfigurationPath();
        Collection<File> files = FileUtils.searchForFilesOfType(configurationPath, "jar");

        log.debug("load jar files: {}", files);

        List<URL> urls = new ArrayList<>();
        for (File jarFile : files) {
            try {
                urls.add(jarFile.toURI().toURL());
            } catch (MalformedURLException | SecurityException | IllegalArgumentException e) {
                log.error(ErrorRegister.AMBASSADOR_ErrorLoadingJarFiles.toString(), e);
            }
        }
        SimulationKernel.SimulationKernel.setClassLoader(
                new URLClassLoader(urls.toArray(new URL[0]), Thread.currentThread().getContextClassLoader()));
    }

    /**
     * This returns {@code false}, since the {@link ApplicationAmbassador} is
     * developed in a way, where it takes care of its own time management.
     *
     * @return {@code false}
     */
    @Override
    public boolean isTimeConstrained() {
        return false;
    }

    /**
     * This returns {@code false}, since the {@link ApplicationAmbassador} is
     * developed in a way, where it takes care of its own time management.
     *
     * @return {@code false}
     */
    @Override
    public boolean isTimeRegulating() {
        return false;
    }

    @Override
    public void finishSimulation() {
        try {
            SimulationKernel.SimulationKernel.setCurrentSimulationTime(this.rti.getNextEventTimestamp());

            log.debug("remaining events: {}", eventScheduler.getAllEvents());
            UnitSimulator.UnitSimulator.removeAllSimulationUnits();

            log.debug("inform about finishSimulation");
            SimulationKernel.SimulationKernel.finishSimulation();
        } catch (IllegalValueException e) {
            log.error("Could not shutdown applications.", e);
        }
    }

    @Override
    public void initialize(final long startTime, final long endTime) throws InternalFederateException {
        super.initialize(startTime, endTime);
        if (log.isTraceEnabled()) {
            log.trace("subscribedInteractions: {}", Arrays.toString(this.rti.getSubscribedInteractions().toArray()));
        }
        SimulationKernel.SimulationKernel.getCentralNavigationComponent().initialize(this.rti);
        SimulationKernel.SimulationKernel.setInteractable(rti);
        SimulationKernel.SimulationKernel.setRandomNumberGenerator(rti.createRandomNumberGenerator());
    }

    @Override
    protected void processTimeAdvanceGrant(final long time) throws InternalFederateException {
        try {
            if (log.isDebugEnabled()) {
                log.debug("processTimeAdvanceGrant({})", TIME.format(time));
            }
            SimulationKernel.SimulationKernel.setCurrentSimulationTime(time);
            final int scheduled = eventScheduler.scheduleEvents(time);
            log.debug("scheduled {} events at time {}", scheduled, TIME.format(time));
            if (log.isTraceEnabled()) {
                log.trace("scheduled events: {}", scheduled);
            }
        } catch (RuntimeException e) {
            throw new InternalFederateException(ErrorRegister.AMBASSADOR_ErrorAdvanceTime.toString(), e);
        }
    }

    @Override
    protected void processInteraction(final Interaction interaction) throws InternalFederateException {
        if (log.isDebugEnabled()) {
            log.debug("#process with interaction {} at time {} with currentSimulationTime {}", interaction.getTypeId(),
                    TIME.format(interaction.getTime()),
                    TIME.format(SimulationKernel.SimulationKernel.getCurrentSimulationTime()));
        }
        try {
            if (interaction.getTypeId().startsWith(RsuRegistration.TYPE_ID)) {
                this.process((RsuRegistration) interaction);
            } else if (interaction.getTypeId().startsWith(ChargingStationRegistration.TYPE_ID)) {
                this.process((ChargingStationRegistration) interaction);
            } else if (interaction.getTypeId().startsWith(TrafficLightRegistration.TYPE_ID)) {
                this.process((TrafficLightRegistration) interaction);
            } else if (interaction.getTypeId().startsWith(VehicleRegistration.TYPE_ID)) {
                this.process((VehicleRegistration) interaction);
            } else if (interaction.getTypeId().startsWith(RoutelessVehicleRegistration.TYPE_ID)) {
                this.process((RoutelessVehicleRegistration) interaction);
            } else if (interaction.getTypeId().startsWith(TmcRegistration.TYPE_ID)) {
                this.process((TmcRegistration) interaction);
            } else if (interaction.getTypeId().startsWith(ServerRegistration.TYPE_ID)) {
                this.process((ServerRegistration) interaction);
            } else if (interaction.getTypeId().startsWith(ChargingDenialResponse.TYPE_ID)) {
                this.process((ChargingDenialResponse) interaction);
            } else if (interaction.getTypeId().startsWith(ChargingStationUpdates.TYPE_ID)) {
                this.process((ChargingStationUpdates) interaction);
            } else if (interaction.getTypeId().startsWith(VehicleRouteRegistration.TYPE_ID)) {
                this.process((VehicleRouteRegistration) interaction);
            } else if (interaction.getTypeId().startsWith(V2xMessageReception.TYPE_ID)) {
                this.process((V2xMessageReception) interaction);
            } else if (interaction.getTypeId().startsWith(V2xFullMessageReception.TYPE_ID)) {
                this.process((V2xFullMessageReception) interaction);
            } else if (interaction.getTypeId().startsWith(EnvironmentSensorUpdates.TYPE_ID)) {
                this.process((EnvironmentSensorUpdates) interaction);
            } else if (interaction.getTypeId().startsWith(TrafficDetectorUpdates.TYPE_ID)) {
                this.process((TrafficDetectorUpdates) interaction);
            } else if (interaction.getTypeId().startsWith(VehicleSeenTrafficSignsUpdate.TYPE_ID)) {
                this.process((VehicleSeenTrafficSignsUpdate) interaction);
            } else if (interaction.getTypeId().startsWith(SumoTraciResponse.TYPE_ID)) {
                this.process((SumoTraciResponse) interaction);
            } else if (interaction.getTypeId().startsWith(V2xMessageAcknowledgement.TYPE_ID)) {
                this.process((V2xMessageAcknowledgement) interaction);
            } else if (interaction.getTypeId().startsWith(TrafficLightUpdates.TYPE_ID)) {
                this.process((TrafficLightUpdates) interaction);
            } else if (interaction.getTypeId().startsWith(VehicleUpdates.TYPE_ID)) {
                this.process((VehicleUpdates) interaction);
            } else if (interaction.getTypeId().startsWith(VehicleElectricityUpdates.TYPE_ID)) {
                this.process((VehicleElectricityUpdates) interaction);
            } else if (interaction.getTypeId().startsWith(VehicleRoutesInitialization.TYPE_ID)) {
                this.process((VehicleRoutesInitialization) interaction);
            } else if (interaction.getTypeId().startsWith(VehicleTypesInitialization.TYPE_ID)) {
                this.process((VehicleTypesInitialization) interaction);
            } else if (interaction.getTypeId().startsWith(ApplicationInteraction.TYPE_ID)) {
                this.process((ApplicationInteraction) interaction);
            } else if (interaction.getTypeId().startsWith(ExternalMessage.TYPE_ID)) {
                this.process((ExternalMessage) interaction);
            } else {
                log.warn("Unknown interaction received with time {} : {}", TIME.format(interaction.getTime()),
                        interaction.getTypeId());
            }
        } catch (RuntimeException e) {
            throw new InternalFederateException(
                    ErrorRegister.AMBASSADOR_UncaughtExceptionInProcessInteraction.toString(), e);
        }
    }

    private void process(final VehicleElectricityUpdates electricInformationMessage) {
        // schedule all updated vehicles
        for (VehicleBatteryState vehicleBatteryState : electricInformationMessage.getUpdated()) {
            final AbstractSimulationUnit simulationUnit = UnitSimulator.UnitSimulator
                    .getUnitFromId(vehicleBatteryState.getName());
            // we don't simulate vehicles without application
            if (simulationUnit == null) {
                continue;
            }
            final Event event = new Event(electricInformationMessage.getTime(), simulationUnit, vehicleBatteryState,
                    EventNicenessPriorityRegister.batteryUpdated);
            addEvent(event);
        }
    }

    private void process(final VehicleTypesInitialization vehicleTypesInitialization) {
        SimulationKernel.SimulationKernel.getVehicleTypes().putAll(vehicleTypesInitialization.getTypes());
    }

    private void process(final VehicleRoutesInitialization vehicleRoutesInitialization) {
        SimulationKernel.SimulationKernel.getRoutes().putAll(vehicleRoutesInitialization.getRoutes());
    }

    private void process(final VehicleRouteRegistration vehicleRouteRegistration) {
        SimulationKernel.SimulationKernel.getRoutes().put(vehicleRouteRegistration.getRoute().getId(),
                vehicleRouteRegistration.getRoute());
    }

    private void process(final RsuRegistration rsuRegistration) {
        UnitSimulator.UnitSimulator.registerRsu(rsuRegistration);
    }

    private void process(final TmcRegistration tmcRegistration) {
        UnitSimulator.UnitSimulator.registerTmc(tmcRegistration);
    }

    private void process(final ServerRegistration serverRegistration) {
        UnitSimulator.UnitSimulator.registerServer(serverRegistration);
    }

    private void process(final ChargingStationRegistration chargingStationRegistration) {
        UnitSimulator.UnitSimulator.registerChargingStation(chargingStationRegistration);
    }

    private void process(final TrafficLightRegistration trafficLightRegistration) {
        UnitSimulator.UnitSimulator.registerTrafficLight(trafficLightRegistration);
    }

    private void process(final VehicleRegistration vehicleRegistration) {
        vehicleRegistrations.put(vehicleRegistration.getMapping().getName(), vehicleRegistration);
    }

    private void process(final RoutelessVehicleRegistration routelessVehicleRegistration) {
        final VehicleDeparture routeInfo = SimulationKernel.SimulationKernel.getCentralNavigationComponent()
                .createRouteForOdInfo(routelessVehicleRegistration.getTime(), routelessVehicleRegistration.getTrip());
        if (routeInfo == null) {
            log.error(ErrorRegister.AMBASSADOR_ErrorCalculateDeparture.toString());
            return;
        }

        final VehicleRegistration addInteraction = new VehicleRegistration(routelessVehicleRegistration.getTime(),
                routelessVehicleRegistration.getMapping().getName(),
                routelessVehicleRegistration.getMapping().getGroup(),
                routelessVehicleRegistration.getMapping().getApplications(), routeInfo,
                routelessVehicleRegistration.getMapping().getVehicleType());
        log.info("Sending VehicleRegistration Interaction:" + addInteraction);
        try {
            rti.triggerInteraction(addInteraction);
        } catch (InternalFederateException | IllegalValueException e) {
            log.error(ErrorRegister.AMBASSADOR_ErrorSendInteraction.toString(), e);
        }
    }

    private void process(final VehicleSeenTrafficSignsUpdate vehicleSeenTrafficSignsUpdate) {
        for (String vehicleId : vehicleSeenTrafficSignsUpdate.getAllRecipients()) {
            final AbstractSimulationUnit simulationUnit = UnitSimulator.UnitSimulator.getUnitFromId(vehicleId);
            if (simulationUnit == null) {
                log.warn("#process(VehicleSeenTrafficSignsUpdate) and the unitId {} is not available.", vehicleId);
            } else {
                // Call the methods onTrafficSign and onPassedTrafficSign for the respective
                // vehicles/signs in all TrafficSignHandlingApplications.
                for (TrafficSignAwareApplication application : simulationUnit
                        .getApplicationsIterator(TrafficSignAwareApplication.class)) {
                    addEvent(new Event(vehicleSeenTrafficSignsUpdate.getTime(),
                            e -> vehicleSeenTrafficSignsUpdate.getNewSigns(vehicleId)
                                    .forEach(application::onTrafficSignNoticed),
                            EventNicenessPriorityRegister.updateSeenTrafficSign

                    ));
                    addEvent(new Event(vehicleSeenTrafficSignsUpdate.getTime(),
                            e -> vehicleSeenTrafficSignsUpdate.getPassedSigns(vehicleId)
                                    .forEach(application::onTrafficSignInvalidated),
                            EventNicenessPriorityRegister.updateSeenTrafficSign));
                }
            }
        }
    }

    private void process(final ChargingStationUpdates chargingStationUpdates) {
        final AbstractSimulationUnit simulationUnit = UnitSimulator.UnitSimulator
                .getUnitFromId(chargingStationUpdates.getChargingStation().getName());
        // we don't simulate vehicles without an application
        if (simulationUnit == null) {
            return;
        }
        final Event event = new Event(chargingStationUpdates.getChargingStation().getTime(), simulationUnit,
                chargingStationUpdates.getChargingStation(), EventNicenessPriorityRegister.updateChargingStation);
        addEvent(event);
    }

    private void process(final ChargingDenialResponse chargingDenialResponse) {
        final AbstractSimulationUnit simulationUnit = UnitSimulator.UnitSimulator
                .getUnitFromId(chargingDenialResponse.getVehicleId());
        // we don't simulate vehicles without an application
        if (simulationUnit == null) {
            return;
        }
        final Event event = new Event(chargingDenialResponse.getTime(), simulationUnit, chargingDenialResponse,
                EventNicenessPriorityRegister.chargingRejected);
        addEvent(event);
    }

    private void process(final V2xMessageReception v2xMessageReception) {
        final AbstractSimulationUnit simulationUnit = UnitSimulator.UnitSimulator
                .getUnitFromId(v2xMessageReception.getReceiverName());
        // we don't simulate vehicles without an application
        if (simulationUnit == null) {
            return;
        }
        // because the sendV2XMessage method put the v2x message in the map and only
        // send a V2XMessageGeneralized around, unmap the message
        V2xMessage v2XMessage = SimulationKernel.SimulationKernel.getV2xMessageCache()
                .getItem(v2xMessageReception.getMessageId());
        if (v2XMessage == null) {
            log.warn("V2XMessage with id {} is unknown", v2xMessageReception.getMessageId());
            return;
        }

        ReceivedV2xMessage receivedV2xMessage = new ReceivedV2xMessage(v2XMessage,
                v2xMessageReception.getReceiverInformation());
        final Event event = new Event(v2xMessageReception.getTime(), simulationUnit, receivedV2xMessage,
                EventNicenessPriorityRegister.v2xMessageReception);

        addEvent(event);
        // Send received externalV2xMessage to the corresponding ambassador
        if (v2XMessage instanceof ExternalV2xMessage) {
            ExternalV2xMessage externalV2xMessage = (ExternalV2xMessage) v2XMessage;
            if (v2xMessageReception.getReceiverName().startsWith("carma")) {
                // ExternalV2xMessage received by a CARMA vehicle
                CarmaV2xMessageReception carmaV2xMessageReception = new CarmaV2xMessageReception(
                        v2xMessageReception.getTime(), externalV2xMessage.getMessage(),
                        v2xMessageReception.getReceiverName());
                try {
                    rti.triggerInteraction(carmaV2xMessageReception);
                } catch (InternalFederateException | IllegalValueException e) {
                    log.error(ErrorRegister.AMBASSADOR_ErrorSendInteraction.toString(), e);
                }
            } else if (v2xMessageReception.getReceiverName().startsWith("carla")) {
                // ExternalV2xMessage received by a Carla vehicle
                CarlaV2xMessageReception carlaV2xMessageReception = new CarlaV2xMessageReception(
                        v2xMessageReception.getTime(), externalV2xMessage.getMessage(),
                        v2xMessageReception.getReceiverName());
                try {
                    rti.triggerInteraction(carlaV2xMessageReception);
                } catch (InternalFederateException | IllegalValueException e) {
                    log.error(ErrorRegister.AMBASSADOR_ErrorSendInteraction.toString(), e);
                }
            } else if (v2xMessageReception.getReceiverName().startsWith("rsu")) {
                // ExternalV2xMessage received by a rsu
                InfrastructureV2xMessageReception infrastructureV2xMessageReception = new InfrastructureV2xMessageReception(
                        v2xMessageReception.getTime(), externalV2xMessage.getMessage(),
                        v2xMessageReception.getReceiverName());
                try {
                    rti.triggerInteraction(infrastructureV2xMessageReception);
                } catch (InternalFederateException | IllegalValueException e) {
                    log.error(ErrorRegister.AMBASSADOR_ErrorSendInteraction.toString(), e);
                }
            } else {
                log.debug("V2X message is received by SUMO vehicles");
            }
        }
    }

    private void process(final V2xFullMessageReception v2xFullMessageReception) {
        final AbstractSimulationUnit simulationUnit = UnitSimulator.UnitSimulator
                .getUnitFromId(v2xFullMessageReception.getReceiverName());
        // we don't simulate vehicles without an application
        if (simulationUnit == null) {
            return;
        }
        ReceivedV2xMessage receivedV2xMessage = new ReceivedV2xMessage(v2xFullMessageReception.getMessage(),
                v2xFullMessageReception.getReceiverInformation());

        final Event event = new Event(v2xFullMessageReception.getTime(), simulationUnit, receivedV2xMessage,
                EventNicenessPriorityRegister.v2xFullMessageReception);
        addEvent(event);
    }

    /**
     * This function does not directly fire an event, but puts it in a
     * environmentEvents-map (see {@link AbstractSimulationUnit}). Use
     * {@link OperatingSystem#getStateOfEnvironmentSensor} to determine the state of
     * a Sensor. Keep in mind, that the map only stores the latest
     * {@link EnvironmentEvent} of a specific type and overwrites old values.
     * <p>
     * Events will not directly be removed from the map, but since events are mapped
     * to their type, there can't be more members than there are SensorType's.
     * Nonetheless, the map can be cleared using
     * {@link AbstractSimulationUnit#cleanPastEnvironmentEvents()}, which is also
     * invoked by {@link SimulationKernel#garbageCollection()}.
     * </p>
     *
     * @param environmentSensorUpdates the Interaction of type
     *                                 EnvironmentSensorUpdates to be processed
     */
    private void process(final EnvironmentSensorUpdates environmentSensorUpdates) {
        // store the sensor data immediately, the sensor event hold their intermittent
        // time
        final AbstractSimulationUnit simulationUnit = UnitSimulator.UnitSimulator
                .getUnitFromId(environmentSensorUpdates.getUnitId());
        // we don't simulate vehicles without an application
        if (simulationUnit == null) {
            return;
        }
        for (EnvironmentEvent event : environmentSensorUpdates.getEvents()) {
            addEvent(new Event(environmentSensorUpdates.getTime(),
                    e -> simulationUnit.putEnvironmentEvent(event.type, event)));
        }
    }

    private void process(final TrafficDetectorUpdates trafficDetectorUpdates) {
        for (TrafficManagementCenterUnit tmc : UnitSimulator.UnitSimulator.getTmcs().values()) {
            final List<InductionLoopInfo> relevantInductionLoops = trafficDetectorUpdates.getUpdatedInductionLoops()
                    .stream().filter(i -> tmc.getInductionLoopIds().contains(i.getName())).collect(Collectors.toList());

            final List<LaneAreaDetectorInfo> relevantLaneAreaDetectors = trafficDetectorUpdates
                    .getUpdatedLaneAreaDetectors().stream().filter(i -> tmc.getLaneAreaIds().contains(i.getName()))
                    .collect(Collectors.toList());

            if (!relevantInductionLoops.isEmpty() || !relevantLaneAreaDetectors.isEmpty()) {
                // Create new TrafficDetectorUpdates interaction containing only relevant
                // updates
                TrafficDetectorUpdates relevantUpdates = new TrafficDetectorUpdates(trafficDetectorUpdates.getTime(),
                        relevantLaneAreaDetectors, relevantInductionLoops);

                final Event event = new Event(relevantUpdates.getTime(), tmc, relevantUpdates,
                        EventNicenessPriorityRegister.updateTrafficDetectors);
                addEvent(event);
            }
        }
    }

    private void process(final ApplicationInteraction applicationInteraction) {
        if (applicationInteraction.getUnitId() == null) {
            // notify all applications on all units (broadcast)
            for (AbstractSimulationUnit simulationUnit : UnitSimulator.UnitSimulator.getAllUnits().values()) {
                // iterate over all applications on the unit
                for (MosaicApplication application : simulationUnit.getApplicationsIterator(MosaicApplication.class)) {
                    addEvent(new Event(applicationInteraction.getTime(),
                            e -> application.onInteractionReceived(applicationInteraction)));
                }
            }
        } else {
            // notify only a specific unit if available
            final AbstractSimulationUnit simulationUnit = UnitSimulator.UnitSimulator
                    .getUnitFromId(applicationInteraction.getUnitId());
            // we don't simulate this application
            if (simulationUnit == null) {
                log.warn("#process(ApplicationInteraction) and the unitId {} is not available.",
                        applicationInteraction.getUnitId());
            } else {
                for (MosaicApplication application : simulationUnit.getApplicationsIterator(MosaicApplication.class)) {
                    addEvent(new Event(applicationInteraction.getTime(),
                            e -> application.onInteractionReceived(applicationInteraction)));
                }
            }
        }
    }

    /**
     * Inform all simulation units and all their applications immediately about the
     * response. The application should determine by itself, if it needs to handle
     * the response. No event is created, because the information is not time
     * relevant. It also may be possible that an RSU or something else wants to talk
     * with SUMO.
     *
     * @param sumoTraciResponse the Interaction of type SumoTraciResponse to be
     *                          processed
     */
    private void process(final SumoTraciResponse sumoTraciResponse) {
        UnitSimulator.UnitSimulator.processSumoTraciMessage(sumoTraciResponse.getSumoTraciResult());
    }

    private void process(final V2xMessageAcknowledgement v2xMessageAcknowledgement) {
        final AbstractSimulationUnit simulationUnit = UnitSimulator.UnitSimulator
                .getUnitFromId(v2xMessageAcknowledgement.getSourceName());
        // we don't simulate vehicles without an application
        if (simulationUnit == null) {
            log.info("V2X message acknowledgement can not be processed without simulation unit");
            return;
        }
        final Event event = new Event(v2xMessageAcknowledgement.getTime(), simulationUnit, v2xMessageAcknowledgement,
                EventNicenessPriorityRegister.v2xMessageAcknowledgement);
        addEvent(event);
    }

    /**
     * Process the external messages and enable applications can use the external
     * message.
     * 
     * @param externalMessage the Interaction of ExternalMessage to be processed
     */
    private void process(final ExternalMessage externalMessage) {

        final AbstractSimulationUnit simulationUnit = UnitSimulator.UnitSimulator
                .getUnitFromId(externalMessage.getSenderID());
        // we don't simulate vehicles without an application
        if (simulationUnit == null) {
            log.info("External messages can not be processed without simulation unit");
            return;
        }
        log.debug("External message received in application ambassador at time: " + externalMessage.getTime());
        final Event event = new Event(externalMessage.getTime(), simulationUnit, externalMessage,
                EventNicenessPriorityRegister.externalMessage);
        addEvent(event);
        log.debug("Event for external message is triggered.");
    }

    private void process(final TrafficLightUpdates trafficLightUpdates) {
        for (TrafficLightGroupUnit simulationUnit : UnitSimulator.UnitSimulator.getTrafficLights().values()) {
            TrafficLightGroupInfo trafficLightGroupInfo = trafficLightUpdates.getUpdated()
                    .get(simulationUnit.getTrafficLightGroup().getGroupId());

            if (trafficLightGroupInfo != null) {
                final Event event = new Event(trafficLightUpdates.getTime(), simulationUnit, trafficLightGroupInfo,
                        EventNicenessPriorityRegister.updateTrafficLight);
                addEvent(event);
            }
        }

    }

    private void process(final VehicleUpdates vehicleUpdates) {
        // schedule all added vehicles
        for (VehicleData vehicleData : vehicleUpdates.getAdded()) {
            addVehicleIfNotYetAdded(vehicleUpdates.getTime(), vehicleData.getName());
            final AbstractSimulationUnit simulationUnit = UnitSimulator.UnitSimulator
                    .getUnitFromId(vehicleData.getName());
            // we don't simulate vehicles without an application
            if (simulationUnit == null) {
                log.info("Add vehicle will not be processed without simulation unit.");
                continue;
            }
            final Event event = new Event(vehicleData.getTime(), simulationUnit, vehicleData,
                    EventNicenessPriorityRegister.vehicleAdded);
            addEvent(event);
        }

        // schedule all updated vehicles
        for (VehicleData vehicleData : vehicleUpdates.getUpdated()) {
            addVehicleIfNotYetAdded(vehicleUpdates.getTime(), vehicleData.getName());
            final AbstractSimulationUnit simulationUnit = UnitSimulator.UnitSimulator
                    .getUnitFromId(vehicleData.getName());
            // we don't simulate vehicles without an application
            if (simulationUnit == null) {
                log.info("Update vehicle will not be processed without simulation unit.");
                continue;
            }
            final Event event = new Event(vehicleData.getTime(), simulationUnit, vehicleData,
                    EventNicenessPriorityRegister.vehicleUpdated);
            addEvent(event);
        }

        /*
         * Schedule an event to remove vehicles. There is no problem if the event occurs
         * only after the simulation. The unit simulator will cleanly terminate the
         * application.
         */
        final RemoveVehicles removeVehicles = new RemoveVehicles(vehicleUpdates.getRemovedNames());
        final Event event = new Event(vehicleUpdates.getTime(), UnitSimulator.UnitSimulator, removeVehicles,
                EventNicenessPriorityRegister.vehicleRemoved);
        addEvent(event);

        /*
         * Finally, a VehicleUpdate interaction is a good (okay, not good, but we have
         * no other choice) event, to trigger an internal garbage collection.
         */

        final Event triggerGarbageCollection = new Event(vehicleUpdates.getTime(),
                e -> SimulationKernel.SimulationKernel.garbageCollection());
        addEvent(triggerGarbageCollection);
    }

    private void addVehicleIfNotYetAdded(long time, String unitName) {
        final VehicleRegistration vehicleRegistration = vehicleRegistrations.remove(unitName);
        if (vehicleRegistration != null) {
            UnitSimulator.UnitSimulator.registerVehicle(time, vehicleRegistration);
        }
    }

    @Override
    public void addEvent(@Nonnull Event event) {
        eventScheduler.addEvent(event);
        if (log.isDebugEnabled()) {
            log.debug("add event to the scheduler with time {}", TIME.format(event.getTime()));
            if (log.isTraceEnabled()) {
                log.trace("event.resource: {}", event.getResourceClassSimpleName());
                log.trace("event.processors: {}", event.getProcessors());
            }
        }

        try {
            rti.requestAdvanceTime(event.getTime());
            if (log.isDebugEnabled()) {
                log.debug("requestAdvanceTime({})", TIME.format(event.getTime()));
            }
        } catch (IllegalValueException ex) {
            log.error(ErrorRegister.AMBASSADOR_RequestingAdvanceTime.toString(), ex);
            throw new RuntimeException(ErrorRegister.AMBASSADOR_RequestingAdvanceTime.toString(), ex);
        }
    }
}
