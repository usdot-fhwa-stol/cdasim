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

package org.eclipse.mosaic.fed.sumo.traci.facades;

import org.eclipse.mosaic.fed.sumo.config.CSumo;
import org.eclipse.mosaic.fed.sumo.traci.TraciCommandException;
import org.eclipse.mosaic.fed.sumo.traci.TraciConnection;
import org.eclipse.mosaic.fed.sumo.traci.commands.InductionLoopSubscribe;
import org.eclipse.mosaic.fed.sumo.traci.commands.LaneAreaSubscribe;
import org.eclipse.mosaic.fed.sumo.traci.commands.LaneGetLength;
import org.eclipse.mosaic.fed.sumo.traci.commands.LaneSetAllow;
import org.eclipse.mosaic.fed.sumo.traci.commands.LaneSetDisallow;
import org.eclipse.mosaic.fed.sumo.traci.commands.LaneSetMaxSpeed;
import org.eclipse.mosaic.fed.sumo.traci.commands.SimulationGetDepartedVehicleIds;
import org.eclipse.mosaic.fed.sumo.traci.commands.SimulationGetTrafficLightIds;
import org.eclipse.mosaic.fed.sumo.traci.commands.SimulationSimulateStep;
import org.eclipse.mosaic.fed.sumo.traci.commands.TrafficLightSubscribe;
import org.eclipse.mosaic.fed.sumo.traci.commands.VehicleAdd;
import org.eclipse.mosaic.fed.sumo.traci.commands.VehicleGetLeader;
import org.eclipse.mosaic.fed.sumo.traci.commands.VehicleSetRemove;
import org.eclipse.mosaic.fed.sumo.traci.commands.VehicleSetUpdateBestLanes;
import org.eclipse.mosaic.fed.sumo.traci.commands.VehicleSubscribe;
import org.eclipse.mosaic.fed.sumo.traci.complex.AbstractSubscriptionResult;
import org.eclipse.mosaic.fed.sumo.traci.complex.InductionLoopSubscriptionResult;
import org.eclipse.mosaic.fed.sumo.traci.complex.LaneAreaSubscriptionResult;
import org.eclipse.mosaic.fed.sumo.traci.complex.LeadingVehicle;
import org.eclipse.mosaic.fed.sumo.traci.complex.TraciSimulationStepResult;
import org.eclipse.mosaic.fed.sumo.traci.complex.TrafficLightSubscriptionResult;
import org.eclipse.mosaic.fed.sumo.traci.complex.VehicleSubscriptionResult;
import org.eclipse.mosaic.fed.sumo.util.InductionLoop;
import org.eclipse.mosaic.fed.sumo.util.TrafficLightStateDecoder;
import org.eclipse.mosaic.interactions.traffic.TrafficDetectorUpdates;
import org.eclipse.mosaic.interactions.traffic.TrafficLightUpdates;
import org.eclipse.mosaic.interactions.traffic.VehicleUpdates;
import org.eclipse.mosaic.interactions.vehicle.VehicleStop;
import org.eclipse.mosaic.lib.enums.DriveDirection;
import org.eclipse.mosaic.lib.objects.road.IRoadPosition;
import org.eclipse.mosaic.lib.objects.road.SimpleRoadPosition;
import org.eclipse.mosaic.lib.objects.traffic.InductionLoopInfo;
import org.eclipse.mosaic.lib.objects.traffic.LaneAreaDetectorInfo;
import org.eclipse.mosaic.lib.objects.trafficlight.TrafficLightGroupInfo;
import org.eclipse.mosaic.lib.objects.trafficlight.TrafficLightState;
import org.eclipse.mosaic.lib.objects.vehicle.Consumptions;
import org.eclipse.mosaic.lib.objects.vehicle.Emissions;
import org.eclipse.mosaic.lib.objects.vehicle.VehicleConsumptions;
import org.eclipse.mosaic.lib.objects.vehicle.VehicleData;
import org.eclipse.mosaic.lib.objects.vehicle.VehicleEmissions;
import org.eclipse.mosaic.lib.objects.vehicle.VehicleSensors;
import org.eclipse.mosaic.lib.objects.vehicle.VehicleSignals;
import org.eclipse.mosaic.lib.objects.vehicle.sensor.DistanceSensor;
import org.eclipse.mosaic.lib.objects.vehicle.sensor.RadarSensor;
import org.eclipse.mosaic.rti.TIME;
import org.eclipse.mosaic.rti.api.InternalFederateException;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;

public class TraciSimulationFacade {

    private final Logger log = LoggerFactory.getLogger(this.getClass());

    private final TraciConnection traciConnection;
    private final CSumo sumoConfiguration;

    private final Set<String> knownVehicles = new HashSet<>();
    private final Map<String, VehicleData> lastVehicleData = new HashMap<>();
    private final Map<String, Double> vehiclesWithFrontSensor = new HashMap<>();
    private final Map<String, Double> vehiclesWithBackSensor = new HashMap<>();
    private final Map<String, InductionLoop> inductionLoops = new HashMap<>();

    private final SimulationSimulateStep simulateStep;
    private final SimulationGetDepartedVehicleIds getDepartedVehicleIds;
    private final SimulationGetTrafficLightIds getTrafficLightIds;
    private final VehicleAdd vehicleAdd;
    private final VehicleSetRemove remove;
    private final VehicleGetLeader vehicleGetLeader;

    private final VehicleSubscribe vehicleSubscribe;
    private final InductionLoopSubscribe inductionloopSubscribe;
    private final LaneAreaSubscribe laneAreaSubscribe;
    private final TrafficLightSubscribe trafficLightSubscribe;

    private final LaneSetAllow laneSetAllow;
    private final LaneSetDisallow laneSetDisallow;
    private final LaneSetMaxSpeed laneSetMaxSpeed;

    private final LaneGetLength laneGetLength;

    /**
     * Whenever lane assignments are changed, the vehicles need to recalculate
     * their "best lanes" for strategic lane-change decisions. If this variable
     * is set to true, the "best lanes" are recalculated for each vehicle right
     * before the next simulation step.
     */
    private boolean updateBestLanesBeforeNextSimulationStep = false;

    /**
     * Creates a new {@link TraciSimulationFacade} object.
     *
     * @param traciConnection   Connection to Traci.
     * @param sumoConfiguration The SUMO configuration file.
     */
    public TraciSimulationFacade(final TraciConnection traciConnection, final CSumo sumoConfiguration) {
        this.traciConnection = traciConnection;
        this.sumoConfiguration = sumoConfiguration;

        this.simulateStep = traciConnection.getCommandRegister().getOrCreate(SimulationSimulateStep.class);
        this.getDepartedVehicleIds = traciConnection.getCommandRegister().getOrCreate(SimulationGetDepartedVehicleIds.class);
        this.getTrafficLightIds = traciConnection.getCommandRegister().getOrCreate(SimulationGetTrafficLightIds.class);
        this.vehicleAdd = traciConnection.getCommandRegister().getOrCreate(VehicleAdd.class);
        this.remove = traciConnection.getCommandRegister().getOrCreate(VehicleSetRemove.class);
        this.vehicleGetLeader = traciConnection.getCommandRegister().getOrCreate(VehicleGetLeader.class);

        this.inductionloopSubscribe = traciConnection.getCommandRegister().getOrCreate(InductionLoopSubscribe.class);
        this.laneAreaSubscribe = traciConnection.getCommandRegister().getOrCreate(LaneAreaSubscribe.class);
        this.vehicleSubscribe = traciConnection.getCommandRegister().register(
                VehicleSubscribe.class,
                new VehicleSubscribe(traciConnection, sumoConfiguration)
        );
        this.trafficLightSubscribe = traciConnection.getCommandRegister().getOrCreate(TrafficLightSubscribe.class);

        this.laneSetAllow = traciConnection.getCommandRegister().getOrCreate(LaneSetAllow.class);
        this.laneSetDisallow = traciConnection.getCommandRegister().getOrCreate(LaneSetDisallow.class);
        this.laneSetMaxSpeed = traciConnection.getCommandRegister().getOrCreate(LaneSetMaxSpeed.class);

        this.laneGetLength = traciConnection.getCommandRegister().getOrCreate(LaneGetLength.class);
    }

    /**
     * Returns a list of all vehicle ids which departed in the previous time step.
     *
     * @return a list of vehicle ids.
     * @throws InternalFederateException if departed vehicles couldn't be retrieved
     */
    public final List<String> getDepartedVehicles() throws InternalFederateException {
        try {
            return getDepartedVehicleIds.execute(traciConnection);
        } catch (TraciCommandException e) {
            throw new InternalFederateException("Could not retrieve departed vehicles", e);
        }
    }

    /**
     * Returns a list of ids of all traffic light groups existing in the simulation scenario.
     *
     * @return a list of ids of all traffic light groups.
     * @throws InternalFederateException if traffic light groups couldn't be read
     */
    public List<String> getTrafficLightGroupIds() throws InternalFederateException {
        try {
            return getTrafficLightIds.execute(traciConnection);
        } catch (TraciCommandException e) {
            throw new InternalFederateException("Could not read traffic light groups");
        }
    }

    /**
     * Adds an vehicle to the simulation.
     *
     * @param vehicleId   the id of the vehicle. Must not be assigned to another vehicle
     * @param vehicleType the vehicle type. Must be existing
     * @param routeId     the id of the route for the vehicle. Must be existing
     * @param laneId      the lane the vehicle should be inserted on
     * @param departPos   the position along the starting edge the vehicle should be inserted on
     * @param departSpeed the speed at which the vehicle should depart.
     * @throws InternalFederateException if the vehicle couldn't be added
     */
    public void addVehicle(String vehicleId, String routeId, String vehicleType,
                           String laneId, String departPos, String departSpeed) throws InternalFederateException {
        try {
            vehicleAdd.execute(traciConnection, vehicleId, routeId, vehicleType, laneId, departPos, departSpeed);
        } catch (TraciCommandException e) {
            throw new InternalFederateException(String.format("Could not add vehicle %s", vehicleId), e);
        }
    }

    /**
     * Subscribes for the given vehicle. It will then be included in the VehicleUpdates result of {@link #simulateStep}.
     *
     * @param vehicleId the id of the vehicle. Must be known to the simulation
     * @param start     the time [ns] the subscription should start
     * @param end       the time [ns] the subscription should end
     * @throws InternalFederateException if it wasn't possible to subscribe for the wanted vehicle
     */
    public void subscribeForVehicle(String vehicleId, long start, long end) throws InternalFederateException {
        try {
            vehicleSubscribe.execute(traciConnection, vehicleId, start, end);
        } catch (TraciCommandException e) {
            throw new InternalFederateException(String.format("Could not subscribe for vehicle %s", vehicleId), e);
        }
    }

    /**
     * Subscribes to the given induction loop. It will then be included in the InductionLoopData result of {@link #simulateStep}.
     *
     * @param inductionLoopId the id of the induction loop. Must be known to the simulation
     * @param start           the time [ns] the subscription should start
     * @param end             the time [ns] the subscription should end
     * @throws InternalFederateException if it wasn't possible to subscribe for the wanted induction loop
     */
    public final void subscribeForInductionLoop(String inductionLoopId, long start, long end) throws InternalFederateException {
        try {
            inductionloopSubscribe.execute(traciConnection, inductionLoopId, start, end);
        } catch (TraciCommandException e) {
            throw new InternalFederateException(String.format("Could not subscribe for induction loop %s", inductionLoopId), e);
        }
    }

    /**
     * Subscribes to the given lane area. It will then be included in the LANEAREAData result of {@link #simulateStep}.
     *
     * @param laneAreaId the laneId of the lane area. Must be known to the simulation
     * @param start      the time [ns] the subscription should start
     * @param end        the time [ns] the subscription should end
     * @throws InternalFederateException if it wasn't possible to subscribe for the wanted lane area
     */
    public final void subscribeForLaneArea(String laneAreaId, long start, long end) throws InternalFederateException {
        try {
            laneAreaSubscribe.execute(traciConnection, laneAreaId, start, end);
        } catch (TraciCommandException e) {
            throw new InternalFederateException(String.format("Could not subscribe for lane area %s", laneAreaId), e);
        }
    }

    /**
     * Subscribes for the given traffic light group. It will then be included in the list
     * of TrafficLightUpdate result of {@link #simulateStep}.
     *
     * @param trafficLightGroupId the id of a traffic light group. Must be known to the simulation
     * @param start               the time [ns] the subscription should start
     * @param end                 the time [ns] the subscription should end
     * @throws InternalFederateException if it wasn't possible to subscribe for the wanted traffic light group
     */
    public void subscribeForTrafficLight(String trafficLightGroupId, long start, long end) throws InternalFederateException {
        try {
            trafficLightSubscribe.execute(traciConnection, trafficLightGroupId, start, end);
        } catch (TraciCommandException e) {
            throw new InternalFederateException(String.format("Could not subscribe for traffic light group %s", trafficLightGroupId), e);
        }
    }

    /**
     * Changes the set of allowed vehicles of the chosen Lane.
     *
     * @param laneId          the id of the lane. Must be known to the simulation
     * @param allowedVClasses classes for which the lane should be opened
     * @throws InternalFederateException if changing the set of allowed vehicles for the chosen lane couldn't be done
     */
    public final void setLaneAllowedVehicles(String laneId, List<String> allowedVClasses) throws InternalFederateException {
        try {
            laneSetAllow.execute(traciConnection, laneId, allowedVClasses);
            updateBestLanesBeforeNextSimulationStep = true;
        } catch (TraciCommandException e) {
            throw new InternalFederateException(String.format("Could not change the set of allowed vehicle classes of lane %s", laneId), e);
        }
    }

    /**
     * Changes the set of disallowed vehicles of the chosen Lane.
     *
     * @param laneId             the id of the lane. Must be known to the simulation
     * @param disallowedVClasses classes for which the lane should be closed
     * @throws InternalFederateException if changing the set of disallowed vehicles for the chosen lane couldn't be done
     */
    public final void setLaneDisallowedVehicles(String laneId, List<String> disallowedVClasses) throws InternalFederateException {
        try {
            laneSetDisallow.execute(traciConnection, laneId, disallowedVClasses);
            updateBestLanesBeforeNextSimulationStep = true;
        } catch (TraciCommandException exception) {
            throw new InternalFederateException(
                    String.format("Could not change the set of disallowed vehicle classes of lane %s", laneId),
                    exception
            );
        }
    }

    /**
     * Changes the allowed maximum speed for the chosen Lane.
     *
     * @param laneId   the id of the lane. Must be known to the simulation
     * @param maxSpeed the maximum speed to set in m/s
     * @throws InternalFederateException if changing the allowed maximum speed for the wanted lane wasn't possible
     */
    public final void setLaneMaxSpeed(String laneId, double maxSpeed) throws InternalFederateException {
        try {
            laneSetMaxSpeed.execute(traciConnection, laneId, maxSpeed);
        } catch (TraciCommandException e) {
            throw new InternalFederateException(String.format("Could not change the allowed maximum speed of lane %s", laneId), e);
        }
    }

    /**
     * Gets length of a lane on an edge.
     *
     * @param laneIndex the id of the lane. Must be known to the simulation
     * @throws InternalFederateException if the length of the wanted lane on the wanted edge couldn't be retrieved
     */
    public final double getLengthOfLane(String edgeId, int laneIndex) throws InternalFederateException {
        try {
            return laneGetLength.execute(traciConnection, edgeId, laneIndex);
        } catch (TraciCommandException e) {
            throw new InternalFederateException(String.format("Could not retrieve length of lane %s (lane %d)", edgeId, laneIndex), e);
        }
    }

    /**
     * Enables the calculation of distance sensor values for the given vehicle.
     */
    public void enableDistanceSensors(String vehicleId, double maximumLookahead, boolean front, boolean rear) {
        if (front) {
            vehiclesWithFrontSensor.put(vehicleId, maximumLookahead);
        } else {
            vehiclesWithFrontSensor.remove(vehicleId);
        }
        if (rear) {
            vehiclesWithBackSensor.put(vehicleId, maximumLookahead);
        } else {
            vehiclesWithBackSensor.remove(vehicleId);
        }
    }

    /**
     * Simulates until the given time and returns the movements of all vehicles
     * within this simulation step.
     *
     * @param time Time step.
     * @return result of the simulation step.
     * @throws InternalFederateException if could not properly simulate step and read subscriptions
     */
    public TraciSimulationStepResult simulateUntil(long time) throws InternalFederateException {
        try {
            final Map<String, VehicleData> addedVehicles = new HashMap<>();
            final Map<String, VehicleData> updatedVehicles = new HashMap<>();
            final Set<String> removedVehicles = new HashSet<>(this.knownVehicles);

            updateBestLanesIfNecessary();

            final List<AbstractSubscriptionResult> subscriptions = simulateStep.execute(traciConnection, time);

            final Map<String, VehicleSensorData> vehicleSensorData = calculateBackSensorData(traciConnection, subscriptions);
            final Map<String, String> vehicleSegmentInfo = calculateVehicleSegmentInfo(subscriptions);

            VehicleSubscriptionResult veh;
            VehicleData lastVehicleData;
            VehicleData vehicleData;
            for (AbstractSubscriptionResult subscriptionResult : subscriptions) {
                if (subscriptionResult instanceof VehicleSubscriptionResult) {
                    veh = (VehicleSubscriptionResult) subscriptionResult;
                } else {
                    continue;
                }

                lastVehicleData = this.lastVehicleData.get(veh.id);

                VehicleStop.VehicleStopMode vehicleStopMode = getStopMode(veh.stoppedStateEncoded);
                if (vehicleStopMode == VehicleStop.VehicleStopMode.PARK) {
                    if (lastVehicleData == null) {
                        log.warn("Skip vehicle {} which is inserted into simulation in STOPPED state.", veh.id);
                        continue;
                    }
                    if (!lastVehicleData.isStopped()) {
                        log.info("Vehicle {} has parked at {} (edge: {})", veh.id, veh.position, veh.edgeId);
                    }
                    vehicleData = new VehicleData.Builder(time, lastVehicleData.getName()).copyFrom(lastVehicleData).stopped(true).create();
                } else if (veh.position == null || !veh.position.isValid()) {
                    // if a vehicle has not yet been simulated but loaded by SUMO, the vehicle's position will be invalid. therefore we just continue
                    // however, if it has already been in the simulation (remove(id) returns true), then there seems to be an error with the vehicle and it is marked as removed.
                    if (removedVehicles.remove(veh.id)) {
                        log.warn("vehicle {} has not properly arrived at its destination and will be removed", veh.id);
                    }
                    continue;
                } else {
                    vehicleData = new VehicleData.Builder(time, veh.id)
                            .position(veh.position.getGeographicPosition(), veh.position.getProjectedPosition())
                            .road(getRoadPosition(lastVehicleData, veh))
                            .movement(veh.speed, veh.acceleration, fixDistanceDriven(veh.distanceDriven, lastVehicleData))
                            .orientation(DriveDirection.UNAVAILABLE, veh.heading, veh.slope)
                            .route(veh.routeId)
                            .signals(decodeVehicleSignals(veh.signalsEncoded))
                            .stopped(vehicleStopMode != null)
                            .consumptions(calculateConsumptions(veh, lastVehicleData))
                            .emissions(calculateEmissions(veh, lastVehicleData))
                            .sensors(calculateSensorData(veh.id, veh.leadingVehicle, veh.minGap, vehicleSensorData.get(veh.id)))
                            .laneArea(vehicleSegmentInfo.get(veh.id))
                            .create();
                }

                this.lastVehicleData.put(vehicleData.getName(), vehicleData);

                updateVehicleLists(addedVehicles, updatedVehicles, removedVehicles, vehicleData);
            }

            for (String removeVehicleName : removedVehicles) {
                this.lastVehicleData.remove(removeVehicleName);
                knownVehicles.remove(removeVehicleName);
                log.info("Removed vehicle \"{}\" at simulation time {}ns", removeVehicleName, time);
            }

            final List<InductionLoopInfo> updatedInductionLoops = new ArrayList<>();
            final List<LaneAreaDetectorInfo> updatedLaneAreas = new ArrayList<>();
            final Map<String, TrafficLightGroupInfo> trafficLightGroupInfos = new HashMap<>();

            InductionLoopSubscriptionResult inductionLoop;
            LaneAreaSubscriptionResult laneArea;
            InductionLoopInfo inductionLoopInfo;
            LaneAreaDetectorInfo laneAreaDetectorInfo;
            TrafficLightSubscriptionResult trafficLightSubscriptionResult;

            for (AbstractSubscriptionResult subscriptionResult : subscriptions) {
                if (subscriptionResult instanceof InductionLoopSubscriptionResult) {
                    inductionLoop = (InductionLoopSubscriptionResult) subscriptionResult;
                    int count = (int) inductionLoop.vehiclesOnInductionLoop.stream().filter((s) -> s.leaveTime >= 0).count();
                    inductionLoopInfo = new InductionLoopInfo.Builder(time, inductionLoop.id)
                            .vehicleData(inductionLoop.meanSpeed, inductionLoop.meanVehicleLength)
                            .traffic(count, calculateFlow(time, inductionLoop.id, count))
                            .create();
                    updatedInductionLoops.add(inductionLoopInfo);
                } else if (subscriptionResult instanceof LaneAreaSubscriptionResult) {
                    laneArea = (LaneAreaSubscriptionResult) subscriptionResult;
                    double meanSpeed = calculateMeanSpeedOfLaneAreaDetector(laneArea.vehicles);
                    laneAreaDetectorInfo = new LaneAreaDetectorInfo.Builder(time, laneArea.id)
                            .vehicleData(laneArea.vehicleCount, meanSpeed)
                            .density((laneArea.vehicleCount * 1000d) / laneArea.length)
                            .haltingVehicles(laneArea.haltingVehicles)
                            .length(laneArea.length)
                            .create();
                    updatedLaneAreas.add(laneAreaDetectorInfo);
                } else if (subscriptionResult instanceof TrafficLightSubscriptionResult) {
                    trafficLightSubscriptionResult = (TrafficLightSubscriptionResult) subscriptionResult;

                    String trafficLightGroupId = trafficLightSubscriptionResult.id;
                    String currentProgram = trafficLightSubscriptionResult.currentProgramId;
                    int currentPhaseIndex = trafficLightSubscriptionResult.currentPhaseIndex;
                    long assumedTimeOfNextSwitch = trafficLightSubscriptionResult.assumedNextPhaseSwitchTime;
                    List<TrafficLightState> trafficLightState = TrafficLightStateDecoder.createStateListFromEncodedString(
                            trafficLightSubscriptionResult.currentStateEncoded
                    );

                    TrafficLightGroupInfo trafficLightGroupInfo = new TrafficLightGroupInfo(
                            trafficLightGroupId, currentProgram, currentPhaseIndex, assumedTimeOfNextSwitch, trafficLightState
                    );
                    trafficLightGroupInfos.put(trafficLightGroupId, trafficLightGroupInfo);
                }
            }

            final VehicleUpdates vehicleUpdates = new VehicleUpdates(time,
                    new ArrayList<>(addedVehicles.values()),
                    new ArrayList<>(updatedVehicles.values()),
                    new ArrayList<>(removedVehicles)
            );
            final TrafficDetectorUpdates trafficDetectorUpdates = new TrafficDetectorUpdates(time, updatedLaneAreas, updatedInductionLoops);
            final TrafficLightUpdates trafficLightUpdates = new TrafficLightUpdates(time, trafficLightGroupInfos);

            return new TraciSimulationStepResult(vehicleUpdates, trafficDetectorUpdates, trafficLightUpdates);
        } catch (TraciCommandException e) {
            throw new InternalFederateException("Could not properly simulate step and read subscriptions", e);
        }
    }

    /**
     * Calculates the flow in an introduction loop.
     *
     * @param time            Time step.
     * @param inductionLoopId The Id of the induction loop.
     * @param passedVehicles  Number of passed vehicles.
     * @return The vehicle flow.
     */
    private double calculateFlow(long time, String inductionLoopId, int passedVehicles) {
        InductionLoop inductionLoop = inductionLoops.get(inductionLoopId);
        if (inductionLoop == null) {
            inductionLoop = new InductionLoop(inductionLoopId, sumoConfiguration.trafficFlowMeasurementWindowInS * TIME.SECOND);
            inductionLoops.put(inductionLoopId, inductionLoop);
        }
        if (passedVehicles > 0) {
            inductionLoop.update(time, passedVehicles);
        }
        return inductionLoop.getTrafficFlowVehPerHour(time);
    }

    /**
     * Updates the lanes before next simulation step.
     *
     * @throws TraciCommandException     if the status code of the response is ERROR. The TraCI connection is still available.
     * @throws InternalFederateException if some serious error occurs during writing or reading. The TraCI connection is shut down.
     */
    private void updateBestLanesIfNecessary() throws TraciCommandException, InternalFederateException {
        if (updateBestLanesBeforeNextSimulationStep) {
            VehicleSetUpdateBestLanes updateBestLanes = traciConnection.getCommandRegister().getOrCreate(VehicleSetUpdateBestLanes.class);
            for (VehicleData vehicle : lastVehicleData.values()) {
                updateBestLanes.execute(traciConnection, vehicle.getName());
            }
            updateBestLanesBeforeNextSimulationStep = false;
        }
    }

    /**
     * - Workaround -
     * State July 2018:
     * Generally TraCI is able to provide meanSpeed of a Lane Area Detector (E2),
     * but currently TraCI provides false values.
     *
     * @param vehicleIds Ids of the vehicles running in the simulation.
     * @return The mean speed calculated by the lane area detector.
     */
    private double calculateMeanSpeedOfLaneAreaDetector(List<String> vehicleIds) {
        double sum = 0;
        int count = 0;
        for (String vehicleId : vehicleIds) {
            VehicleData vehInfo = lastVehicleData.get(vehicleId);
            sum += vehInfo != null ? vehInfo.getSpeed() : 0;
            count += vehInfo != null ? 1 : 0;
        }
        return sum / count;
    }

    /**
     * Updates the vehicle Ids running in the simulation.
     *
     * @param added        The added vehicle.
     * @param updated      The updated vehicle.
     * @param removedNames The removed vehicles.
     * @param info         Vehicle data.
     */
    private void updateVehicleLists(Map<String, VehicleData> added, Map<String, VehicleData> updated,
                                    Collection<String> removedNames, VehicleData info) {
        String name = info.getName();

        if (knownVehicles.contains(name)) {
            // updated vehicle
            updated.put(name, info);
        } else {
            // new vehicle
            knownVehicles.add(name);
            added.put(name, info);
        }

        // remove not removed values
        removedNames.remove(name);
    }

    /**
     * Calculates sensor data.
     *
     * @param id                             The Id of the sensor.
     * @param leadingVehicleFromSubscription Information of the leading vehicle.
     * @param minGap                         The minimum gap.
     * @param sensorData                     The at sensor captured data.
     * @return Calculated sensor data.
     */
    private VehicleSensors calculateSensorData(String id, LeadingVehicle leadingVehicleFromSubscription,
                                               double minGap, VehicleSensorData sensorData) {

        if (sensorData == null) {
            final VehicleData leading = leadingVehicleFromSubscription != null
                    ? getLastKnownVehicleData(leadingVehicleFromSubscription.getLeadingVehicleId())
                    : null;
            return new VehicleSensors(new DistanceSensor(
                    leadingVehicleFromSubscription != null
                            ? leadingVehicleFromSubscription.getLeadingVehicleDistance() + minGap
                            : -1, -1d, -1d, -1d),
                    new RadarSensor(leading != null ? leading.getSpeed() : -1d)
            );
        }
        return new VehicleSensors(
                new DistanceSensor(sensorData.frontDistance, sensorData.rearDistance, -1d, -1d),
                new RadarSensor(sensorData.frontSpeed)
        );
    }

    private static class VehicleSensorData {
        private double frontDistance = -1d;
        private double frontSpeed = -1d;
        private double rearDistance = -1d;

        private VehicleSensorData frontDistance(double frontDistance) {
            this.frontDistance = frontDistance;
            return this;
        }

        @SuppressWarnings("UnusedReturnValue")
        private VehicleSensorData frontSpeed(double frontSpeed) {
            this.frontSpeed = frontSpeed;
            return this;
        }

        private VehicleSensorData rearDistance(double rearDistance) {
            this.rearDistance = rearDistance;
            return this;
        }
    }

    /**
     * Maps vehicles to the their current lane segments (on which a vehicle is located).
     *
     * @param subscriptions Subscription data.
     * @return The segment of the vehicle.
     */
    private Map<String, String> calculateVehicleSegmentInfo(List<AbstractSubscriptionResult> subscriptions) {
        Map<String, String> vehicleToSegmentMap = new HashMap<>();
        for (AbstractSubscriptionResult laneAreaDetector : subscriptions) {
            if (!(laneAreaDetector instanceof LaneAreaSubscriptionResult)) {
                continue;
            }
            for (String vehicle : ((LaneAreaSubscriptionResult) laneAreaDetector).vehicles) {
                vehicleToSegmentMap.putIfAbsent(vehicle, laneAreaDetector.id);
            }
        }
        return vehicleToSegmentMap;
    }

    /**
     * Calculates the sensor data from rear.
     *
     * @param traciConnection Connection to Traci.
     * @param subscriptions   Subscription data.
     * @return Calculated sensor data.
     * @throws InternalFederateException if leading vehicle for a vehicle couldn't be read
     */
    private Map<String, VehicleSensorData> calculateBackSensorData(TraciConnection traciConnection, List<AbstractSubscriptionResult> subscriptions) throws InternalFederateException {
        if (vehiclesWithFrontSensor.isEmpty() && vehiclesWithBackSensor.isEmpty()) {
            return new HashMap<>();
        }

        final boolean queryingAllVehiclesRequired = !vehiclesWithBackSensor.isEmpty();

        final Map<String, VehicleSensorData> sensorResult = initSensorData();
        double distance;
        for (AbstractSubscriptionResult veh : subscriptions) {
            if (!(veh instanceof VehicleSubscriptionResult)) {
                continue;
            }
            if (queryingAllVehiclesRequired || vehiclesWithFrontSensor.containsKey(veh.id)) {
                try {
                    LeadingVehicle leader = ((VehicleSubscriptionResult) veh).leadingVehicle != null
                            ? ((VehicleSubscriptionResult) veh).leadingVehicle
                            : vehicleGetLeader.execute(traciConnection, veh.id, Double.MAX_VALUE);

                    if (leader != null && vehiclesWithBackSensor.containsKey(leader.getLeadingVehicleId())) {
                        distance = (leader.getLeadingVehicleDistance() <= vehiclesWithBackSensor.get(leader.getLeadingVehicleId()))
                                ? leader.getLeadingVehicleDistance()
                                : Double.POSITIVE_INFINITY;
                        sensorResult.get(leader.getLeadingVehicleId()).rearDistance(distance);
                    }
                    if (leader != null && vehiclesWithFrontSensor.containsKey(veh.id)) {
                        VehicleData leaderInfo = getLastKnownVehicleData(leader.getLeadingVehicleId());
                        if (leader.getLeadingVehicleDistance() <= vehiclesWithFrontSensor.get(veh.id)) {
                            sensorResult.get(veh.id)
                                    .frontDistance(leader.getLeadingVehicleDistance())
                                    .frontSpeed(leaderInfo != null ? leaderInfo.getSpeed() : -1d);
                        } else {
                            sensorResult.get(veh.id)
                                    .frontDistance(Double.POSITIVE_INFINITY)
                                    .frontSpeed(-1d);
                        }
                    }
                } catch (TraciCommandException e) {
                    log.error("Could not read vehicle leader for vehicle {}", veh);
                }
            }
        }
        return sensorResult;
    }

    /**
     * Initialize the sensor data.
     *
     * @return Initialized sensor data.
     */
    private Map<String, VehicleSensorData> initSensorData() {
        final Map<String, VehicleSensorData> sensorResult = new HashMap<>();
        for (String id : vehiclesWithFrontSensor.keySet()) {
            sensorResult.put(id, new VehicleSensorData().frontDistance(Double.POSITIVE_INFINITY));
        }
        VehicleSensorData vehSensorData;
        for (String id : vehiclesWithBackSensor.keySet()) {
            vehSensorData = sensorResult.get(id);
            if (vehSensorData == null) {
                sensorResult.put(id, new VehicleSensorData().rearDistance(Double.POSITIVE_INFINITY));
            } else {
                vehSensorData.rearDistance(Double.POSITIVE_INFINITY);
            }
        }
        return sensorResult;
    }

    /**
     * Calculates vehicle consumption.
     *
     * @param veh             The result of subscribed vehicle.
     * @param lastVehicleData Last information of the vehicle.
     * @return The vehicle consumption.
     */
    private VehicleConsumptions calculateConsumptions(VehicleSubscriptionResult veh, VehicleData lastVehicleData) {
        final Consumptions currentConsumptions = new Consumptions(
                fixConsumptionValue(veh.fuel),
                fixConsumptionValue(veh.electricity)
        );
        if (lastVehicleData != null && lastVehicleData.getVehicleConsumptions() != null) {
            return new VehicleConsumptions(
                    currentConsumptions,
                    lastVehicleData.getVehicleConsumptions().getAllConsumptions().addConsumptions(currentConsumptions)
            );
        }
        return new VehicleConsumptions(currentConsumptions, currentConsumptions);
    }

    private double fixConsumptionValue(double consumption) {
        return consumption * (sumoConfiguration.updateInterval / 1000d);
    }

    /**
     * Calculates the produced emissions of the vehicle.
     *
     * @param veh             The result of subscribed vehicle.
     * @param lastVehicleData Last information of the vehicle.
     * @return The produced emissions.
     */
    private VehicleEmissions calculateEmissions(VehicleSubscriptionResult veh, VehicleData lastVehicleData) {
        final Emissions currentEmissions = new Emissions(
                fixConsumptionValue(veh.co2),
                fixConsumptionValue(veh.co),
                fixConsumptionValue(veh.hc),
                fixConsumptionValue(veh.pmx),
                fixConsumptionValue(veh.nox));
        if (lastVehicleData != null && lastVehicleData.getVehicleEmissions() != null) {
            return new VehicleEmissions(
                    currentEmissions,
                    lastVehicleData.getVehicleEmissions().getAllEmissions().addEmissions(currentEmissions)
            );
        }
        return new VehicleEmissions(currentEmissions, currentEmissions);
    }

    /**
     * Getter for the {@link IRoadPosition}.
     *
     * @param lastVehicleData Last information of the vehicle.
     * @param veh             The result of subscribed vehicle.
     * @return The position in the form of IRoadPosition.
     */
    private IRoadPosition getRoadPosition(VehicleData lastVehicleData, VehicleSubscriptionResult veh) {
        if (veh.edgeId == null) {
            return null;
        }

        IRoadPosition roadPosition = null;
        if (!veh.edgeId.contains(":")) {
            roadPosition = createRoadPosition(
                    veh.edgeId,
                    veh.laneIndex,
                    veh.lanePosition,
                    veh.lateralLanePosition
            );
        } else if (lastVehicleData != null) {
            roadPosition = lastVehicleData.getRoadPosition();
        }
        return roadPosition;
    }

    /**
     * Fixing the driven distance.
     *
     * @param value           Driven distance.
     * @param lastVehicleData Last vehicle info.
     * @return Fixed driven distance.
     */
    private double fixDistanceDriven(double value, VehicleData lastVehicleData) {
        if (value < 0) {
            return (lastVehicleData != null) ? lastVehicleData.getDistanceDriven() : 0;
        } else {
            return value;
        }
    }

    /**
     * Creates a road position as {@link IRoadPosition}.
     *
     * @param edgeId              The Id of the edge.
     * @param offset              The offset.
     * @param lateralLanePosition The lateral lane position.
     * @return Road position.
     */
    private IRoadPosition createRoadPosition(String edgeId, int laneIndex, double offset, double lateralLanePosition) {
        return new SimpleRoadPosition(edgeId, laneIndex, offset, lateralLanePosition);
    }

    /**
     * Getter for the stop mode (stop, park).
     *
     * @param stoppedStateEncoded Encoded number indicating the stop mode.
     * @return The stop mode.
     */
    private VehicleStop.VehicleStopMode getStopMode(int stoppedStateEncoded) {
        if ((stoppedStateEncoded & 0b0010) > 0) {
            return VehicleStop.VehicleStopMode.PARK;
        }
        if ((stoppedStateEncoded & 0b0001) > 0) {
            return VehicleStop.VehicleStopMode.STOP;
        }
        return null;
    }

    /**
     * This method decodes the vehicle signals.
     *
     * @param signalsEncoded Encoded number indicating the vehicle signals.
     * @return a new {@link VehicleSignals} object
     */
    private VehicleSignals decodeVehicleSignals(int signalsEncoded) {
        boolean blinkerRight = getBit(signalsEncoded, 0);
        boolean blinkerLeft = getBit(signalsEncoded, 1);
        boolean blinkerEmergency = getBit(signalsEncoded, 2);
        boolean brakeLight = getBit(signalsEncoded, 3);
        boolean reverseDrive = getBit(signalsEncoded, 7);
        return new VehicleSignals(
                blinkerLeft, blinkerRight, blinkerEmergency, brakeLight, reverseDrive
        );
    }


    /**
     * Takes the decimal number Function takes the Nth bit (1 to 31).
     *
     * @param decimal the number
     * @param n       the Nth bit
     * @return the value of Nth bit from decimal. <code>true</code> if the bit is set, otherwise <code>false</code>.
     */
    private static boolean getBit(int decimal, int n) {
        // Shifting the 1 for N-1 bits
        int constant = 1 << n;
        // if the bit is set, return true, otherwise false
        return (decimal & constant) > 0;
    }

    /**
     * Getter for the last vehicle data.
     *
     * @param vehicleId The Id of the vehicle.
     * @return The latest known vehicle data.
     */
    public VehicleData getLastKnownVehicleData(String vehicleId) {
        return lastVehicleData.get(vehicleId);
    }

    public Map<String, String> getLastKnownLaneAreaIds() {
        return lastVehicleData.values().stream()
                .filter(veh -> veh.getLaneAreaId() != null)
                .collect(Collectors.toMap(VehicleData::getName, VehicleData::getLaneAreaId));
    }

    /**
     * This method removes vehicles from the simulation.
     *
     * @param vehicleId The Id of the vehicle.
     * @param reason    The reason of remove.
     * @throws InternalFederateException if could not remove the wanted vehicle from the simulation
     */
    public void removeVehicle(String vehicleId, VehicleSetRemove.Reason reason) throws InternalFederateException {
        try {
            remove.execute(traciConnection, vehicleId, reason);
        } catch (TraciCommandException e) {
            log.warn("Could not remove vehicle {}", vehicleId);
        }
    }
}
