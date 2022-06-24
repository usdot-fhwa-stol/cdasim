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

package org.eclipse.mosaic.fed.sumo.traci;

import static org.eclipse.mosaic.fed.sumo.config.CSumo.SUBSCRIPTION_EMISSIONS;
import static org.eclipse.mosaic.fed.sumo.config.CSumo.SUBSCRIPTION_LEADER;
import static org.eclipse.mosaic.fed.sumo.config.CSumo.SUBSCRIPTION_ROAD_POSITION;
import static org.eclipse.mosaic.fed.sumo.config.CSumo.SUBSCRIPTION_SIGNALS;
import static org.hamcrest.CoreMatchers.is;
import static org.hamcrest.MatcherAssert.assertThat;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import org.eclipse.mosaic.fed.sumo.config.CSumo;
import org.eclipse.mosaic.fed.sumo.traci.complex.SumoLaneChangeMode;
import org.eclipse.mosaic.fed.sumo.traci.complex.SumoSpeedMode;
import org.eclipse.mosaic.fed.sumo.traci.complex.TraciSimulationStepResult;
import org.eclipse.mosaic.fed.sumo.traci.junit.SinceTraci;
import org.eclipse.mosaic.fed.sumo.traci.junit.SumoRunner;
import org.eclipse.mosaic.fed.sumo.traci.junit.SumoTraciRule;
import org.eclipse.mosaic.interactions.traffic.VehicleUpdates;
import org.eclipse.mosaic.lib.enums.LaneChangeMode;
import org.eclipse.mosaic.lib.enums.SpeedMode;
import org.eclipse.mosaic.lib.geo.GeoPoint;
import org.eclipse.mosaic.lib.geo.UtmPoint;
import org.eclipse.mosaic.lib.geo.UtmZone;
import org.eclipse.mosaic.lib.junit.GeoProjectionRule;
import org.eclipse.mosaic.lib.objects.traffic.InductionLoopInfo;
import org.eclipse.mosaic.lib.objects.trafficlight.TrafficLightGroup;
import org.eclipse.mosaic.lib.objects.vehicle.VehicleData;
import org.eclipse.mosaic.lib.objects.vehicle.sensor.SensorValue.SensorStatus;
import org.eclipse.mosaic.rti.TIME;
import org.eclipse.mosaic.rti.api.InternalFederateException;

import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import org.apache.commons.io.FileUtils;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.runner.RunWith;

import java.io.File;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;

@RunWith(SumoRunner.class)
public class TraciTest {

    private final File scenarioConfig = FileUtils.toFile(getClass().getResource("/sumo-test-scenario/scenario.sumocfg"));

    private final CSumo sumoConfig = new CSumo();

    private final static double MAX_SPEED = 30d;
    private final static double REACTION_TIME = 1.4d;
    private final static double IMPERFECTION = 0.5d;
    private final static double MAX_ACCEL = 2.4d;
    private final static double MAX_DECEL = 3.4d;
    private final static double SPEED_FACTOR = 1.2d;
    private final static SumoLaneChangeMode LANE_CHANGE_MODE = SumoLaneChangeMode.translateFromEnum(LaneChangeMode.AGGRESSIVE);
    private final static SumoSpeedMode SPEED_MODE = SumoSpeedMode.translateFromEnum(SpeedMode.AGGRESSIVE);

    @Rule
    public final SumoTraciRule traciRule = new SumoTraciRule(scenarioConfig, sumoConfig);

    @Rule
    public GeoProjectionRule coordinateTransformationRule = new GeoProjectionRule(
            UtmPoint.eastNorth(UtmZone.from(GeoPoint.lonLat(13.0, 52.0)), -385281.94, 5817994.50)
    );

    @Before
    public void setup() {
        sumoConfig.subscriptions =
                Lists.newArrayList(SUBSCRIPTION_ROAD_POSITION, SUBSCRIPTION_SIGNALS, SUBSCRIPTION_EMISSIONS, SUBSCRIPTION_LEADER);
    }

    @Test
    public void addVehicles() throws Exception {
        final TraciClient traci = traciRule.getTraciClient();

        TraciSimulationStepResult result = traci.getSimulationControl().simulateUntil(TIME.SECOND);
        assertVehicleInSimulation(result.getVehicleUpdates());

        // RUN
        traci.getSimulationControl().addVehicle("veh_0", "1", "PKW", "0", "0", "max");
        traci.getSimulationControl().subscribeForVehicle("veh_0", 2 * TIME.SECOND, 4000 * TIME.SECOND);

        // ASSERT
        result = traci.getSimulationControl().simulateUntil(2 * TIME.SECOND);
        assertVehicleInSimulation(result.getVehicleUpdates(), "veh_0");

        // RUN
        traci.getSimulationControl().addVehicle("veh_1", "0", "PKW", "0", "0", "max");
        traci.getSimulationControl().subscribeForVehicle("veh_1", 3 * TIME.SECOND, 4000 * TIME.SECOND);

        // ASSERT
        result = traci.getSimulationControl().simulateUntil(10 * TIME.SECOND);
        assertVehicleInSimulation(result.getVehicleUpdates(), "veh_0", "veh_1");
    }

    @Test
    public void setVehicleParameters() throws Exception {
        final TraciClient traci = traciRule.getTraciClient();
        final String vehicle = "1";

        // RUN + ASSERT Tests if Commands are properly executed
        traci.getVehicleControl().setMaxSpeed(vehicle, MAX_SPEED);
        traci.getVehicleControl().setReactionTime(vehicle, REACTION_TIME);
        traci.getVehicleControl().setImperfection(vehicle, IMPERFECTION);
        traci.getVehicleControl().setMaxAcceleration(vehicle, MAX_ACCEL);
        traci.getVehicleControl().setMaxDeceleration(vehicle, MAX_DECEL);
        traci.getVehicleControl().setSpeedFactor(vehicle, SPEED_FACTOR);
        traci.getVehicleControl().setLaneChangeMode(vehicle, LANE_CHANGE_MODE);
        traci.getVehicleControl().setSpeedMode(vehicle, SPEED_MODE);

    }

    @Test
    public void stopResumeVehicle() throws Exception {
        final TraciClient traci = traciRule.getTraciClient();

        traci.getSimulationControl().addVehicle("veh_0", "1", "PKW", "0", "0", "max");
        traci.getSimulationControl().subscribeForVehicle("veh_0", 0L, 400 * TIME.SECOND);
        traci.getSimulationControl().simulateUntil(10 * TIME.SECOND);

        // RUN (park)
        traci.getVehicleControl().stop("veh_0", "1_1_2", 200, 0, Integer.MAX_VALUE, (byte) 0);
        for (int t = 11; t < 100; t++) {
            traci.getSimulationControl().simulateUntil(t * TIME.SECOND);
        }

        // ASSERT
        VehicleData vehData = traci.getSimulationControl().getLastKnownVehicleData("veh_0");
        assertEquals(200, vehData.getRoadPosition().getOffset(), 2d);
        assertTrue(vehData.isStopped());

        // RUN (resume)
        traci.getVehicleControl().resume("veh_0");
        for (int t = 101; t < 200; t++) {
            traci.getSimulationControl().simulateUntil(t * TIME.SECOND);
        }
        // ASSERT
        vehData = traci.getSimulationControl().getLastKnownVehicleData("veh_0");
        assertTrue(vehData.getSpeed() > 0d);
        assertFalse(vehData.isStopped());
    }

    @Test
    public void setSpeedFactor() throws Exception {
        final TraciClient traci = traciRule.getTraciClient();

        traci.getSimulationControl().addVehicle("veh_0", "1", "PKW", "0", "0", "max");
        traci.getSimulationControl().subscribeForVehicle("veh_0", 0L, 400 * TIME.SECOND);

        // RUN set greater than 1
        traci.getVehicleControl().setSpeedFactor("veh_0", 1.2d);
        traci.getSimulationControl().simulateUntil(15 * TIME.SECOND);

        // ASSERT
        VehicleData vehData = traci.getSimulationControl().getLastKnownVehicleData("veh_0");
        assertEquals("Speed should be greater than road limit (13.3 m/s)", 16, vehData.getSpeed(), 0.5d);

        // RUN set lower than 1
        traci.getVehicleControl().setSpeedFactor("veh_0", 0.8d);
        traci.getSimulationControl().simulateUntil(30 * TIME.SECOND);

        // ASSERT
        vehData = traci.getSimulationControl().getLastKnownVehicleData("veh_0");
        assertEquals("Speed should be lower than road limit (13.3 m/s)", 11, vehData.getSpeed(), 0.5d);
    }

    @Test
    public void slowDown() throws Exception {
        // SETUP
        final TraciClient traci = traciRule.getTraciClient();
        traci.getSimulationControl().addVehicle("veh_0", "1", "PKW", "0", "0", "max");
        traci.getSimulationControl().subscribeForVehicle("veh_0", 0L, 4000 * TIME.SECOND);
        traci.getSimulationControl().simulateUntil(4 * TIME.SECOND); // simulate a few steps to reach a certain speed

        // PRE-ASSERT
        assertEquals(13.5, traci.getSimulationControl().getLastKnownVehicleData("veh_0").getSpeed(), 1d);

        // RUN
        traci.getVehicleControl().slowDown("veh_0", 3d, 4000 /* ms */);

        // ASSERT (by checking if slow down speed is reached after given duration)
        traci.getSimulationControl().simulateUntil(9 * TIME.SECOND);
        assertEquals(3d, traci.getSimulationControl().getLastKnownVehicleData("veh_0").getSpeed(), 0.2d);
    }

    @Test
    public void getTrafficLightGroups() throws Exception {
        // RUN
        final Collection<String> tlgList = traciRule.getTraciClient().getSimulationControl().getTrafficLightGroupIds();

        // ASSERT
        assertEquals(1, tlgList.size());

        final TrafficLightGroup tlg =
                traciRule.getTraciClient().getTrafficLightControl().getTrafficLightGroup(Iterables.getOnlyElement(tlgList));
        assertNotNull(tlg);

        assertEquals("2", tlg.getGroupId());
        assertEquals(11, tlg.getTrafficLights().size());
    }

    @Test
    public void getTrafficLightControlledLanes() throws Exception {
        // RUN
        final Collection<String> lanesActual = traciRule.getTraciClient().getTrafficLightControl().getControlledLanes("2");

        // ASSERT
        final Collection<String> lanesExpected =
                Lists.newArrayList(
                        "1_3_2_0", "1_3_2_1", "1_3_2_1",
                        "1_3_2_1", "2_6_3_0", "2_6_3_0", "2_6_3_0",
                        "1_1_2_0", "1_1_2_0", "1_1_2_1", "1_1_2_1"
                );

        assertThat(lanesActual, is(lanesExpected));
    }

    @Test
    public void addRoute() throws Exception {
        final TraciClient traci = traciRule.getTraciClient();

        // RUN
        traci.getRouteControl().addRoute("2", Lists.newArrayList("2_5_2", "1_2_3", "1_3_4"));

        // ASSERT (by adding a vehicle on that route
        traci.getSimulationControl().addVehicle("veh_0", "2", "PKW", "0", "0", "max");
        traci.getSimulationControl().subscribeForVehicle("veh_0", 0L, 4000 * TIME.SECOND);

        // ASSERT (by checking if vehicle is first edge on route)
        final TraciSimulationStepResult result = traci.getSimulationControl().simulateUntil(3 * TIME.SECOND);
        assertVehicleInSimulation(result.getVehicleUpdates(), "veh_0");
        assertEquals("2_5_2", traci.getSimulationControl().getLastKnownVehicleData("veh_0").getRoadPosition().getConnectionId());
    }

    @Test
    public void testFrontSensor_notRestrictedToSubscriptions() throws Exception {
        testFrontSensor(false);
    }

    @Test
    public void testFrontSensor_withSubscriptionsOnly() throws Exception {
        testFrontSensor(true);
    }

    private void testFrontSensor(boolean useSubscriptionsOnly) throws Exception {
        // SETUP
        final TraciClient traci = traciRule.getTraciClient();

        traci.getSimulationControl().simulateUntil(5 * TIME.SECOND);
        traci.getSimulationControl().addVehicle("veh_0", "1", "PKW", "0", "0", "max");
        traci.getSimulationControl().subscribeForVehicle("veh_0", 5 * TIME.SECOND, 4000 * TIME.SECOND);

        // RUN(enable distance sensor)
        if (!useSubscriptionsOnly) {
            traci.getSimulationControl().enableDistanceSensors("veh_0", 200d, true, false);
        }

        TraciSimulationStepResult result = null;
        for (int t = 6; t < 20; t++) {
            result = traci.getSimulationControl().simulateUntil(t * TIME.SECOND);
        }

        // ASSERT (subscribe)
        VehicleData info = Iterables.getFirst(result.getVehicleUpdates().getUpdated(), null);
        assertNotNull(info);
        assertNotNull(info.getVehicleSensors());

        assertTrue(info.getVehicleSensors().distance.front.distValue > 0);
        assertFalse(info.getVehicleSensors().distance.back.distValue > 0);
    }

    @Test
    public void testBackSensor() throws Exception {
        // SETUP
        final TraciClient traci = traciRule.getTraciClient();

        traci.getSimulationControl().simulateUntil(100 * TIME.SECOND);
        traci.getSimulationControl().addVehicle("veh_0", "1", "PKW", "0", "0", "max");
        traci.getSimulationControl().subscribeForVehicle("veh_0", 100 * TIME.SECOND, 4000 * TIME.SECOND);

        // RUN
        traci.getSimulationControl().enableDistanceSensors("veh_0", 200d, false, true);

        TraciSimulationStepResult result = null;
        for (int t = 106; t < 110; t++) {
            result = traci.getSimulationControl().simulateUntil(t * TIME.SECOND);
        }

        // ASSERT back sensor without vehicle behind
        VehicleData info = Iterables.getFirst(result.getVehicleUpdates().getUpdated(), null);
        assertNotNull(info);
        assertNotNull(info.getVehicleSensors());

        assertFalse(info.getVehicleSensors().distance.front.distValue > 0);
        assertEquals(0, info.getVehicleSensors().distance.back.distValue, 0.0);
        assertEquals(SensorStatus.NO_VEHICLE_DETECTED, info.getVehicleSensors().distance.back.status);

        // add new vehicle behind
        traci.getSimulationControl().addVehicle("veh_1", "1", "PKW", "0", "0", "max");
        traci.getSimulationControl().subscribeForVehicle("veh_1", 110 * TIME.SECOND, 4000 * TIME.SECOND);

        for (int t = 110; t < 115; t++) {
            result = traci.getSimulationControl().simulateUntil(t * TIME.SECOND);
        }

        // ASSERT back sensor with vehicle in range
        info = Iterables.getFirst(result.getVehicleUpdates().getUpdated(), null);
        assertNotNull(info);
        assertNotNull(info.getVehicleSensors());

        assertFalse(info.getVehicleSensors().distance.front.distValue > 0);
        assertTrue(info.getVehicleSensors().distance.back.distValue > 0);
        assertEquals(SensorStatus.VEHICLE_DETECTED, info.getVehicleSensors().distance.back.status);
    }

    @Test
    public void testSetColor() throws Exception {
        // SETUP
        final TraciClient traci = traciRule.getTraciClient();

        traci.getSimulationControl().addVehicle("veh_0", "1", "PKW", "0", "0", "max");
        traci.getSimulationControl().subscribeForVehicle("veh_0", 0L, 4000 * TIME.SECOND);

        traci.getSimulationControl().simulateUntil(10 * TIME.SECOND);

        traci.getVehicleControl().setColor("veh_0", 255, 0, 0, 255);

        traci.getSimulationControl().simulateUntil(20 * TIME.SECOND);
        // no assert possible here, visual check required
    }

    @Test
    public void testAccelerationCalculation() throws Exception {
        // SETUP
        final TraciClient traci = traciRule.getTraciClient();

        traci.getSimulationControl().addVehicle("veh_0", "1", "PKW", "0", "0", "max");
        traci.getSimulationControl().subscribeForVehicle("veh_0", 0L, 4000 * TIME.SECOND);

        double prevSpeed = -1;
        VehicleData vehData;
        for (int t = 0; t < 10; t++) {
            traci.getSimulationControl().simulateUntil(t * TIME.SECOND);
            vehData = traci.getSimulationControl().getLastKnownVehicleData("veh_0");

            double expectedAcc = ((vehData.getSpeed() - prevSpeed));
            if (prevSpeed < 0) {
                expectedAcc = 0;
            }
            double actualAcc = vehData.getLongitudinalAcceleration();
            assertEquals(expectedAcc, actualAcc, 0.001d);

            prevSpeed = vehData.getSpeed();
        }
    }

    @SinceTraci(TraciVersion.API_20)
    @Test
    public void testEmissionsAndFuelConsumptionCalculation() throws Exception {
        // SETUP
        final TraciClient traci = traciRule.getTraciClient();

        traci.getSimulationControl().addVehicle("veh_0", "1", "PKW", "0", "0", "max");
        traci.getSimulationControl().subscribeForVehicle("veh_0", 0, 4000 * TIME.SECOND);

        // RUN
        VehicleData vehData = null;
        for (int t = 0; t < 200; t++) {
            traci.getSimulationControl().simulateUntil(t * TIME.SECOND);
            vehData = traci.getSimulationControl().getLastKnownVehicleData("veh_0");
        }

        // ASSERT (for emission class HBEFA3/PC_G_EU4: http://sumo.dlr.de/wiki/Models/Emissions/HBEFA3-based)
        double per1kmFactor = 1000 / vehData.getDistanceDriven();
        assertEquals(54.5, vehData.getVehicleConsumptions().getAllConsumptions().getFuel() * per1kmFactor, 1d);
        assertEquals(490, vehData.getVehicleEmissions().getAllEmissions().getCo() * per1kmFactor, 2d);
        assertEquals(126800, vehData.getVehicleEmissions().getAllEmissions().getCo2() * per1kmFactor, 100d);
        assertEquals(4.7, vehData.getVehicleEmissions().getAllEmissions().getHc() * per1kmFactor, 1d);
        assertEquals(43.9, vehData.getVehicleEmissions().getAllEmissions().getNox() * per1kmFactor, 1d);
        assertEquals(1.23, vehData.getVehicleEmissions().getAllEmissions().getPmx() * per1kmFactor, 0.1d);
    }

    /**
     * Simulates SUMO scenario and checks for vehicles when they should be departed.
     *
     * @throws InternalFederateException e.g. if there was a problem with TraCI connection
     */
    @Test
    public void testGetDepartedVehicles() throws InternalFederateException {
        // SETUP
        final TraciClient traci = traciRule.getTraciClient();

        // RUN & ASSERT
        traci.getSimulationControl().simulateUntil(0L);
        assertTrue(traci.getSimulationControl().getDepartedVehicles().isEmpty());

        traci.getSimulationControl().simulateUntil(TIME.SECOND);
        assertTrue(traci.getSimulationControl().getDepartedVehicles().isEmpty());

        traci.getSimulationControl().simulateUntil(2 * TIME.SECOND);
        assertTrue(traci.getSimulationControl().getDepartedVehicles().isEmpty());

        List<String> departedVehicles;
        traci.getSimulationControl().simulateUntil(3 * TIME.SECOND);
        departedVehicles = traci.getSimulationControl().getDepartedVehicles();
        assertEquals(1, departedVehicles.size());
        assertEquals(TraciClient.VEHICLE_ID_TRANSFORMER.fromExternalId("1"), departedVehicles.get(0));

        traci.getSimulationControl().simulateUntil(4 * TIME.SECOND);
        assertTrue(traci.getSimulationControl().getDepartedVehicles().isEmpty());

        traci.getSimulationControl().simulateUntil(7 * TIME.SECOND);
        departedVehicles = traci.getSimulationControl().getDepartedVehicles();
        assertEquals(1, departedVehicles.size());
        assertEquals(TraciClient.VEHICLE_ID_TRANSFORMER.fromExternalId("0"), departedVehicles.get(0));

        traci.getSimulationControl().simulateUntil(8 * TIME.SECOND);
        assertTrue(traci.getSimulationControl().getDepartedVehicles().isEmpty());
    }

    @Test
    public void testGetRouteEdges() throws Exception {
        // SETUP
        final TraciClient traci = traciRule.getTraciClient();

        List<String> edges = traci.getRouteControl().getRouteEdges("1");
        assertEquals(Arrays.asList("1_1_2", "2_2_5", "2_5_6", "2_6_3", "1_3_4"), edges);
    }

    @Test
    public void testDetectors() throws Exception {
        // SETUP
        final TraciClient traci = traciRule.getTraciClient();
        traci.getSimulationControl().subscribeForInductionLoop("induction_loop_1", 0, 80 * TIME.SECOND);
        traci.getSimulationControl().subscribeForInductionLoop("induction_loop_2", 0, 80 * TIME.SECOND);
        traci.getSimulationControl().subscribeForLaneArea("lane_area_1", 0, 90 * TIME.SECOND);

        // RUN
        int totalCountInductionLoop = 0;

        List<Double> flows = Lists.newArrayList();

        TraciSimulationStepResult simulationStepResult;
        for (int t = 1; t < 80; t++) {
            simulationStepResult = traci.getSimulationControl().simulateUntil(t * TIME.SECOND);
            for (InductionLoopInfo inductionLoopInfo : simulationStepResult.getTrafficDetectorUpdates().getUpdatedInductionLoops()) {
                totalCountInductionLoop += inductionLoopInfo.getVehicleCount();
                if (inductionLoopInfo.getName().equals("induction_loop_1")) {
                    flows.add(inductionLoopInfo.getTrafficFlow());
                }
            }

            if (t > 10) {
                assertEquals(
                        "There should be two vehicle on lane at time " + t,
                        2,
                        Iterables.getOnlyElement(
                                simulationStepResult.getTrafficDetectorUpdates().getUpdatedLaneAreaDetectors()
                        ).getVehicleCount()
                );
            }
        }

        assertEquals(2, totalCountInductionLoop);

        assertEquals(400.0, flows.stream().mapToDouble(Double::doubleValue).max().orElse(0d), 0.5d);
        assertEquals(150.0, flows.stream().mapToDouble(Double::doubleValue).average().orElse(0d), 2d);
    }

    @SinceTraci(TraciVersion.API_20)
    @Test
    public void testLaneSpeedChange() throws Exception {
        // SETUP
        final TraciClient traci = traciRule.getTraciClient();

        traci.getSimulationControl().subscribeForVehicle("1", 10 * TIME.SECOND, 80 * TIME.SECOND);

        double maxSpeedVeh = 0;
        TraciSimulationStepResult simulationStepResult;
        for (int t = 20; t < 40; t++) {
            simulationStepResult = traci.getSimulationControl().simulateUntil(t * TIME.SECOND);
            for (VehicleData info : simulationStepResult.getVehicleUpdates().getUpdated()) {
                maxSpeedVeh = Math.max(maxSpeedVeh, info.getSpeed());
            }
        }
        assertEquals(50 / 3.6d, maxSpeedVeh, 0.5d);

        // RUN
        traci.getSimulationControl().setLaneMaxSpeed("1_1_2_0", 30 / 3.6d);
        traci.getSimulationControl().setLaneMaxSpeed("1_1_2_1", 30 / 3.6d);

        // let the vehicle decelerate
        VehicleData vehicleData =
                Iterables.getOnlyElement(traci.getSimulationControl().simulateUntil(40 * TIME.SECOND).getVehicleUpdates().getUpdated());
        assertEquals(32 / 3.6, vehicleData.getSpeed(), 0.2);

        maxSpeedVeh = 0;
        for (int t = 41; t < 80; t++) {
            simulationStepResult = traci.getSimulationControl().simulateUntil(t * TIME.SECOND);
            for (VehicleData info : simulationStepResult.getVehicleUpdates().getUpdated()) {
                maxSpeedVeh = Math.max(maxSpeedVeh, info.getSpeed());
            }
        }
        assertEquals(30 / 3.6d, maxSpeedVeh, 0.5d);
    }

    private void assertVehicleInSimulation(VehicleUpdates movements, String... vehiclesExpected) {
        String[] vehiclesActual = new String[movements.getAdded().size() + movements.getUpdated().size()];
        int i = 0;
        for (VehicleData vehInfo : movements.getAdded()) {
            vehiclesActual[i++] = vehInfo.getName();
        }
        for (VehicleData vehInfo : movements.getUpdated()) {
            vehiclesActual[i++] = vehInfo.getName();
        }
        Arrays.sort(vehiclesActual);
        Arrays.sort(vehiclesExpected);
        Assert.assertArrayEquals(vehiclesExpected, vehiclesActual);
    }
}
