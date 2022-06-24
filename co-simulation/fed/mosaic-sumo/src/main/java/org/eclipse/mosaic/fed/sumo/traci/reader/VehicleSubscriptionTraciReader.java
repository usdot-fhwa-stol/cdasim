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

package org.eclipse.mosaic.fed.sumo.traci.reader;

import org.eclipse.mosaic.fed.sumo.traci.AbstractTraciResultReader;
import org.eclipse.mosaic.fed.sumo.traci.TraciClient;
import org.eclipse.mosaic.fed.sumo.traci.complex.LeadingVehicle;
import org.eclipse.mosaic.fed.sumo.traci.complex.VehicleSubscriptionResult;
import org.eclipse.mosaic.fed.sumo.traci.constants.CommandRetrieveVehicleState;
import org.eclipse.mosaic.lib.util.objects.Position;

import org.apache.commons.lang3.StringUtils;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.DataInputStream;
import java.io.IOException;

public class VehicleSubscriptionTraciReader extends AbstractSubscriptionTraciReader<VehicleSubscriptionResult> {

    private final Logger log = LoggerFactory.getLogger(this.getClass());

    public VehicleSubscriptionTraciReader() {
        getTypeBasedTraciReader().registerCompoundReader(new LeadingVehicleReader());
    }

    @Override
    VehicleSubscriptionResult createSubscriptionResult(String id) {
        VehicleSubscriptionResult result = new VehicleSubscriptionResult();
        result.id = TraciClient.VEHICLE_ID_TRANSFORMER.fromExternalId(id);
        return result;
    }

    /**
     * This method enables to handle the subscription variable of the vehicle.
     *
     * @param result   The result of the vehicle.
     * @param varId    The Id of the variable.
     * @param varValue The value of the variable.
     */
    protected void handleSubscriptionVariable(VehicleSubscriptionResult result, int varId, Object varValue) {
        if (varId == CommandRetrieveVehicleState.VAR_SPEED.var) {
            result.speed = (double) varValue;
        } else if (varId == CommandRetrieveVehicleState.VAR_POSITION.var) {
            result.position = (Position) varValue;
        } else if (varId == CommandRetrieveVehicleState.VAR_POSITION_3D.var) {
            result.position = (Position) varValue;
        } else if (varId == CommandRetrieveVehicleState.VAR_ANGLE.var) {
            result.heading = (double) varValue;
        } else if (varId == CommandRetrieveVehicleState.VAR_ACCELERATION.var) {
            result.acceleration = (double) varValue;
        } else if (varId == CommandRetrieveVehicleState.VAR_ROAD_ID.var) {
            result.edgeId = (String) varValue;
        } else if (varId == CommandRetrieveVehicleState.VAR_LANE_INDEX.var) {
            result.laneIndex = (int) varValue;
        } else if (varId == CommandRetrieveVehicleState.VAR_ROUTE_ID.var) {
            result.routeId = (String) varValue;
        } else if (varId == CommandRetrieveVehicleState.VAR_SIGNAL_STATES.var) {
            result.signalsEncoded = (int) varValue;
        } else if (varId == CommandRetrieveVehicleState.VAR_EMISSIONS_CO2.var) {
            result.co2 = (double) varValue;
        } else if (varId == CommandRetrieveVehicleState.VAR_EMISSIONS_CO.var) {
            result.co = (double) varValue;
        } else if (varId == CommandRetrieveVehicleState.VAR_EMISSIONS_HC.var) {
            result.hc = (double) varValue;
        } else if (varId == CommandRetrieveVehicleState.VAR_EMISSIONS_PMX.var) {
            result.pmx = (double) varValue;
        } else if (varId == CommandRetrieveVehicleState.VAR_EMISSIONS_NOX.var) {
            result.nox = (double) varValue;
        } else if (varId == CommandRetrieveVehicleState.VAR_EMISSIONS_FUEL.var) {
            result.fuel = (double) varValue;
        } else if (varId == CommandRetrieveVehicleState.VAR_EMISSIONS_ELECTRICITY.var) {
            result.electricity = (double) varValue;
        } else if (varId == CommandRetrieveVehicleState.VAR_LANE_POSITION.var) {
            result.lanePosition = (double) varValue;
        } else if (varId == CommandRetrieveVehicleState.VAR_LATERAL_LANE_POSITION.var) {
            result.lateralLanePosition = (double) varValue;
        } else if (varId == CommandRetrieveVehicleState.VAR_DISTANCE.var) {
            result.distanceDriven = (double) varValue;
        } else if (varId == CommandRetrieveVehicleState.VAR_SLOPE.var) {
            result.slope = (double) varValue;
        } else if (varId == CommandRetrieveVehicleState.VAR_STOP_STATE.var) {
            result.stoppedStateEncoded = (int) varValue;
        } else if (varId == CommandRetrieveVehicleState.VAR_LEADER.var) {
            result.leadingVehicle = (LeadingVehicle) varValue;
        } else if (varId == CommandRetrieveVehicleState.VAR_MIN_GAP.var) {
            result.minGap = (double) varValue;
        } else {
            log.warn("Unknown subscription variable {}. Skipping.", String.format("%02X ", varId));
        }
    }

    static class LeadingVehicleReader extends AbstractTraciResultReader<LeadingVehicle> {

        protected LeadingVehicleReader() {
            super(null);
        }

        @Override
        protected LeadingVehicle readFromStream(DataInputStream in) throws IOException {
            readByte(in);
            String leaderId = readString(in);

            readByte(in);
            double leaderDistance = readDouble(in);

            if (StringUtils.isEmpty(leaderId)) {
                return null;
            }
            return new LeadingVehicle(leaderId, leaderDistance);
        }
    }
}
