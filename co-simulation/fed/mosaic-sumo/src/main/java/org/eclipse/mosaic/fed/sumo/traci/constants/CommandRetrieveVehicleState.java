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

package org.eclipse.mosaic.fed.sumo.traci.constants;

public class CommandRetrieveVehicleState {

    public final static int COMMAND = 0xa4;

    public final static SumoVar VAR_MIN_GAP = SumoVar.var(0x4c);

    public final static SumoVar VAR_SPEED = SumoVar.var(0x40);

    public final static SumoVar VAR_POSITION = SumoVar.var(0x42);

    public final static SumoVar VAR_POSITION_3D = SumoVar.var(0x39);

    public final static SumoVar VAR_ACCELERATION = SumoVar.var(0x72);

    public final static SumoVar VAR_ANGLE = SumoVar.var(0x43);

    public final static SumoVar VAR_TYPE_ID = SumoVar.var(0x4f);

    public final static SumoVar VAR_ROAD_ID = SumoVar.var(0x50);

    public final static SumoVar VAR_LANE_ID = SumoVar.var(0x51);

    public final static SumoVar VAR_LANE_INDEX = SumoVar.var(0x52);

    public final static SumoVar VAR_ROUTE_ID = SumoVar.var(0x53);

    public final static SumoVar VAR_LANE_POSITION = SumoVar.var(0x56);

    public final static SumoVar VAR_LATERAL_LANE_POSITION = SumoVar.var(0xb8);

    public static final SumoVar VAR_LEADER = SumoVar.WithParam.var(0x68, 100d);

    public final static SumoVar VAR_DISTANCE = SumoVar.var(0x84);

    public final static SumoVar VAR_SIGNAL_STATES = SumoVar.var(0x5b);

    public final static SumoVar VAR_EMISSIONS_CO2 = SumoVar.var(0x60);

    public final static SumoVar VAR_EMISSIONS_CO = SumoVar.var(0x61);

    public final static SumoVar VAR_EMISSIONS_HC = SumoVar.var(0x62);

    public final static SumoVar VAR_EMISSIONS_PMX = SumoVar.var(0x63);

    public final static SumoVar VAR_EMISSIONS_NOX = SumoVar.var(0x64);

    public final static SumoVar VAR_EMISSIONS_FUEL = SumoVar.var(0x65);

    public final static SumoVar VAR_EMISSIONS_ELECTRICITY = SumoVar.var(0x71);

    public final static SumoVar VAR_STOP_STATE = SumoVar.var(0xb5);

    public final static SumoVar VAR_SLOPE = SumoVar.var(0x36);
}

