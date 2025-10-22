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

package org.eclipse.mosaic.fed.carla.config;

import org.eclipse.mosaic.lib.util.gson.TimeFieldAdapter;

import com.google.gson.annotations.JsonAdapter;

import java.io.Serializable;

/**
 * The CARLA Ambassador configuration class.
 */
public class CarlaConfiguration implements Serializable {

    private static final long serialVersionUID = 1479294781446446539L;

    /**
     * The Interval after which vehicle info is updated. Define the size of one
     * simulation step in carla (minimal value: 100). The default value is 1000
     * (1s). Unit: [ms].
     */
    @JsonAdapter(TimeFieldAdapter.LegacyMilliSeconds.class)
    public Long updateInterval = 1000L;

    /**
     * CARLA executable path
     */
    public String carlaUE4Path;

    /**
     * path to connection bridge
     */

    /**
     * Carla connection port
     */
    public int carlaConnectionPort;
    /**
     * URL where CARLACDASimAdapter XMLRPC Server is hosted
     */
    public String carlaSensorLibRPCUrl;
    /**
     * URL where CARLACDASimAdapter XMLRPC Server is hosted
     */
    public String carlaActorLibRPCUrl;

    /**
     * Default CARLA vehicle blueprint to spawn for SUMO vehicles.
     * Example: "vehicle.tesla.model3".
     */
    public String defaultVehicleBlueprint = "vehicle.tesla.model3";

    /**
     * When running on Linux, use the direct CARLA binary instead of the shell launcher.
     * This avoids chmod attempts inside the launcher script on some filesystems.
     */
    public Boolean useDirectBinary = Boolean.FALSE;

    /**
     * Directory where SUMO .net.xml located
     */
    public String sumoNetXmlPath;

    /**
     * CARLA map name to load. If null, uses default map.
     * Examples: "Town01", "Town02", "Town03", "Town04", "Town05", "Town10HD"
     */
    public String mapName;

    /**
     * Whether to automatically load the specified map on initialization
     */
    public Boolean autoLoadMap = Boolean.TRUE;


}
