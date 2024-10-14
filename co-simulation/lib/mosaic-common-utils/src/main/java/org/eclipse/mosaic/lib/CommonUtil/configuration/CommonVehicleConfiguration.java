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
package org.eclipse.mosaic.lib.CommonUtil.configuration;

import java.util.List;

import org.eclipse.mosaic.lib.geo.CartesianPoint;
import org.eclipse.mosaic.lib.geo.GeoPoint;

public class CommonVehicleConfiguration {
    /**
     * The route ID on which the vehicle will be spawned.
     */
    public String routeID;

    /**
     * The lane on which the vehicle will be spawned.
     */
    public int lane;

    /**
     * Position within the route where the vehicle should be spawned.
     */
    public double position;

    /**
     * The speed at which the vehicle is supposed to depart.
     */
    public double departSpeed;

    /**
     * The vehicle type
     */
    public String vehicleType;

    /**
     * Specify the applications to be used for this vehicle.
     */
    public List<String> applications;

    /**
     * The geo position at which the vehicle is currently located.
     */
    public GeoPoint geoPosition;

    /**
     * The projected position at which currently the vehicle is located.
     */
    public CartesianPoint projectedPosition;

    /**
     * Vehicle heading in degrees
     */
    public Double heading;

    /**
     * The slope of vehicle in degrees
     */
    public double slope;
}
