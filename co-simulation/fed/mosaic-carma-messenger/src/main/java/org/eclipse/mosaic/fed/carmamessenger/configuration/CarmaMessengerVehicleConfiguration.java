/*
 * Copyright (C) 2023 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */
package org.eclipse.mosaic.fed.carmamessenger.configuration;

import java.util.List;

import org.eclipse.mosaic.lib.geo.CartesianPoint;
import org.eclipse.mosaic.lib.geo.GeoPoint;

public class CarmaMessengerVehicleConfiguration {
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
