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

package org.eclipse.mosaic.lib.objects.vehicle;

import java.io.Serializable;
import javax.annotation.concurrent.Immutable;

@Immutable
public class VehicleDeparture implements Serializable {

    private static final long serialVersionUID = 1L;

    public enum DepartSpeedMode {
        /* the vehicle departs with the speed given in the mapping definiton of the spawner */
        PRECISE,
        /* the vehicle departs with a random speed */
        RANDOM,
        /* the vehicle departs with the maximum speed possible, according to the speed limit of the road and its own abilities */
        MAXIMUM
    }

    public enum LaneSelectionMode {
        DEFAULT
        /* General LaneSelectionModes */
        /* selects the lane for the next vehicle based on the list of given lanes in a roundrobin manner*/,
        ROUNDROBIN,
        /* trucks will be spawned on the right most lane, other vehicles on the next of the given lanes in roundrobin fashion */
        ROUNDROBIN_HIGHWAY,

        /* Advanced SUMO LaneSelectionModes */
        /* trucks will be spawned on the right most lane, other vehicles on the best available lane */
        HIGHWAY,

        /* Standard SUMO LaneSelectionModes */
        /* the vehicle will be placed randomly on one of the available lanes of the road */
        RANDOM,
        /* the vehicle will be placed on a free lane of the road */
        FREE,
        /* the vehicle will be placed on a allowed lane of the road */
        ALLOWED,
        /* the vehicle will be placed on the best lane of the road, according to the current traffic situation and departure speed */
        BEST,
        /* the vehicle will be placed on the rightmost lane */
        FIRST
    }

    /**
     * Id of the route.
     */
    private final String routeId;

    /**
     * The specific lane the vehicle will depart on, if no laneSelectionMode is given.
     */
    private final int departureLane;

    /**
     * The departure offset from the beginning of the lane, where the vehicle will be spawned on.
     */
    private final double departurePos;

    /**
     * The speed with which the vehicle will depart, if present space is free.
     */
    private final double departSpeed;

    /**
     * The mode with which the departure speed is chosen depending on the current traffic situation.
     */
    private final DepartSpeedMode departSpeedMode;

    /**
     * The mode with which the departure lane is chosen depending on the current traffic situation.
     */
    private final LaneSelectionMode laneSelectionMode;

    private VehicleDeparture(String routeId, int departureLane, double departurePos,
                             double departSpeed, DepartSpeedMode departSpeedMode,
                             LaneSelectionMode laneSelectionMode) {
        this.routeId = routeId;
        this.departureLane = departureLane;
        this.departurePos = departurePos;
        this.departSpeed = departSpeed;
        this.departSpeedMode = departSpeedMode;
        this.laneSelectionMode = laneSelectionMode;
    }

    public String getRouteId() {
        return routeId;
    }

    public int getDepartureLane() {
        return departureLane;
    }

    public double getDeparturePos() {
        return departurePos;
    }

    public double getDepartSpeed() {
        return departSpeed;
    }

    public DepartSpeedMode getDepartSpeedMode() {
        return departSpeedMode;
    }

    public LaneSelectionMode getLaneSelectionMode() {
        return laneSelectionMode;
    }

    public static class Builder {
        private final String routeId;

        private int departureLane;
        private double departurePos;
        private double departSpeed;
        private DepartSpeedMode departSpeedMode;
        private LaneSelectionMode laneSelectionMode;

        public Builder(String routeId) {
            this.routeId = routeId;
        }

        public Builder departureLane(LaneSelectionMode laneSelectionMode, int lane, double pos) {
            this.laneSelectionMode = laneSelectionMode;
            this.departurePos = pos;
            this.departureLane = lane;
            return this;
        }

        public Builder departureSpeed(DepartSpeedMode departSpeedMode, double departSpeed) {
            this.departSpeed = departSpeed;
            this.departSpeedMode = departSpeedMode;
            return this;
        }

        public Builder departureSpeed(double speed) {
            this.departSpeed = speed;
            this.departSpeedMode = DepartSpeedMode.PRECISE;
            return this;
        }

        public VehicleDeparture create() {
            return new VehicleDeparture(routeId, departureLane, departurePos, departSpeed, departSpeedMode, laneSelectionMode);
        }
    }
}
