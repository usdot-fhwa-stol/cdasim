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

package org.eclipse.mosaic.app.tutorial;

import org.eclipse.mosaic.fed.application.app.AbstractApplication;
import org.eclipse.mosaic.fed.application.app.api.VehicleApplication;
import org.eclipse.mosaic.fed.application.app.api.os.VehicleOperatingSystem;
import org.eclipse.mosaic.lib.enums.SensorType;
import org.eclipse.mosaic.lib.objects.vehicle.VehicleData;
import org.eclipse.mosaic.lib.util.scheduling.Event;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;

/**
 * This application shall induce vehicles to slow down in hazardous environments.
 * In onVehicleUpdated() the application requests new data from a vehicle's
 * sensors and analyzes the data with respect to strength after every single update.
 * Once a sensor indicates that a certain vehicle has entered a potentially
 * hazardous area, the application will reduce the speed of the respective vehicle
 * within a specified time frame. After the respective vehicle has left the dangerous
 * zone, its speed will no longer be reduced.
 */
public class SlowDownApp extends AbstractApplication<VehicleOperatingSystem> implements VehicleApplication {

    private final static float SPEED = 25 / 3.6f;

    private boolean hazardousArea = false;

    /**
     * This method is used to request new data from the sensors and in that case
     * react on the retrieved data.
     * It is called at each simulation step when the vehicle info has been updated for
     * the vehicle that has this application equipped.
     */
    @Override
    public void onVehicleUpdated(@Nullable VehicleData previousVehicleData, @Nonnull VehicleData updatedVehicleData) {

        // Enumeration of possible environment sensor types that are available in a vehicle
        SensorType[] types = SensorType.values();

        // Initialize sensor strength
        int strength = 0;

        /*
         * The current strength of each environment sensor is examined here.
         * If one is higher than zero, we reason that we are in a hazardous area with the
         * given hazard.
         */
        for (SensorType currentType : types) {
            // The strength of a detected sensor
            strength = getOs().getStateOfEnvironmentSensor(currentType);

            if (strength > 0) {
                break;
            }
        }

        if (strength > 0 && !hazardousArea) {
            // Reduce speed when entering potentially hazardous area
            getOs().changeSpeedWithInterval(SPEED, 5000);
            hazardousArea = true;
        }

        if (strength == 0 && hazardousArea) {
            // Reset speed when leaving potentially hazardous area
            getOs().resetSpeed();
            hazardousArea = false;
        }

    }

    @Override
    public void processEvent(Event event) throws Exception {

    }

    @Override
    public void onStartup() {

    }

    @Override
    public void onShutdown() {

    }

}
