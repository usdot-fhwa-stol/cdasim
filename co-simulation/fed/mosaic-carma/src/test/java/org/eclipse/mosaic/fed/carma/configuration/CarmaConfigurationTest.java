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

package org.eclipse.mosaic.fed.carma.configuration;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

import org.eclipse.mosaic.lib.geo.MutableCartesianPoint;
import org.eclipse.mosaic.lib.geo.MutableGeoPoint;
import org.eclipse.mosaic.lib.util.objects.ObjectInstantiation;

import org.junit.Test;

/**
 * Test for {@link CarmaConfiguration}.
 *
 */
public class CarmaConfigurationTest {

    /**
     * Test case using a properly formed json configuration and asserting, that all
     * values were properly deserialized.
     *
     * @throws InstantiationException if configuration couldn't be properly
     *                                deserialized, under normal circumstances this
     *                                should not occur.
     */
    @Test
    public void readConfig_assertProperties() throws InstantiationException {
        // SETUP + RUN
        String validConfig = "/carma_config.json";
        CarmaConfiguration carmaConfiguration = getCarmaConfiguration(validConfig);
        // ASSERT
        assertNotNull(carmaConfiguration); // assert that configuration is created
        assertEquals(Long.valueOf(100L), carmaConfiguration.updateInterval);
        assertEquals("0", carmaConfiguration.carmaVehicles.get(0).routeID);
        assertEquals(1, carmaConfiguration.carmaVehicles.get(0).lane);
        assertEquals(Double.valueOf(0.0D), carmaConfiguration.carmaVehicles.get(0).position, 0.001);
        assertEquals(Double.valueOf(0.0D), carmaConfiguration.carmaVehicles.get(0).departSpeed, 0.001);
        assertEquals("vehicle.chevrolet.impala", carmaConfiguration.carmaVehicles.get(0).vehicleType);
        assertEquals("org.eclipse.mosaic.app.tutorial.VehicleCommunicationApp",
                carmaConfiguration.carmaVehicles.get(0).applications.get(0));
        assertEquals(new MutableGeoPoint(52.579272059028646, 13.467165499469328),
                carmaConfiguration.carmaVehicles.get(0).geoPosition);
        assertEquals(new MutableCartesianPoint(501.62, 116.95, 0.0),
                carmaConfiguration.carmaVehicles.get(0).projectedPosition);
        assertEquals(Double.valueOf(24.204351784500364D), carmaConfiguration.carmaVehicles.get(0).heading);
        assertEquals(Double.valueOf(0.0), carmaConfiguration.carmaVehicles.get(0).slope, 0.001);
        assertEquals("carma_0", carmaConfiguration.senderCarmaVehicleId);
    }

    /**
     * Small helper class, which returns the instantiated object of a
     * json-configuration.
     *
     * @param filePath the path to the configuration.
     * @return the instantiated object.
     * @throws InstantiationException if there was an error during
     *                                deserialization/instantiation.
     */
    private CarmaConfiguration getCarmaConfiguration(String filePath) throws InstantiationException {
        return new ObjectInstantiation<>(CarmaConfiguration.class).read(getClass().getResourceAsStream(filePath));
    }

}