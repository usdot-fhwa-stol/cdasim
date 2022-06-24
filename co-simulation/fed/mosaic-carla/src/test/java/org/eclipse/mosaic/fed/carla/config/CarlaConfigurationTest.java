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

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

import org.eclipse.mosaic.lib.util.objects.ObjectInstantiation;

import org.junit.Test;

public class CarlaConfigurationTest {

    /**
     * Test case using a properly formed json configuration and asserting, that all
     * values were properly deserialized.
     *
     * @throws InstantiationException if configuration couldn't be properly
     *                                deserialized, under normal circumstances this
     *                                should not occur
     */
    @Test
    public void readConfig_assertProperties() throws InstantiationException {
        // SETUP + RUN
        String validConfig = "/carla_config.json";
        CarlaConfiguration carlaConfiguration = getCarlaConfiguration(validConfig);
        // ASSERT
        assertNotNull(carlaConfiguration); // assert that configuration is created
        assertEquals(Long.valueOf(200L), carlaConfiguration.updateInterval);
        assertEquals("D:/CARLA_0.9.10/", carlaConfiguration.carlaUE4Path);
        assertEquals("./scenarios/Town04_10/carla; bridge.bat", carlaConfiguration.bridgePath);
        assertEquals(8913, carlaConfiguration.carlaConnectionPort);
    }

    /**
     * Small helper class, which returns the instantiated object of a
     * json-configuration.
     *
     * @param filePath the path to the configuration
     * @return the instantiated object
     * @throws InstantiationException if there was an error during
     *                                deserialization/instantiation
     */
    private CarlaConfiguration getCarlaConfiguration(String filePath) throws InstantiationException {
        return new ObjectInstantiation<>(CarlaConfiguration.class).read(getClass().getResourceAsStream(filePath));
    }

}