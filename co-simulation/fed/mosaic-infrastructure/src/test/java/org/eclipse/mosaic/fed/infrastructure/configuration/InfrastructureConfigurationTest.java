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

package org.eclipse.mosaic.fed.infrastructure.configuration;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

import org.eclipse.mosaic.lib.util.objects.ObjectInstantiation;

import org.junit.Test;

public class InfrastructureConfigurationTest {

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
        String validConfig = "/infrastructure_config.json";
        InfrastructureConfiguration infrastructureConfiguration = getInfrastructureConfiguration(validConfig);
        // ASSERT
        assertNotNull(infrastructureConfiguration); // assert that configuration is created
        assertEquals(Long.valueOf(100L), infrastructureConfiguration.updateInterval);
        assertEquals("rsu_0", infrastructureConfiguration.senderRSUId);
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
    private InfrastructureConfiguration getInfrastructureConfiguration(String filePath) throws InstantiationException {
        return new ObjectInstantiation<>(InfrastructureConfiguration.class)
                .read(getClass().getResourceAsStream(filePath));
    }

}