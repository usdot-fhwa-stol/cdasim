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
package org.eclipse.mosaic.fed.carla.ambassador;

import org.junit.Test;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import org.eclipse.mosaic.fed.carla.carlaconnect.CarlaXmlRpcClient;
import org.eclipse.mosaic.lib.util.junit.TestFileRule;
import org.eclipse.mosaic.rti.TIME;
import org.eclipse.mosaic.rti.api.RtiAmbassador;
import org.eclipse.mosaic.rti.api.parameters.AmbassadorParameter;
import org.eclipse.mosaic.rti.api.parameters.FederateDescriptor;
import org.eclipse.mosaic.rti.config.CLocalHost;

import org.junit.Before;
import org.junit.Rule;
import org.junit.rules.RuleChain;
import org.junit.rules.TemporaryFolder;

import java.io.File;
import java.io.IOException;

/**
 * Tests for {@link CarlaAmbassador}.
 */
public class CarlaAmbassadorTest {

    private final TemporaryFolder temporaryFolder = new TemporaryFolder();

    private final TestFileRule testFileRule = new TestFileRule(temporaryFolder).basedir("carla");

    @Rule
    public RuleChain chain = RuleChain.outerRule(temporaryFolder).around(testFileRule);

    private RtiAmbassador rtiMock;

    private CarlaAmbassador ambassador;

    @Before
    public void setup() throws IOException {

        rtiMock = mock(RtiAmbassador.class);



        FederateDescriptor handleMock = mock(FederateDescriptor.class);

        File workingDir = temporaryFolder.getRoot();

        CLocalHost testHostConfig = new CLocalHost();

        testHostConfig.workingDirectory = workingDir.getAbsolutePath();

        when(handleMock.getHost()).thenReturn(testHostConfig);

        when(handleMock.getId()).thenReturn("carla");

        ambassador = new CarlaAmbassador(
                new AmbassadorParameter("carla", temporaryFolder.newFile("carla/carla_config.json")));

        ambassador.setRtiAmbassador(rtiMock);

        ambassador.setFederateDescriptor(handleMock);
    }

    @Test
    public void initialize() throws Throwable {

        // RUN
        ambassador.initialize(0, 100 * TIME.SECOND);
        // ASSERT
        verify(rtiMock, times(1)).requestAdvanceTime(eq(0L), eq(0L), eq((byte) 1));
    }

}
