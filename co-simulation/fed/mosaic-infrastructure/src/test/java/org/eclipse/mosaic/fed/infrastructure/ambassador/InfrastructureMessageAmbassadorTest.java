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

package org.eclipse.mosaic.fed.infrastructure.ambassador;

import org.eclipse.mosaic.interactions.communication.V2xMessageReception;
import org.eclipse.mosaic.interactions.detector.DetectedObjectInteraction;
import org.eclipse.mosaic.lib.geo.CartesianPoint;
import org.eclipse.mosaic.lib.math.Vector3d;
import org.eclipse.mosaic.lib.objects.detector.DetectedObject;
import org.eclipse.mosaic.lib.objects.detector.DetectionType;
import org.eclipse.mosaic.lib.objects.detector.Size;
import org.eclipse.mosaic.lib.util.junit.TestFileRule;
import org.eclipse.mosaic.rti.TIME;
import org.eclipse.mosaic.rti.api.InternalFederateException;
import org.eclipse.mosaic.rti.api.RtiAmbassador;
import org.eclipse.mosaic.rti.api.parameters.AmbassadorParameter;
import org.eclipse.mosaic.rti.api.parameters.FederateDescriptor;
import org.eclipse.mosaic.rti.config.CLocalHost;
import org.junit.After;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.RuleChain;
import org.junit.rules.TemporaryFolder;

import java.io.File;
import java.io.IOException;

import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.*;

/**
 * Tests for {@link InfrastructureMessageAmbassador}.
 */
public class InfrastructureMessageAmbassadorTest {

    private final TemporaryFolder temporaryFolder = new TemporaryFolder();

    private final TestFileRule testFileRule = new TestFileRule(temporaryFolder).basedir("infrastructure");

    @Rule
    public RuleChain chain = RuleChain.outerRule(temporaryFolder).around(testFileRule);

    private RtiAmbassador rtiMock;

    private InfrastructureMessageAmbassador ambassador;

    private InfrastructureInstanceManager instanceManagerMock;

    @Before
    public void setup() throws IOException {

        rtiMock = mock(RtiAmbassador.class);

        instanceManagerMock = mock(InfrastructureInstanceManager.class);
        FederateDescriptor handleMock = mock(FederateDescriptor.class);

        File workingDir = temporaryFolder.getRoot();

        CLocalHost testHostConfig = new CLocalHost();

        testHostConfig.workingDirectory = workingDir.getAbsolutePath();

        when(handleMock.getHost()).thenReturn(testHostConfig);

        when(handleMock.getId()).thenReturn("infrastructure");

        ambassador = new InfrastructureMessageAmbassador(new AmbassadorParameter("infrastructure",
                temporaryFolder.newFile("infrastructure/infrastructure_config.json")));

        ambassador.setRtiAmbassador(rtiMock);

        ambassador.setFederateDescriptor(handleMock);
    }

    @After
    public void teardown() throws IOException {
        ambassador.close();
    }

    @Test
    public void initialize() throws Throwable {

        // RUN
        ambassador.initialize(0, 100 * TIME.SECOND);
        // ASSERT
        verify(rtiMock, times(1)).requestAdvanceTime(eq(0L), eq(0L), eq((byte) 1));
    }

    @Test
    public void testProcessInteraction() throws InternalFederateException{
        ambassador.initialize(0, 100 * TIME.SECOND);DetectedObject detectedObject = new DetectedObject(
                DetectionType.VAN,
                0.5,
                "sensor1",
                "projection String",
                "Object1",
                CartesianPoint.xyz(1.1, 2, 3.2),
                new Vector3d(2, 3, 4),
                new Vector3d(-4.4,-5.5,-6.6),
                new Size(3, 4, 5));
        Double[] covarianceMatrix = new Double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        detectedObject.setPositionCovariance(covarianceMatrix);
        detectedObject.setVelocityCovariance(covarianceMatrix);
        detectedObject.setAngularVelocityCovariance(covarianceMatrix);
        DetectedObjectInteraction interaction = new DetectedObjectInteraction(100,detectedObject);

        ambassador.processInteraction(interaction);

    }

}
