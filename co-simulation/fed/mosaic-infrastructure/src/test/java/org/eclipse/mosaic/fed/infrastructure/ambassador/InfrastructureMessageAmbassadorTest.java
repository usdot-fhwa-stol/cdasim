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

import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import org.eclipse.mosaic.interactions.communication.V2xMessageReception;
import org.eclipse.mosaic.interactions.detector.DetectedObjectInteraction;
import org.eclipse.mosaic.interactions.detector.DetectorRegistration;
import org.eclipse.mosaic.interactions.mapping.RsuRegistration;
import org.eclipse.mosaic.lib.geo.CartesianPoint;
import org.eclipse.mosaic.lib.geo.GeoPoint;
import org.eclipse.mosaic.lib.junit.GeoProjectionRule;
import org.eclipse.mosaic.lib.junit.IpResolverRule;
import org.eclipse.mosaic.lib.math.Vector3d;
import org.eclipse.mosaic.lib.objects.detector.DetectedObject;
import org.eclipse.mosaic.lib.objects.detector.DetectionType;
import org.eclipse.mosaic.lib.objects.detector.Detector;
import org.eclipse.mosaic.lib.objects.detector.DetectorType;
import org.eclipse.mosaic.lib.objects.detector.Orientation;
import org.eclipse.mosaic.lib.objects.detector.Size;
import org.eclipse.mosaic.lib.util.junit.TestFileRule;
import org.eclipse.mosaic.rti.TIME;
import org.eclipse.mosaic.rti.api.IllegalValueException;
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
import org.mockito.ArgumentCaptor;
import org.mockito.internal.util.reflection.FieldSetter;

import gov.dot.fhwa.saxton.CarmaV2xMessageReceiver;

/**
 * Tests for {@link InfrastructureMessageAmbassador}.
 */
public class InfrastructureMessageAmbassadorTest {

    private final TemporaryFolder temporaryFolder = new TemporaryFolder();

    private final TestFileRule testFileRule = new TestFileRule(temporaryFolder).basedir("infrastructure");

    @Rule
    public RuleChain chain = RuleChain.outerRule(temporaryFolder).around(testFileRule);

    private static GeoPoint BERLIN = GeoPoint.latLon(52.5, 13.4);

    @Rule
    public GeoProjectionRule geoProjectionRule = new GeoProjectionRule(BERLIN);

    @Rule
    public IpResolverRule ipResolverRule = new IpResolverRule();

    private RtiAmbassador rtiMock;

    private InfrastructureMessageAmbassador ambassador;

    private InfrastructureInstanceManager instanceManagerMock;

    private InfrastructureRegistrationReceiver receiverMock;

    private CarmaV2xMessageReceiver v2xMessageReceiverMock;

    @Before
    public void setup() throws IOException, NoSuchFieldException, InternalFederateException, IllegalValueException {

        rtiMock = mock(RtiAmbassador.class);
        FederateDescriptor handleMock = mock(FederateDescriptor.class);
        
        instanceManagerMock = mock(InfrastructureInstanceManager.class);

        receiverMock = mock(InfrastructureRegistrationReceiver.class);

        v2xMessageReceiverMock = mock(CarmaV2xMessageReceiver.class);


        File workingDir = temporaryFolder.getRoot();

        CLocalHost testHostConfig = new CLocalHost();

        testHostConfig.workingDirectory = workingDir.getAbsolutePath();

        when(handleMock.getHost()).thenReturn(testHostConfig);

        when(handleMock.getId()).thenReturn("infrastructure");

        ambassador = new InfrastructureMessageAmbassador(new AmbassadorParameter("infrastructure",
                temporaryFolder.newFile("infrastructure/infrastructure_config.json")));

        ambassador.setRtiAmbassador(rtiMock);

        ambassador.setFederateDescriptor(handleMock);
        FieldSetter.setField(ambassador, ambassador.getClass().getDeclaredField("infrastructureInstanceManager"), instanceManagerMock);
        
        ArrayList<Detector> sensors = new ArrayList<>();
        sensors.add(
            new Detector(
                "sensor1", 
                DetectorType.SEMANTIC_LIDAR, 
                new Orientation( 0.0,0.0,0.0),
                CartesianPoint.ORIGO));
        sensors.add(
            new Detector(
                "sensor2", 
                DetectorType.SEMANTIC_LIDAR, 
                new Orientation( 20.0,0.0,0.0),
                CartesianPoint.xy(1,1)));
        sensors.add(
            new Detector(
                "sensor3", 
                DetectorType.SEMANTIC_LIDAR, 
                new Orientation( 180.0,0.0,0.0),
                CartesianPoint.xy(-1,1)));
        InfrastructureRegistrationMessage message = new InfrastructureRegistrationMessage(
                "127.0.0.1",
                 "rsu_1", 
                 4567 , 
                 5678, 
                 8642, 
                 CartesianPoint.xy(1, 2), 
                 sensors);
        ArrayList<InfrastructureRegistrationMessage> registrationMessages = new ArrayList<>();
        registrationMessages.add(message);
        when(receiverMock.getReceivedMessages()).thenReturn(registrationMessages);

        // ASSERT
        FieldSetter.setField(ambassador, ambassador.getClass().getDeclaredField("infrastructureRegistrationReceiver"), receiverMock);
        FieldSetter.setField(ambassador, ambassador.getClass().getDeclaredField("v2xMessageReceiver"), v2xMessageReceiverMock);

    }

    @After
    public void teardown() throws IOException {
        
    }
    @Test
    public void testInitialize() throws InternalFederateException, IllegalValueException{
        ambassador.initialize(0, 100 * TIME.SECOND);
        verify(rtiMock, times(1)).requestAdvanceTime(eq(0L), eq(0L), eq((byte) 1));
        ambassador.close();

    }
    @Test
    public void testProcessInteraction() throws InternalFederateException{
        DetectedObject detectedObject = new DetectedObject(
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
        verify(instanceManagerMock).onDetectedObject(detectedObject);

    }

    @Test
    public void testProcessTimeAdvanceGrant() throws InternalFederateException, IllegalValueException, NoSuchFieldException, SecurityException {
        //Initialize GeoSingleton

        ambassador.processTimeAdvanceGrant(100);
        verify(receiverMock, times(1)).getReceivedMessages();
        verify(rtiMock, times(3)).triggerInteraction(any(DetectorRegistration.class));
        verify(rtiMock, times(1)).triggerInteraction(any(RsuRegistration.class));
        verify(rtiMock, times(0)).triggerInteraction(any(V2xMessageReception.class));

    }

}
