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

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import gov.dot.fhwa.saxton.CarmaV2xMessageReceiver;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.eclipse.mosaic.interactions.communication.AdHocCommunicationConfiguration;
import org.eclipse.mosaic.interactions.detector.DetectedObjectInteraction;
import org.eclipse.mosaic.interactions.detector.DetectorRegistration;
import org.eclipse.mosaic.interactions.mapping.RsuRegistration;
import org.eclipse.mosaic.lib.enums.AdHocChannel;
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
import org.eclipse.mosaic.rti.api.Interaction;
import org.eclipse.mosaic.rti.api.InternalFederateException;
import org.eclipse.mosaic.rti.api.RtiAmbassador;
import org.eclipse.mosaic.rti.api.parameters.AmbassadorParameter;
import org.eclipse.mosaic.rti.api.parameters.FederateDescriptor;
import org.eclipse.mosaic.rti.config.CLocalHost;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.RuleChain;
import org.junit.rules.TemporaryFolder;
import org.mockito.ArgumentCaptor;
import org.mockito.internal.util.reflection.FieldSetter;



/**
 * Tests for {@link InfrastructureMessageAmbassador}.
 */
public class InfrastructureMessageAmbassadorTest {

    private final TemporaryFolder temporaryFolder = new TemporaryFolder();

    private final TestFileRule testFileRule = new TestFileRule(temporaryFolder).basedir("infrastructure");

    @Rule
    public RuleChain chain = RuleChain.outerRule(temporaryFolder).around(testFileRule);

    private static GeoPoint BERLIN = GeoPoint.latLon(52.5, 13.4);
    /**
     * Rule to initialize {@link GeoProjection} Singleton.
     */
    @Rule
    public GeoProjectionRule geoProjectionRule = new GeoProjectionRule(BERLIN);
    /**
     * Rule to initialize {@link IpResolver} Singleton.
     */
    @Rule
    public IpResolverRule ipResolverRule = new IpResolverRule();

    /**
     * {@link RtiAmbassador} mock.
     */
    private RtiAmbassador rtiMock;

    private InfrastructureMessageAmbassador ambassador;
    /**
     * {@link InfrastructureInstanceManager} mock.
     */
    private InfrastructureInstanceManager instanceManagerMock;
    /**
     * {@link InfrastructureRegistrationReceiver} mock.
     */
    private InfrastructureRegistrationReceiver receiverMock;
    /**
     * {@link CarmaV2xMessageReceiver} mock,
     */
    private CarmaV2xMessageReceiver v2xMessageReceiverMock;

    private ArrayList<InfrastructureRegistrationMessage> registrationMessages;

    @Before
    public void setup() throws IOException, NoSuchFieldException, InternalFederateException, IllegalValueException {
        // Initialize Mocks
        rtiMock = mock(RtiAmbassador.class);
        FederateDescriptor handleMock = mock(FederateDescriptor.class);
        instanceManagerMock = mock(InfrastructureInstanceManager.class);
        receiverMock = mock(InfrastructureRegistrationReceiver.class);
        v2xMessageReceiverMock = mock(CarmaV2xMessageReceiver.class);
        File workingDir = temporaryFolder.getRoot();
        CLocalHost testHostConfig = new CLocalHost();
        testHostConfig.workingDirectory = workingDir.getAbsolutePath();

        // Mock methods     
        when(handleMock.getHost()).thenReturn(testHostConfig);
        when(handleMock.getId()).thenReturn("infrastructure");
        // Create Infrastructure registration
        ArrayList<Detector> rsu_1_sensors = new ArrayList<>();
        rsu_1_sensors.add(
            new Detector(
                "sensor1", 
                DetectorType.SEMANTIC_LIDAR, 
                new Orientation( 0.0,0.0,0.0),
                CartesianPoint.ORIGO));
        rsu_1_sensors.add(
            new Detector(
                "sensor2", 
                DetectorType.SEMANTIC_LIDAR, 
                new Orientation( 20.0,0.0,0.0),
                CartesianPoint.xy(1,1)));
        rsu_1_sensors.add(
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
                 rsu_1_sensors);
        registrationMessages = new ArrayList<>();
        registrationMessages.add(message);
        when(receiverMock.getReceivedMessages()).thenReturn(registrationMessages);

        // Set mocks as ambassador members through reflection or setters
        ambassador = new InfrastructureMessageAmbassador(new AmbassadorParameter("infrastructure",
        temporaryFolder.newFile("infrastructure/infrastructure_config.json")));
        ambassador.setRtiAmbassador(rtiMock);
        ambassador.setFederateDescriptor(handleMock);
        FieldSetter.setField(ambassador, ambassador.getClass().getDeclaredField("infrastructureRegistrationReceiver"), receiverMock);
        FieldSetter.setField(ambassador, ambassador.getClass().getDeclaredField("v2xMessageReceiver"), v2xMessageReceiverMock);
        FieldSetter.setField(ambassador, ambassador.getClass().getDeclaredField("infrastructureInstanceManager"), instanceManagerMock);


    }

    @Test
    public void testInitialize() throws InternalFederateException, IllegalValueException{
        // Test initialize method
        ambassador.initialize(0, 100 * TIME.SECOND);
        verify(rtiMock, times(1)).requestAdvanceTime(eq(0L), eq(0L), eq((byte) 1));
        // cleanup threads and UDP Sockets
        ambassador.close();

    }
    @Test
    public void testProcessInteraction() throws InternalFederateException{
        // Test process interaction for detected objects interactions
        DetectedObject detectedObject = new DetectedObject(
                DetectionType.VAN,
                0.5,
                "sensor1",
                "projection String",
                100,
                CartesianPoint.xyz(1.1, 2, 3.2),
                new Vector3d(2, 3, 4),
                new Vector3d(-4.4,-5.5,-6.6),
                new Size(3, 4, 5),
                100);
        Double[][] covarianceMatrix =  { {0.0, 0.0, 0.0} , {0.0, 0.0, 0.0} , {0.0, 0.0, 0.0}};
        detectedObject.setPositionCovariance(covarianceMatrix);
        detectedObject.setVelocityCovariance(covarianceMatrix);
        detectedObject.setAngularVelocityCovariance(covarianceMatrix);
        DetectedObjectInteraction interaction = new DetectedObjectInteraction(100,detectedObject);

        ambassador.processInteraction(interaction);
        verify(instanceManagerMock).onDetectedObject(detectedObject);

    }

    @Test
    public void testProcessTimeAdvanceGrant() throws InternalFederateException, IllegalValueException, NoSuchFieldException, SecurityException {
        //Test processTimeAdvanceGrant for Infrastructure Registration
        ambassador.processTimeAdvanceGrant(100);
        // Verify received messages were attempted to be pulled from Infrastructure Registration Receiver mock
        verify(receiverMock, times(1)).getReceivedMessages();
        // Capture interaction triggered with trigger interaction call
        ArgumentCaptor<Interaction> interactions = ArgumentCaptor.forClass(Interaction.class);
        // Verify that trigger interaction was call 5 times on rti mock
        verify(rtiMock, times(5)).triggerInteraction(interactions.capture());
        // Separate triggered interactions by interaction type
        List<Interaction> captured_interactions = interactions.getAllValues();
        List<DetectorRegistration> capturedDetectorRegistrations = new ArrayList<>();
        List<RsuRegistration> capturedRsuRegistrations = new ArrayList<>();
        List<AdHocCommunicationConfiguration> capturedAdHocCommunicationConfiguration = new ArrayList<>();
        List<Interaction> otherInteractions = new ArrayList<>();
        for (Interaction interaction : captured_interactions) {
            if ( interaction.getClass().equals(DetectorRegistration.class)) {
                capturedDetectorRegistrations.add((DetectorRegistration) interaction);
            }
            else if (interaction.getClass().equals(RsuRegistration.class) ) {
                capturedRsuRegistrations.add((RsuRegistration)interaction);
            }
            else if (interaction.getClass().equals(AdHocCommunicationConfiguration.class)) {
                capturedAdHocCommunicationConfiguration.add((AdHocCommunicationConfiguration) interaction);
            }
            else {
                otherInteractions.add(interaction);
            }
        }

        int rsu_registration_index = 0;
        int sensor_registration_index = 0;
        // Loop through registration messages given message ambassador 
        for (InfrastructureRegistrationMessage registrationMessage : registrationMessages) {
            // For each registration message, confirm that all the sensors in the regisration message
            // trigger sensor registration calls.
            for (Detector detector: registrationMessage.getSensors()) {
                assertEquals(detector, capturedDetectorRegistrations.get(sensor_registration_index).getDetector());
                sensor_registration_index++;
            }
            // For each registration message, ensure that RsuRegistration interactions and AdHocCommunicationConfiguration
            // interactions are trigger
            RsuRegistration registration = capturedRsuRegistrations.get( rsu_registration_index);
            AdHocCommunicationConfiguration adhocConfig = capturedAdHocCommunicationConfiguration.get(rsu_registration_index);
            assertEquals(registrationMessage.getInfrastructureId(), registration.getMapping().getName());
            assertTrue(
                registrationMessage.getLocation().toVector3d().isFuzzyEqual(
                   registration.getMapping().getPosition().toCartesian().toVector3d()
                )
            );
            assertEquals(300.0, adhocConfig.getConfiguration().getConf0().getRadius(), 0.01);
            assertEquals(50.0, adhocConfig.getConfiguration().getConf0().getNewPower(), 0.01);
            assertEquals(AdHocChannel.CCH, adhocConfig.getConfiguration().getConf0().getChannel0());
            rsu_registration_index++;
        }

        // Assert that no other interactions were triggered
        assertEquals(0, otherInteractions.size());

    }

}
