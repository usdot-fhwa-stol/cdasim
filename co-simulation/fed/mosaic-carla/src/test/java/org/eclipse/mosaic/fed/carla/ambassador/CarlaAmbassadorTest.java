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

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.apache.xmlrpc.XmlRpcException;
import org.eclipse.mosaic.fed.carla.carlaconnect.CarlaXmlRpcClient;
import org.eclipse.mosaic.fed.carla.config.CarlaConfiguration;
import org.eclipse.mosaic.interactions.detector.DetectedObjectInteraction;
import org.eclipse.mosaic.interactions.detector.DetectorRegistration;
import org.eclipse.mosaic.lib.geo.CartesianPoint;
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
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.RuleChain;
import org.junit.rules.TemporaryFolder;
import org.mockito.internal.util.reflection.FieldSetter;

import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.doThrow;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;




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

    private CarlaXmlRpcClient carlaXmlRpcClientMock;

    @Before
    public void setup() throws IOException, NoSuchFieldException {

        rtiMock = mock(RtiAmbassador.class);

        carlaXmlRpcClientMock = mock(CarlaXmlRpcClient.class); 

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

        FieldSetter.setField(ambassador, ambassador.getClass().getDeclaredField("carlaXmlRpcClient"), carlaXmlRpcClientMock);

       

    }

    @Test
    public void initialize() throws Throwable {
        CarlaConfiguration config = new CarlaConfiguration();
        config.carlaCDASimAdapterUrl="https://testing/something";
        FieldSetter.setField(ambassador, ambassador.getClass().getDeclaredField("carlaConfig"), config);

        // RUN
        ambassador.initialize(0, 100 * TIME.SECOND);
        // ASSERT
        verify(rtiMock, times(1)).requestAdvanceTime(eq(0L), eq(0L), eq((byte) 1));
        verify(carlaXmlRpcClientMock, times(1)).initialize();
    }

    @Test
    public void processTimeAdvanceGrant() throws InternalFederateException, NoSuchFieldException, SecurityException, XmlRpcException, IllegalValueException {
        List<DetectorRegistration> registeredDetectors = new ArrayList<>();
        Detector detector = new Detector("sensorID1", DetectorType.SEMANTIC_LIDAR, new Orientation( 0.0,0.0,0.0), CartesianPoint.ORIGO);
        DetectorRegistration registration = new DetectorRegistration(0, detector, "rsu_2");
        registeredDetectors.add( registration);
        FieldSetter.setField(ambassador, ambassador.getClass().getDeclaredField("registeredDetectors"), registeredDetectors);

        // Setup Get detected objects return
        // Object 1 is CAR
        DetectedObject predictedCar = new DetectedObject(
                DetectionType.CAR,
                0.7,
                "sensorID1",
                "projection String2",
                100,
                CartesianPoint.xyz(-1.1, -2, -3.2),
                new Vector3d(1, 1, 1),
                new Vector3d(.1, .2, .3),
                new Vector3d(.1, .2, .3),
                new Vector3d(.1, .2, .3),
                new Size(2, 1, .5),
                100);
        Double[][] covarianceMatrix =  { {1.0, 0.0, 0.0} , {1.0, 0.0, 0.0} , {1.0, 0.0, 0.0}};
        predictedCar.setPositionCovariance(covarianceMatrix);
        predictedCar.setVelocityCovariance(covarianceMatrix);
        predictedCar.setAngularVelocityCovariance(covarianceMatrix);
        DetectedObject predictedBus = new DetectedObject(
            DetectionType.BUS,
            0.5,
            "sensorID1",
            "projection String",
            101,
            CartesianPoint.xyz(1.1, 2, 3.2),
            new Vector3d(0, 0, 0),
            new Vector3d(),
            new Vector3d(.1, .2, .3),
            new Vector3d(.1, .2, .3),
            new Size(0, 0, 0),
            100);
        Double[][] bus_covarianceMatrix =  { {0.0, 0.0, 0.0} , {0.0, 0.0, 0.0} , {0.0, 0.0, 0.0}};
        predictedBus.setPositionCovariance(bus_covarianceMatrix);
        predictedBus.setVelocityCovariance(bus_covarianceMatrix);
        predictedBus.setAngularVelocityCovariance(bus_covarianceMatrix);

        DetectedObject[] detectedObjects = {predictedBus, predictedCar};
        when(carlaXmlRpcClientMock.getDetectedObjects(registration.getInfrastructureId(), registration.getDetector().getSensorId() )).thenReturn(detectedObjects);

        ambassador.processTimeAdvanceGrant(100);

        verify(carlaXmlRpcClientMock, times(1)).getDetectedObjects(registration.getInfrastructureId(), registration.getDetector().getSensorId());
        verify(rtiMock, times(2)).triggerInteraction(any(DetectedObjectInteraction.class));
    }

    @Test
    public void processTimeAdvanceGrantException() throws InternalFederateException, NoSuchFieldException, SecurityException, XmlRpcException, IllegalValueException {
        List<DetectorRegistration> registeredDetectors = new ArrayList<>();
        Detector detector = new Detector("sensorID1", DetectorType.SEMANTIC_LIDAR, new Orientation( 0.0,0.0,0.0), CartesianPoint.ORIGO);
        DetectorRegistration registration = new DetectorRegistration(0, detector, "rsu_2");
        registeredDetectors.add( registration);
        FieldSetter.setField(ambassador, ambassador.getClass().getDeclaredField("registeredDetectors"), registeredDetectors);

        
        when(carlaXmlRpcClientMock.getDetectedObjects(registration.getInfrastructureId(), registration.getDetector().getSensorId() )).thenThrow(XmlRpcException.class);
        // Verify that when exceptiopn is thrown by CarlaXmlRpcClient, no interactions are trigger and exception is caught
        ambassador.processTimeAdvanceGrant(100);

        verify(carlaXmlRpcClientMock, times(1)).getDetectedObjects(registration.getInfrastructureId(), registration.getDetector().getSensorId());
        verify(rtiMock, times(0)).triggerInteraction(any(DetectedObjectInteraction.class));
        verify(carlaXmlRpcClientMock, times(1)).closeConnection();

    }

    @Test
    public void processDetectorRegistrationInteraction() throws XmlRpcException {
        Detector detector = new Detector("sensorID1", DetectorType.SEMANTIC_LIDAR, new Orientation( 0.0,0.0,0.0), CartesianPoint.ORIGO);
        DetectorRegistration registration = new DetectorRegistration(0, detector, "rsu_2");

        ambassador.processInteraction(registration);

        verify(carlaXmlRpcClientMock, times(1)).createSensor(registration);

    }

    @Test
    public void processDetectorRegistrationInteractionException() throws XmlRpcException {
        Detector detector = new Detector("sensorID1", DetectorType.SEMANTIC_LIDAR, new Orientation( 0.0,0.0,0.0), CartesianPoint.ORIGO);
        DetectorRegistration registration = new DetectorRegistration(0, detector, "rsu_2");

        doThrow(new XmlRpcException("")).when(carlaXmlRpcClientMock).createSensor(registration);
        ambassador.processInteraction(registration);

        verify(carlaXmlRpcClientMock, times(1)).createSensor(registration);
        verify(carlaXmlRpcClientMock, times(1)).closeConnection();


    }



}
