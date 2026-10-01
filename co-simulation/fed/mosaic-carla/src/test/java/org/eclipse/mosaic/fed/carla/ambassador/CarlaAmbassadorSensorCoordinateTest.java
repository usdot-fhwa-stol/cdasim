/*
 * Copyright (C) 2026 LEIDOS.
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
package org.eclipse.mosaic.fed.carla.ambassador;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

import java.nio.file.Files;
import java.nio.file.Path;

import org.eclipse.mosaic.fed.carla.carlaconnect.CarlaXmlRpcClient;
import org.eclipse.mosaic.fed.carla.config.CarlaConfiguration;
import org.eclipse.mosaic.interactions.detector.DetectorRegistration;
import org.eclipse.mosaic.lib.geo.CartesianPoint;
import org.eclipse.mosaic.lib.objects.detector.Detector;
import org.eclipse.mosaic.lib.objects.detector.DetectorType;
import org.eclipse.mosaic.lib.objects.detector.Orientation;
import org.eclipse.mosaic.rti.api.parameters.AmbassadorParameter;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.mockito.ArgumentCaptor;
import org.mockito.internal.util.reflection.FieldSetter;

class CarlaAmbassadorSensorCoordinateTest {

    @TempDir
    Path temporaryDirectory;

    private CarlaAmbassador ambassador;
    private CarlaXmlRpcClient carlaXmlRpcClientMock;

    @BeforeEach
    void setup() throws Exception {
        Path configPath = temporaryDirectory.resolve("carla_config.json");
        Files.write(configPath, "{}".getBytes());
        ambassador = new CarlaAmbassador(new AmbassadorParameter("carla", configPath.toFile()));
        carlaXmlRpcClientMock = mock(CarlaXmlRpcClient.class);
        FieldSetter.setField(
                ambassador,
                ambassador.getClass().getDeclaredField("carlaXmlRpcClient"),
                carlaXmlRpcClientMock
        );
    }

    @Test
    void preservesCarlaCoordinatesByDefault() throws Exception {
        DetectorRegistration registration = registrationAt(-46.0, 127.1, 10.0, 15.0);

        ambassador.processInteraction(registration);

        ArgumentCaptor<DetectorRegistration> registrationCaptor =
                ArgumentCaptor.forClass(DetectorRegistration.class);
        verify(carlaXmlRpcClientMock).createSensor(registrationCaptor.capture());
        assertSame(registration, registrationCaptor.getValue());
    }

    @Test
    void convertsSumoCoordinatesAndOrientationToCarla() throws Exception {
        CarlaConfiguration config = new CarlaConfiguration();
        config.sensorCoordinateFrame = "SUMO";
        FieldSetter.setField(ambassador, ambassador.getClass().getDeclaredField("carlaConfig"), config);
        FieldSetter.setField(
                ambassador,
                ambassador.getClass().getDeclaredField("sumoNetOffsetXY"),
                new double[]{109.34, 135.96}
        );

        ambassador.processInteraction(registrationAt(63.34, 8.86, 10.0, 90.0));

        ArgumentCaptor<DetectorRegistration> registrationCaptor =
                ArgumentCaptor.forClass(DetectorRegistration.class);
        verify(carlaXmlRpcClientMock).createSensor(registrationCaptor.capture());

        Detector convertedDetector = registrationCaptor.getValue().getDetector();
        assertEquals(-46.0, convertedDetector.getLocation().getX(), 0.0001);
        assertEquals(127.1, convertedDetector.getLocation().getY(), 0.0001);
        assertEquals(10.0, convertedDetector.getLocation().getZ(), 0.0001);
        assertEquals(0.0, convertedDetector.getOrientation().getYaw(), 0.0001);
        assertEquals(2.0, convertedDetector.getOrientation().getPitch(), 0.0001);
        assertEquals(3.0, convertedDetector.getOrientation().getRoll(), 0.0001);
    }

    private DetectorRegistration registrationAt(double x, double y, double z, double yaw) {
        Detector detector = new Detector(
                "sensorID1",
                DetectorType.SEMANTIC_LIDAR,
                new Orientation(yaw, 2.0, 3.0),
                CartesianPoint.xyz(x, y, z)
        );
        return new DetectorRegistration(0, detector, "rsu_1");
    }
}
