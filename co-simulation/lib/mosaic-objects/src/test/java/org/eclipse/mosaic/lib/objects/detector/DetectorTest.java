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
package org.eclipse.mosaic.lib.objects.detector;

import static org.junit.Assert.assertEquals;

import org.eclipse.mosaic.lib.geo.CartesianPoint;
import org.junit.Before;
import org.junit.Test;

import com.google.gson.Gson;

public class DetectorTest {
    @Before
    public void setUp() throws Exception {

    }

    @Test
    public void testDetectorJsonSerialization() {
        Detector sensor = new Detector("something", DetectorType.SEMANTIC_LIDAR,new DetectorReferenceLocation( LocationDataType.CARTESIAN,
                CartesianPoint.xyz(1, 2, 3), new Orientation(23.0, 0, 0)));
        Gson gson = new Gson();
        String sensorJson = gson.toJson(sensor);
        String predictedSensorJson = "{"
        +   "\"sensorId\":\"something\","
        +   "\"type\":\"SemanticLidar\","
        +   "\"ref\":{"
        +       "\"type\":\"CARTESIAN\","
        +       "\"location\":"
        +       "{"
        +           "\"x\":1.0,"
        +           "\"y\":2.0,"
        +           "\"z\":3.0"
        +       "},"
        +       "\"orientation\":"
        +       "{"
        +           "\"yaw\":23.0,"
        +           "\"pitch\":0.0,"
        +           "\"roll\":0.0"
        +       "}"
        +  "}"
        + "}";
        
        assertEquals(sensorJson, predictedSensorJson);
    }

    @Test
    public void testDetectorJsonDeserialization() {
        String predictedSensorJson = "{"
        +   "\"sensorId\":\"something\","
                + "\"type\":\"SemanticLidar\","
        +   "\"ref\":{"
        +       "\"type\":\"CARTESIAN\","
        +       "\"orientation\":"
        +       "{"
        +           "\"yaw\":23.0,"
        +           "\"pitch\":0.0,"
        +           "\"roll\":0.0"
        +       "},"
        +       "\"location\":"
        +       "{"
        +           "\"x\":1.0,"
        +           "\"y\":2.0,"
        +           "\"z\":3.0"
        +       "}"
        +   "}"
        +"}";
        Gson gson = new Gson();
        Detector predictedSensor = new Detector("something", DetectorType.SEMANTIC_LIDAR,new DetectorReferenceLocation( LocationDataType.CARTESIAN,
        CartesianPoint.xyz(1, 2, 3), new Orientation(23.0, 0, 0)));
        Detector sensor = gson.fromJson(predictedSensorJson, Detector.class);
        assertEquals(sensor, predictedSensor);
        assertEquals(sensor.hashCode(), predictedSensor.hashCode());
    }

    @Test
    public void testGetterSetterConstructor() {
        Detector sensor = new Detector("something", DetectorType.SEMANTIC_LIDAR,new DetectorReferenceLocation( LocationDataType.CARTESIAN,
            CartesianPoint.xyz(1, 2, 3), new Orientation(23.0, 0, 0)));
        assertEquals("something", sensor.getSensorId());
        assertEquals(DetectorType.SEMANTIC_LIDAR, sensor.getType());
        assertEquals(new Orientation(23.0, 0, 0), sensor.getRef().getOrientation());
        assertEquals(CartesianPoint.xyz(1,2,3), sensor.getRef().getLocation());
        // Test Setters
        sensor.setSensorId("NewSensor");
        sensor.getRef().setLocation(CartesianPoint.xy(45,67));
        sensor.getRef().setOrientation(new Orientation(22, 33, 44));
        sensor.setType(DetectorType.INSTANCE_SEGMENTATION_CAMERA);
        assertEquals("NewSensor", sensor.getSensorId());
        assertEquals(DetectorType.INSTANCE_SEGMENTATION_CAMERA, sensor.getType());
        assertEquals(new Orientation(22, 33, 44), sensor.getRef().getOrientation());
        assertEquals(CartesianPoint.xy(45,67), sensor.getRef().getLocation());
    }
    @Test
    public void testToString() {
        Detector sensor = new Detector("something", DetectorType.SEMANTIC_LIDAR,new DetectorReferenceLocation( LocationDataType.CARTESIAN,
        CartesianPoint.xyz(1, 2, 3), new Orientation(23.0, 0, 0)));
        String sensoString = "Detector [sensorId=something, type=SEMANTIC_LIDAR, ref=DetectorReferenceLocation [locationDataType=CARTESIAN, location=CartesianPoint{x=1.00,y=2.00,z=3.00}, orientation=Orientation [yaw=23.0, pitch=0.0, roll=0.0]]]";      
        assertEquals(sensoString, sensor.toString());
    }
}


