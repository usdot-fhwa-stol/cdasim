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
package org.eclipse.mosaic.interactions.sensor;

import static org.junit.Assert.assertEquals;

import org.eclipse.mosaic.lib.geo.CartesianPoint;
import org.junit.Before;
import org.junit.Test;

import com.google.gson.Gson;

public class SensorRegistrationTest {
    @Before
    public void setUp() throws Exception {

    }

    @Test
    public void testSensorRegistrationJsonSerialization() {
        Sensor sensor = new Sensor("something", SensorType.SEMANTIC_LIDAR, new Orientation(23.0, 0, 0),
                CartesianPoint.xyz(1, 2, 3));
        Gson gson = new Gson();
        String sensorJson = gson.toJson(sensor);
        String predictedSensorJson = "{"
        +   "\"sensorId\":\"something\","
        +   "\"type\":\"SemanticLidar\","
        +   "\"orientation\":"
        +   "{"
        +       "\"yaw\":23.0,"
        +       "\"pitch\":0.0,"
        +       "\"roll\":0.0"
        +   "},"
        +   "\"location\":"
        +   "{"
        +       "\"x\":1.0,"
        +       "\"y\":2.0,"
        +       "\"z\":3.0"
        +   "}"
        +"}";
        assertEquals(sensorJson, predictedSensorJson);
    }

    @Test
    public void testSensorRegistrationJsonDeserialization() {
        String predictedSensorJson = "{"
        +   "\"sensorId\":\"something\","
        +   "\"type\":\"SemanticLidar\","
        +   "\"orientation\":"
        +   "{"
        +       "\"yaw\":23.0,"
        +       "\"pitch\":0.0,"
        +       "\"roll\":0.0"
        +   "},"
        +   "\"location\":"
        +   "{"
        +       "\"x\":1.0,"
        +       "\"y\":2.0,"
        +       "\"z\":3.0"
        +   "}"
        +"}";
        Gson gson = new Gson();
        Sensor predictedSensor = new Sensor("something", SensorType.SEMANTIC_LIDAR, new Orientation(23.0, 0, 0),
                CartesianPoint.xyz(1, 2, 3));
        Sensor sensor = gson.fromJson(predictedSensorJson, Sensor.class);
        assertEquals(sensor, predictedSensor);
    }
}
