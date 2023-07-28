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
import org.eclipse.mosaic.lib.math.Vector3d;
import org.eclipse.mosaic.lib.objects.detector.DetectedObject;
import org.eclipse.mosaic.lib.objects.detector.DetectionType;
import org.eclipse.mosaic.lib.objects.detector.Size;
import org.junit.Before;
import org.junit.Test;

import com.google.gson.Gson;

public class DetectedObjectInteractionTest {
    @Before
    public void setUp() throws Exception {

    }

    @Test
    public void testDetectObjectJsonSerialization() {
        // Set up the registration object
        DetectedObject detectedObject = new DetectedObject(
                DetectionType.BUS,
                0.5,
                "sensor1",
                "projection String",
                "Object1",
                CartesianPoint.xyz(1.1, 2, 3.2),
                new Vector3d(0, 0, 0),
                new Vector3d(),
                new Size(0, 0, 0));
        Double[] covarianceMatrix = new Double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        detectedObject.setPositionCovariance(covarianceMatrix);
        detectedObject.setVelocityCovariance(covarianceMatrix);
        detectedObject.setAngularVelocityCovariance(covarianceMatrix);
        Gson gson = new Gson();
        String json = gson.toJson(detectedObject);
        String json_prediction = "{"
                + "\"type\":\"BUS\","
                + "\"confidence\":0.5,"
                + "\"sensorId\":\"sensor1\","
                + "\"projString\":\"projection String\","
                + "\"objectId\":\"Object1\","
                + "\"position\":"
                + "{"
                + "\"x\":1.1,"
                + "\"y\":2.0,"
                + "\"z\":3.2"
                + "},"
                + "\"positionCovariance\":[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],"
                + "\"velocity\":"
                + "{"
                + "\"x\":0.0,"
                + "\"y\":0.0,"
                + "\"z\":0.0"
                + "},"
                + "\"velocityCovariance\":[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],"
                + "\"angularVelocity\":"
                + "{"
                + "\"x\":0.0,"
                + "\"y\":0.0,"
                + "\"z\":0.0"
                + "},"
                + "\"angularVelocityCovariance\":[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],"
                + "\"size\":"
                + "{"
                + "\"length\":0.0,"
                + "\"height\":0.0,"
                + "\"width\":0.0"
                + "}"
                + "}";
        assertEquals(json,
                json_prediction);

    }

    @Test
    public void testDetectObjectJsonDeserialization() {
        Gson gson = new Gson();
        String json = "{"
                + "\"type\":\"CAR\","
                + "\"confidence\":0.7,"
                + "\"sensorId\":\"sensor2\","
                + "\"projString\":\"projection String2\","
                + "\"objectId\":\"Object7\","
                + "\"position\":"
                + "{"
                + "\"x\":-1.1,"
                + "\"y\":-2.0,"
                + "\"z\":-3.2"
                + "},"
                + "\"positionCovariance\":[1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0],"
                + "\"velocity\":"
                + "{"
                + "\"x\":1,"
                + "\"y\":1,"
                + "\"z\":1"
                + "},"
                + "\"velocityCovariance\":[1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0],"
                + "\"angularVelocity\":"
                + "{"
                + "\"x\":0.1,"
                + "\"y\":0.2,"
                + "\"z\":0.3"
                + "},"
                + "\"angularVelocityCovariance\":[1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0],"
                + "\"size\":"
                + "{"
                + "\"length\":2.0,"
                + "\"height\":1.0,"
                + "\"width\":0.5"
                + "}"
                + "}";

        DetectedObject detectedObject = gson.fromJson(json, DetectedObject.class);
        DetectedObject predictedDetectedObject = new DetectedObject(
                DetectionType.CAR,
                0.7,
                "sensor2",
                "projection String2",
                "Object7",
                CartesianPoint.xyz(-1.1, -2, -3.2),
                new Vector3d(1, 1, 1),
                new Vector3d(.1, .2, .3),
                new Size(2, 1, .5));
        Double[] covarianceMatrix = new Double[] { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
        predictedDetectedObject.setPositionCovariance(covarianceMatrix);
        predictedDetectedObject.setVelocityCovariance(covarianceMatrix);
        predictedDetectedObject.setAngularVelocityCovariance(covarianceMatrix);
        assertEquals(detectedObject, predictedDetectedObject);
    }

}
