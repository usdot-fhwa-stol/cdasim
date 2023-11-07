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

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;

import org.eclipse.mosaic.lib.geo.CartesianPoint;
import org.eclipse.mosaic.lib.math.Vector3d;
import org.junit.Before;
import org.junit.Test;

import com.google.gson.Gson;

public class DetectedObjectTest {
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
        Double[][] covarianceMatrix =  { {0.0, 0.0, 0.0} , {0.0, 0.0, 0.0} , {0.0, 0.0, 0.0}};
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
                + "\"positionCovariance\":[[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]],"
                + "\"velocity\":"
                + "{"
                + "\"x\":0.0,"
                + "\"y\":0.0,"
                + "\"z\":0.0"
                + "},"
                + "\"velocityCovariance\":[[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]],"
                + "\"angularVelocity\":"
                + "{"
                + "\"x\":0.0,"
                + "\"y\":0.0,"
                + "\"z\":0.0"
                + "},"
                + "\"angularVelocityCovariance\":[[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]],"
                + "\"size\":"
                + "{"
                + "\"length\":0.0,"
                + "\"height\":0.0,"
                + "\"width\":0.0"
                + "}"
                + "}";
        assertEquals(json,
                json_prediction);
        System.out.println(json);
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
                + "\"positionCovariance\":[[1.0,0.0,0.0],[1.0,0.0,0.0],[1.0,0.0,0.0]],"
                + "\"velocity\":"
                + "{"
                + "\"x\":1,"
                + "\"y\":1,"
                + "\"z\":1"
                + "},"
                + "\"velocityCovariance\":[[1.0,0.0,0.0],[1.0,0.0,0.0],[1.0,0.0,0.0]],"
                + "\"angularVelocity\":"
                + "{"
                + "\"x\":0.1,"
                + "\"y\":0.2,"
                + "\"z\":0.3"
                + "},"
                + "\"angularVelocityCovariance\":[[1.0,0.0,0.0],[1.0,0.0,0.0],[1.0,0.0,0.0]],"
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
        Double[][] covarianceMatrix =  { {1.0, 0.0, 0.0} , {1.0, 0.0, 0.0} , {1.0, 0.0, 0.0}};
        predictedDetectedObject.setPositionCovariance(covarianceMatrix);
        predictedDetectedObject.setVelocityCovariance(covarianceMatrix);
        predictedDetectedObject.setAngularVelocityCovariance(covarianceMatrix);
        assertEquals(detectedObject, predictedDetectedObject);
        assertEquals(detectedObject.hashCode(), predictedDetectedObject.hashCode());
    }

    @Test
    public void testGetterSetterConstructor() {
        //Test constructor
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
        Double[][] covarianceMatrix =  { {0.0, 0.0, 0.0} , {0.0, 0.0, 0.0} , {0.0, 0.0, 0.0}};
        detectedObject.setPositionCovariance(covarianceMatrix);
        detectedObject.setVelocityCovariance(covarianceMatrix);
        detectedObject.setAngularVelocityCovariance(covarianceMatrix);
        // Test Getters
        assertEquals(DetectionType.VAN, detectedObject.getType());
        assertEquals(0.5, detectedObject.getConfidence(), .01);
        assertEquals("sensor1", detectedObject.getSensorId());
        assertEquals("projection String", detectedObject.getProjString());
        assertEquals("Object1", detectedObject.getObjectId());
        assertEquals(CartesianPoint.xyz(1.1, 2, 3.2), detectedObject.getPosition());
        assertEquals(new Vector3d(2, 3, 4), detectedObject.getVelocity());
        assertEquals(new Vector3d(-4.4, -5.5, -6.6), detectedObject.getAngularVelocity());
        assertEquals( new Size(3,4,5), detectedObject.getSize());
        assertArrayEquals(covarianceMatrix, detectedObject.getPositionCovariance());
        assertArrayEquals(covarianceMatrix, detectedObject.getVelocityCovariance());
        assertArrayEquals(covarianceMatrix, detectedObject.getAngularVelocityCovariance());

       
    }

    @Test
    public void testEquals() {
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
        Double[][] covarianceMatrix =  { {0.0, 0.0, 0.0} , {0.0, 0.0, 0.0} , {0.0, 0.0, 0.0}};
        detectedObject.setPositionCovariance(covarianceMatrix);
        detectedObject.setVelocityCovariance(covarianceMatrix);
        detectedObject.setAngularVelocityCovariance(covarianceMatrix);

        DetectedObject detectedObject1 = new DetectedObject(
                null,
                0.6,
                null,
                null,
                null,
                null,
                null,
                null,
                null);
        DetectedObject detectedObject2 = new DetectedObject(
                null,
                0.6,
                null,
                null,
                null,
                null,
                null,
                null,
                null);

        assertNotEquals(detectedObject1, detectedObject);
        assertEquals(detectedObject2, detectedObject1);


        //Correct Type
        detectedObject1.setType(detectedObject.getType());
        assertNotEquals(detectedObject1, detectedObject);
        //Correct Confidence
        detectedObject1.setConfidence(detectedObject.getConfidence());
        assertNotEquals(detectedObject1, detectedObject);
        //Correct Sensor ID
        detectedObject1.setSensorId(detectedObject.getSensorId());
        assertNotEquals(detectedObject1, detectedObject);
        //Correct Projection String
        detectedObject1.setProjString(detectedObject.getProjString());
        assertNotEquals(detectedObject1, detectedObject);
        //Correct Object ID
        detectedObject1.setObjectId(detectedObject.getObjectId());
        assertNotEquals(detectedObject1, detectedObject);
        //Correct Size
        detectedObject1.setSize(detectedObject.getSize());
        assertNotEquals(detectedObject1, detectedObject);
        //Correct Position 
        detectedObject1.setPosition(detectedObject.getPosition());
        assertNotEquals(detectedObject1, detectedObject);
        //Correct Position Covariance
        detectedObject1.setPositionCovariance(detectedObject.getPositionCovariance());
        assertNotEquals(detectedObject1, detectedObject);
        //Correct Velocity 
        detectedObject1.setVelocity(detectedObject.getVelocity());
        assertNotEquals(detectedObject1, detectedObject);
        //Correct Velocity Covariance
        detectedObject1.setVelocityCovariance(detectedObject.getVelocityCovariance());
        assertNotEquals(detectedObject1, detectedObject);
        //Correct Angular Velocity 
        detectedObject1.setAngularVelocity(detectedObject.getAngularVelocity());
        assertNotEquals(detectedObject1, detectedObject);
        //Correct Angular Velocity Covariance
        detectedObject1.setAngularVelocityCovariance(detectedObject.getAngularVelocityCovariance());
        assertEquals(detectedObject1, detectedObject);

        


        

        

        

        

        

        

       

        

    }

}
