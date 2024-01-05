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
package org.eclipse.mosaic.interactions.detector;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;

import org.eclipse.mosaic.lib.geo.CartesianPoint;
import org.eclipse.mosaic.lib.math.Vector3d;
import org.eclipse.mosaic.lib.objects.detector.DetectedObject;
import org.eclipse.mosaic.lib.objects.detector.DetectionType;
import org.eclipse.mosaic.lib.objects.detector.Size;
import org.junit.Before;
import org.junit.Test;

public class DetectedObjectInteractionTest {

    private DetectedObjectInteraction interaction;
    @Before
    public void setUp() throws Exception {
        interaction = new DetectedObjectInteraction(0, null);
    }

    @Test
    public void testGetterSetterConstructor() {
        DetectedObject detectedObject = new DetectedObject(
                DetectionType.CAR,
                0.7,
                "sensor2",
                "projection String2",
                100,
                CartesianPoint.xyz(-1.1, -2, -3.2),
                new Vector3d(1, 1, 1),
                new Vector3d(.1, .2, .3),
                new Size(2, 1, .5),
                100);
        Double[][] covarianceMatrix =  { {0.0, 0.0, 0.0} , {0.0, 0.0, 0.0} , {0.0, 0.0, 0.0}};
        detectedObject.setPositionCovariance(covarianceMatrix);
        detectedObject.setVelocityCovariance(covarianceMatrix);
        detectedObject.setAngularVelocityCovariance(covarianceMatrix);
        
        interaction.setDetectedObject(detectedObject);

        assertEquals(detectedObject, interaction.getDetectedObject());
        
        DetectedObjectInteraction interaction2 = new DetectedObjectInteraction(0, detectedObject);
        DetectedObjectInteraction interaction3 = interaction2;
        assertNotEquals(interaction, interaction2);
        assertEquals(interaction2, interaction3);
        assertEquals(interaction2.hashCode(), interaction3.hashCode());
    }
}
