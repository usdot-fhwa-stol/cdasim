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
import org.eclipse.mosaic.lib.objects.detector.Detector;
import org.eclipse.mosaic.lib.objects.detector.DetectorType;
import org.eclipse.mosaic.lib.objects.detector.Orientation;
import org.junit.Before;
import org.junit.Test;

public class DetectorRegistrationTest {
    DetectorRegistration detectorRegistration;
    @Before
    public void setUp() throws Exception {
        detectorRegistration = new DetectorRegistration(0, null, "");
    }

    @Test
    public void testGetterSetterConstructor() {
        Detector detector = new Detector("something", DetectorType.SEMANTIC_LIDAR, new Orientation(23.0, 0, 0),
                CartesianPoint.xyz(1, 2, 3));
        detectorRegistration.setDetector(detector);
        assertEquals(detector, detectorRegistration.getDetector());

        DetectorRegistration detectorRegistration1 = new DetectorRegistration(0, detector, "rsu_1");

        assertNotEquals(detectorRegistration, detectorRegistration1);
        DetectorRegistration detectorRegistration2 = detectorRegistration1;
        assertEquals(detectorRegistration1, detectorRegistration2);
        assertEquals(detectorRegistration1.hashCode(), detectorRegistration2.hashCode());

    }
}
