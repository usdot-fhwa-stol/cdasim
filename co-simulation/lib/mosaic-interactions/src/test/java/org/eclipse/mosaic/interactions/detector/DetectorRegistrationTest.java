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
        detectorRegistration = new DetectorRegistration(0, null);
    }

    @Test
    public void testGetterSetterConstructor() {
        Detector detector = new Detector("something", DetectorType.SEMANTIC_LIDAR, new Orientation(23.0, 0, 0),
                CartesianPoint.xyz(1, 2, 3));
        detectorRegistration.setDetector(detector);
        assertEquals(detector, detectorRegistration.getDetector());

        DetectorRegistration detectorRegistration1 = new DetectorRegistration(0, detector);

        assertNotEquals(detectorRegistration, detectorRegistration1);
        
    }
}
