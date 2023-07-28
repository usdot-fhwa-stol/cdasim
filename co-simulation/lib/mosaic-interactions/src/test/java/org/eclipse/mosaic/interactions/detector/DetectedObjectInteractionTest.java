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
                "Object7",
                CartesianPoint.xyz(-1.1, -2, -3.2),
                new Vector3d(1, 1, 1),
                new Vector3d(.1, .2, .3),
                new Size(2, 1, .5));
        Double[] covarianceMatrix = new Double[] { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
        detectedObject.setPositionCovariance(covarianceMatrix);
        detectedObject.setVelocityCovariance(covarianceMatrix);
        detectedObject.setAngularVelocityCovariance(covarianceMatrix);
        
        interaction.setDetectedObject(detectedObject);

        assertEquals(detectedObject, interaction.getDetectedObject());
        
        DetectedObjectInteraction interaction2 = new DetectedObjectInteraction(0, detectedObject);

        assertNotEquals(interaction, interaction2);
    }
}
