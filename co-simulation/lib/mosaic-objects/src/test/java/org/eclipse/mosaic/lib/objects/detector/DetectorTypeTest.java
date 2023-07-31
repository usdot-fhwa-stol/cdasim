package org.eclipse.mosaic.lib.objects.detector;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class DetectorTypeTest {
    @Test
    public void testFromLabel() {
        DetectorType type = DetectorType.fromLabel("SemanticLidar");
        assertEquals( DetectorType.SEMANTIC_LIDAR, type);
        Exception exception = assertThrows(IllegalArgumentException.class, () -> {
            DetectorType.fromLabel("NotSupportedDetected");
        });
       assertTrue(exception.getMessage().contains("Unknown DetectorType label"));
    }
}
