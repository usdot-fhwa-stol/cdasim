package org.eclipse.mosaic.interactions.msger;

import org.eclipse.mosaic.interactions.application.MsgerResponseTrafficEvent;
import org.eclipse.mosaic.lib.objects.trafficevent.MsgerTrafficEvent;
import org.junit.Test;
import static org.junit.Assert.*;

public class MsgerResponseTrafficEventTest {

    @Test
    public void testConstructorAndGetters() {
        long time = 100L;
        MsgerTrafficEvent trafficEvent = new MsgerTrafficEvent("vehicle123", 100.5f, 200.5f, 5.0f, 45.0f);

        MsgerResponseTrafficEvent response = new MsgerResponseTrafficEvent(time, trafficEvent);

        // Validate constructor and getters
        assertEquals(time, response.getTime());
        assertEquals(trafficEvent, response.getTrafficEvent());
    }

    @Test
    public void testToString() {
        MsgerTrafficEvent trafficEvent = new MsgerTrafficEvent("vehicle123", 100.5f, 200.5f, 5.0f, 45.0f);
        MsgerResponseTrafficEvent response = new MsgerResponseTrafficEvent(100L, trafficEvent);

        // Verify toString output
        String toStringOutput = response.toString();
        assertTrue(toStringOutput.contains("upTrack=100.5"));
        assertTrue(toStringOutput.contains("downTrack=200.5"));
        assertTrue(toStringOutput.contains("minimumGap=5.0"));
        assertTrue(toStringOutput.contains("advisorySpeed=45.0"));
    }
}
