package org.eclipse.mosaic.lib.trafficevent;

import org.junit.Test;
import static org.junit.Assert.*;

import org.eclipse.mosaic.lib.objects.trafficevent.MsgerTrafficEvent;

public class MsgerTrafficEventTest {

    @Test
    public void testConstructorAndGetters() {
        String vehicleId = "vehicle123";
        float upTrack = 100.5f;
        float downTrack = 200.5f;
        float minimumGap = 5.0f;
        float advisorySpeed = 45.0f;

        MsgerTrafficEvent event = new MsgerTrafficEvent(vehicleId, upTrack, downTrack, minimumGap, advisorySpeed);

        // Validate constructor and getters
        assertEquals(vehicleId, event.getVehicleId());
        assertEquals(upTrack, event.getUpTrack(), 0.001);
        assertEquals(downTrack, event.getDownTrack(), 0.001);
        assertEquals(minimumGap, event.getMinimumGap(), 0.001);
        assertEquals(advisorySpeed, event.getAdvisorySpeed(), 0.001);
    }

    @Test
    public void testEqualsAndHashCode() {
        MsgerTrafficEvent event1 = new MsgerTrafficEvent("vehicle123", 100.5f, 200.5f, 5.0f, 45.0f);
        MsgerTrafficEvent event2 = new MsgerTrafficEvent("vehicle123", 100.5f, 200.5f, 5.0f, 45.0f);
        MsgerTrafficEvent event3 = new MsgerTrafficEvent("vehicle456", 50.0f, 150.0f, 3.0f, 30.0f);

        // Test equality
        assertEquals(event1, event2);
        assertNotEquals(event1, event3);

        // Test hash codes
        assertEquals(event1.hashCode(), event2.hashCode());
        assertNotEquals(event1.hashCode(), event3.hashCode());
    }

    @Test
    public void testToString() {
        MsgerTrafficEvent event = new MsgerTrafficEvent("vehicle123", 100.5f, 200.5f, 5.0f, 45.0f);

        // Verify toString output
        String expectedString = "MsgerTrafficEvent[vehicleId=vehicle123,upTrack=100.5,downTrack=200.5,minimumGap=5.0,advisorySpeed=45.0]";
        assertTrue(event.toString().contains("vehicleId=vehicle123"));
        assertTrue(event.toString().contains("upTrack=100.5"));
        assertTrue(event.toString().contains("downTrack=200.5"));
        assertTrue(event.toString().contains("minimumGap=5.0"));
        assertTrue(event.toString().contains("advisorySpeed=45.0"));
    }
}
