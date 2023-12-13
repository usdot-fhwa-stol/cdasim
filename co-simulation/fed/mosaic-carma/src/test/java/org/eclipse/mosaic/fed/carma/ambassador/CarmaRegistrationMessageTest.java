package org.eclipse.mosaic.fed.carma.ambassador;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class CarmaRegistrationMessageTest {
    @Test
    public void testGetterSettersConstructor() {

        CarmaRegistrationMessage message = new CarmaRegistrationMessage(
                "ID",
                "role",
                "127.0.0.1",
                5678,
                8642);
        // Test Getter
        assertEquals("ID", message.getCarmaVehicleId());
        assertEquals("role", message.getCarlaVehicleRole());

        assertEquals("127.0.0.1", message.getRxMessageIpAddress());
        assertEquals(5678, message.getRxMessagePort());
        assertEquals(8642, message.getRxTimeSyncPort());

    }

}
