package org.eclipse.mosaic.fed.infrastructure.ambassador;

import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.mock;

import java.net.InetAddress;

import org.eclipse.mosaic.lib.geo.GeoPoint;
import org.junit.Before;
import org.junit.Test;
import org.mockito.Mockito;

public class InfrastructureInstanceManagerTest {

    private InfrastructureInstanceManager manager;
    private InfrastructureRegistrationMessage registration;
    private InetAddress ipAddress;
    private GeoPoint location;

    @Before
    public void setUp() throws Exception {
        manager = new InfrastructureInstanceManager();
        registration = mock(InfrastructureRegistrationMessage.class);
        ipAddress = mock(InetAddress.class);
        location = mock(GeoPoint.class);
    }

    @Test
    public void testOnNewRegistration() {
        // Set up the registration object
        String infrastructureId = "infrastructure-123";
        int rxMessagePort = 1234;
        int timeSyncPort = 5678;
        String ipAddressString = "127.0.0.1";
        GeoPoint pt = GeoPoint.latLon(37.3382, -121.8863);

        // Mock the behavior of the registration object
        mockRegistrationObject(infrastructureId, rxMessagePort, timeSyncPort, ipAddressString, pt);

        // Call the onNewRegistration method with the mocked registration object
        manager.onNewRegistration(registration);

        // Verify that the infrastructure instance was added to the manager
        assertEquals(true, manager.checkIfRegistered(infrastructureId));
    }

    private void mockRegistrationObject(String infrastructureId, int rxMessagePort, int timeSyncPort,
            String ipAddressString, GeoPoint pt) {
        // Mock the behavior of the registration object
        Mockito.when(registration.getInfrastructureId()).thenReturn(infrastructureId);
        Mockito.when(registration.getRxMessagePort()).thenReturn(rxMessagePort);
        Mockito.when(registration.getTimeSyncPort()).thenReturn(timeSyncPort);
        Mockito.when(registration.getRxMessageIpAddress()).thenReturn(ipAddressString);
        Mockito.when(registration.getLocation()).thenReturn(pt);
        Mockito.when(ipAddress.getHostAddress()).thenReturn(ipAddressString);
    }
}
