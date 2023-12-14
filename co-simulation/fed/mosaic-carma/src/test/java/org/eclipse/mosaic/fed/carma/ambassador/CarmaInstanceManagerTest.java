package org.eclipse.mosaic.fed.carma.ambassador;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyByte;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.verify;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import org.junit.Before;
import org.junit.Test;
import org.mockito.internal.util.reflection.FieldSetter;

import com.google.gson.Gson;

import gov.dot.fhwa.saxton.TimeSyncMessage;

public class CarmaInstanceManagerTest {
    private CarmaInstanceManager manager;
    private CarmaInstance instance1;
    private CarmaInstance instance2;
    private CarmaInstance instance3;

    @Before
    public void setUp() throws Exception {
        manager = new CarmaInstanceManager();
        instance1 = mock(CarmaInstance.class);
        instance2 = mock(CarmaInstance.class);
        instance3 = mock(CarmaInstance.class);
        Map<String, CarmaInstance>  managedInstances = new HashMap<>();
        managedInstances.put("instance1", instance1);
        managedInstances.put("instance2", instance2);
        managedInstances.put("instance3", instance3);
      
        // Set private instance field to mock using reflection
        FieldSetter.setField(manager, manager.getClass().getDeclaredField("managedInstances"), managedInstances);
    }

    @Test
    public void testOnNewRegistration() {
        // Set up the registration object
        String infrastructureId = "infrastructure-123";
        int rxMessagePort = 1234;
        int timeSyncPort = 5678;
        String ipAddressString = "127.0.0.1";
      
        // Mock the behavior of the registration object
        CarmaRegistrationMessage registration = new CarmaRegistrationMessage(
                                                                            infrastructureId, 
                                                                            infrastructureId,
                                                                            ipAddressString, 
                                                                            rxMessagePort, 
                                                                            timeSyncPort);
        // Ensure checkIfRegistered returns false for infrastructure ID before registering 
        assertFalse( manager.checkIfRegistered(infrastructureId) );

        // Call the onNewRegistration method with the mocked registration object
        manager.onNewRegistration(registration);

        // Verify that the infrastructure instance was added to the manager
        assertTrue( manager.checkIfRegistered(infrastructureId) );
        // Ensure checkIfRegistered returns false for other Ids
        assertFalse( manager.checkIfRegistered(infrastructureId + "something") );
    }

    @Test
    public void testOnTimeStepUpdate() throws IOException {
        TimeSyncMessage message = new TimeSyncMessage();
    
        message.setSeq(3);
        message.setTimestep(300);
        Gson gson = new Gson();
        byte[] message_bytes = gson.toJson(message).getBytes();

        manager.onTimeStepUpdate(message);
        // Verify that all instances sendTimeSyncMsgs was called.
        verify(instance1).sendTimeSyncMsg(message_bytes);
        verify(instance2).sendTimeSyncMsg(message_bytes);
        verify(instance3).sendTimeSyncMsg(message_bytes);

    }

    public void testOnTimeStepUpdateWithoutRegisteredIntstances() throws NoSuchFieldException, SecurityException, IOException{
        // Verify that with no managed instances nothing is called and no exception is thrown.
        Map<String, CarmaInstance>  managedInstances = new HashMap<>();
      
        // Set private instance field to mock using reflection
        FieldSetter.setField(manager, manager.getClass().getDeclaredField("managedInstances"), managedInstances);
        TimeSyncMessage message = new TimeSyncMessage();
    
        message.setSeq(3);
        message.setTimestep(300);
        assertDoesNotThrow(manager.onTimeStepUpdate(message));
        
        verify(instance1, never()).sendTimeSyncMsg(any());
        verify(instance2, never()).sendTimeSyncMsg(any());
        verify(instance3, never()).sendTimeSyncMsg(any());

    }

}
