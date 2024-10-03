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
package org.eclipse.mosaic.fed.carmamessenger.ambassador;

import java.io.IOException;
import java.net.InetAddress;
import java.util.HashMap;
import java.util.Map;

import org.eclipse.mosaic.interactions.communication.V2xMessageTransmission;
import org.eclipse.mosaic.lib.geo.GeoPoint;
import org.eclipse.mosaic.lib.junit.IpResolverRule;
import org.eclipse.mosaic.lib.objects.addressing.IpResolver;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;
import org.mockito.internal.util.reflection.FieldSetter;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.gson.Gson;

import gov.dot.fhwa.saxton.CarmaV2xMessage;
import gov.dot.fhwa.saxton.TimeSyncMessage;

public class CarmaMessengerInstanceManagerTest {
    private CarmaMessengerInstanceManager manager;
    private CarmaMessengerInstance instance1;
    private CarmaMessengerInstance instance2;
    private CarmaMessengerInstance instance3;
    private final String sampleMessage =
    "Version=0.7\n" +
    "Type=BSM\n" +
    "PSID=0020\n" +
    "Priority=6\n" +
    "TxMode=ALT\n" +
    "TxChannel=172\n" +
    "TxInterval=0\n" +
    "DeliveryStart=\n" +
    "DeliveryStop=\n" +
    "Signature=False\n" +
    "Encryption=False\n" +
    "Payload=00142500400000000f0e35a4e900eb49d20000007fffffff8ffff080fdfa1fa1007fff8000960fa0\n";
    private final Logger log = LoggerFactory.getLogger(this.getClass());

    /**
     * Rule to initialize {@link IpResolver} Singleton.
     */
    @Rule
    public IpResolverRule ipResolverRule = new IpResolverRule();

    @Before
    public void setUp() throws Exception {
        manager = new CarmaMessengerInstanceManager();
        instance1 = mock(CarmaMessengerInstance.class);
        instance2 = mock(CarmaMessengerInstance.class);
        instance3 = mock(CarmaMessengerInstance.class);
        Map<String, CarmaMessengerInstance>  managedInstances = new HashMap<>();
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
        String emergencyState = "MockState";
      
        // Mock the behavior of the registration object
        CarmaMessengerRegistrationMessage registration = new CarmaMessengerRegistrationMessage(
                                                                            infrastructureId, 
                                                                            infrastructureId,
                                                                            ipAddressString, 
                                                                            rxMessagePort, 
                                                                            timeSyncPort,
                                                                            emergencyState);
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
        TimeSyncMessage message = new TimeSyncMessage(300, 3);
    
        Gson gson = new Gson();
        byte[] message_bytes = gson.toJson(message).getBytes();

        manager.onTimeStepUpdate(message);
        // Verify that all instances sendTimeSyncMsgs was called.
        verify(instance1).sendTimeSyncMsg(message_bytes);
        verify(instance2).sendTimeSyncMsg(message_bytes);
        verify(instance3).sendTimeSyncMsg(message_bytes);

    }
    @Test
    public void testOnTimeStepUpdateWithoutRegisteredIntstances() throws NoSuchFieldException, SecurityException, IOException{
        // Verify that with no managed instances nothing is called and no exception is thrown.
        Map<String, CarmaMessengerInstance>  managedInstances = new HashMap<>();
      
        // Set private instance field to mock using reflection
        FieldSetter.setField(manager, manager.getClass().getDeclaredField("managedInstances"), managedInstances);
        TimeSyncMessage message = new TimeSyncMessage(300, 3);
    
        manager.onTimeStepUpdate(message);
        
        verify(instance1, never()).sendTimeSyncMsg(any());
        verify(instance2, never()).sendTimeSyncMsg(any());
        verify(instance3, never()).sendTimeSyncMsg(any());

    }
    @Test
    public void testonV2XMessageTx() {
        CarmaV2xMessage message = new CarmaV2xMessage(sampleMessage.getBytes());
        // Setup mock addresses for registered carma platform instances
        InetAddress address1 = mock(InetAddress.class);
        InetAddress address2 = mock(InetAddress.class);
        InetAddress address3 = mock(InetAddress.class);
        when(instance1.getTargetAddress()).thenReturn(address1);
        when(instance2.getTargetAddress()).thenReturn(address2);
        when(instance3.getTargetAddress()).thenReturn(address3);
        // Register host with IpResolver singleton
        IpResolver.getSingleton().registerHost("veh_0");
        // Set CarlaRoleName to veh_0 to macth registered host
        when(instance1.getCarlaRoleName()).thenReturn("veh_0");
        // Set location to origin
        when(instance1.getLocation()).thenReturn(GeoPoint.ORIGO);

        V2xMessageTransmission messageTx = manager.onV2XMessageTx(address1, message, 1000);
        assertEquals(1000, messageTx.getTime());
        assertEquals(GeoPoint.ORIGO, messageTx.getSourcePosition());
        assertEquals("veh_0", messageTx.getMessage().getRouting().getSource().getSourceName());
    }

    @Test(expected = IllegalStateException.class)
    public void testonV2XMessageTxUnregisteredCarmaPlatform() {
        CarmaV2xMessage message = new CarmaV2xMessage(sampleMessage.getBytes());
        InetAddress address1 = mock(InetAddress.class);
        InetAddress address2 = mock(InetAddress.class);
        InetAddress address3 = mock(InetAddress.class);
        InetAddress unregisteredAddress = mock(InetAddress.class);

        when(instance1.getTargetAddress()).thenReturn(address1);
        when(instance2.getTargetAddress()).thenReturn(address2);
        when(instance3.getTargetAddress()).thenReturn(address3);
        IpResolver.getSingleton().registerHost("veh_0");
        when(instance1.getCarlaRoleName()).thenReturn("veh_0");
        when(instance1.getLocation()).thenReturn(GeoPoint.ORIGO);
        // Attempt to create V2X Message Transmission for unregistered address.
        // Throws IllegalStateException
        manager.onV2XMessageTx(unregisteredAddress, message, 1000);
        
    }
}
