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

import java.util.HashMap;
import java.util.Map;

import org.eclipse.mosaic.lib.junit.IpResolverRule;
import org.eclipse.mosaic.lib.objects.addressing.IpResolver;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import static org.mockito.Mockito.mock;
import org.mockito.internal.util.reflection.FieldSetter;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

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
        FieldSetter.setField(manager, manager.getClass().getSuperclass().getDeclaredField("managedInstances"), managedInstances);
    }

    @Test
    public void testOnNewRegistration() {
        // Set up the registration object
        String infrastructureId = "infrastructure-123";
        int rxMessagePort = 1234;
        int timeSyncPort = 5678;
        String ipAddressString = "127.0.0.1";
        String emergencyState = "MockState";
        int rxBridgeMessagePort = 5600;
      
        // Mock the behavior of the registration object
        CarmaMessengerRegistrationMessage registration = new CarmaMessengerRegistrationMessage(
                                                                            infrastructureId, 
                                                                            infrastructureId,
                                                                            ipAddressString, 
                                                                            rxMessagePort, 
                                                                            timeSyncPort,
                                                                            emergencyState,
                                                                            rxBridgeMessagePort);
        // Ensure checkIfRegistered returns false for infrastructure ID before registering 
        assertFalse( manager.checkIfRegistered(infrastructureId) );

        // Call the onNewRegistration method with the mocked registration object
        manager.onNewRegistration(registration);

        // Verify that the infrastructure instance was added to the manager
        assertTrue( manager.checkIfRegistered(infrastructureId) );
        // Ensure checkIfRegistered returns false for other Ids
        assertFalse( manager.checkIfRegistered(infrastructureId + "something") );
    }

}
