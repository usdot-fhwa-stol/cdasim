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
 
package org.eclipse.mosaic.fed.carmacloud.ambassador;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

import java.io.IOException;
import java.util.Map;
import org.junit.Before;
import org.junit.Test;

import gov.dot.fhwa.saxton.TimeSyncMessage;



public class CarmaCloudInstanceManagerTest {

    private CarmaCloudInstanceManager manager;
    private CarmaCloudInstance instance1;
    private static final String carmacloudId = "carma-cloud";
    private static final String carmacloudUrl = "carma-cloud-url";

    @Before
    public void setUp() throws Exception {
        manager = new CarmaCloudInstanceManager();
        instance1 = mock(CarmaCloudInstance.class);
    }

    @Test
    public void testOnNewRegistration() {
        // Set up the registration object
        CarmaCloudRegistrationMessage registration = new CarmaCloudRegistrationMessage(carmacloudId, carmacloudUrl);

        // Call the onNewRegistration method with the mocked registration object
        manager.onNewRegistration(registration);

        // Verify that the infrastructure instance was added to the manager
        assertTrue(manager.checkIfRegistered(carmacloudId));
        // Ensure checkIfRegistered returns false for other Ids
        assertFalse(manager.checkIfRegistered(carmacloudId + "something") );
    }

    @Test
    public void testOnTimeStepUpdate() throws IOException {
        // replace registered CARMA Cloud instance with mock instance
        manager.getManagedInstances().put(carmacloudId, instance1);

        TimeSyncMessage message = new TimeSyncMessage(999L, 11);
        manager.onTimeStepUpdate(message);
        // Verify that all instances sendTimeSyncMsgs was called.
        verify(instance1).sendTimeSyncMsg(message);
    }
}
