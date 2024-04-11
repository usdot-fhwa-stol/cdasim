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

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.junit.Before;
import org.junit.Test;


/**
 * Tests for {@link CarmaCloudMessageAmbassador}.
 */
public class CarmaCloudMessageAmbassadorTest {

    private CarmaCloudMessageAmbassador ambassador;
    /**
     * {@link CarmaCloudInstanceManager} mock.
     */
    private CarmaCloudInstanceManager instanceManagerMock;
    /**
     * {@link CarmaCloudRegistrationReceiver} mock.
     */
    private CarmaCloudRegistrationReceiver receiverMock;

    private ArrayList<CarmaCloudRegistrationMessage> registrationMessages;

    @Before
    public void setUp() throws IOException, NoSuchFieldException, InternalFederateException, IllegalValueException {
        // Initialize Mocks
        instanceManagerMock = mock(CarmaCloudInstanceManager.class);
        receiverMock = mock(CarmaCloudRegistrationReceiver.class);

        CarmaCloudRegistrationMessage message = new CarmaCloudRegistrationMessage("", "");
        registrationMessages = new ArrayList<>();
        registrationMessages.add(message);
    }

    @Test
    public void testInitialize() throws InternalFederateException, IllegalValueException {
        assertTrue(true);
    }

    @Test
    public void testProcessInteraction() throws InternalFederateException {
        assertTrue(true);
    }

    @Test
    public void testProcessTimeAdvanceGrant() throws InternalFederateException, IllegalValueException, NoSuchFieldException, SecurityException
    {
        assertTrue(true);
    }
}
