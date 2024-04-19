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

import org.eclipse.mosaic.lib.util.junit.TestFileRule;
import org.eclipse.mosaic.rti.api.IllegalValueException;
import org.eclipse.mosaic.rti.api.InternalFederateException;
import org.eclipse.mosaic.rti.api.Interaction;
import org.eclipse.mosaic.rti.api.RtiAmbassador;
import org.eclipse.mosaic.rti.api.parameters.AmbassadorParameter;
import org.eclipse.mosaic.rti.api.parameters.FederateDescriptor;
import org.eclipse.mosaic.rti.config.CLocalHost;
import org.eclipse.mosaic.rti.TIME;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;
import org.mockito.internal.util.reflection.FieldSetter;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.RuleChain;
import org.junit.rules.TemporaryFolder;


/**
 * Tests for {@link CarmaCloudMessageAmbassador}.
 */
public class CarmaCloudMessageAmbassadorTest {

    private final TemporaryFolder temporaryFolder = new TemporaryFolder();

    private final TestFileRule testFileRule = new TestFileRule(temporaryFolder).basedir("carmacloud");

    @Rule
    public RuleChain chain = RuleChain.outerRule(temporaryFolder).around(testFileRule);

    /**
     * {@link RtiAmbassador} mock.
     */
    private RtiAmbassador rtiMock;

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
        rtiMock = mock(RtiAmbassador.class);
        FederateDescriptor handleMock = mock(FederateDescriptor.class);
        instanceManagerMock = mock(CarmaCloudInstanceManager.class);
        receiverMock = mock(CarmaCloudRegistrationReceiver.class);
        File workingDir = temporaryFolder.getRoot();
        CLocalHost testHostConfig = new CLocalHost();
        testHostConfig.workingDirectory = workingDir.getAbsolutePath();

        // Mock methods     
        when(handleMock.getHost()).thenReturn(testHostConfig);
        when(handleMock.getId()).thenReturn("carmacloud");
        CarmaCloudRegistrationMessage message = new CarmaCloudRegistrationMessage("carmacloud-id", "carmacloud-url");
        registrationMessages = new ArrayList<>();
        registrationMessages.add(message);
        when(receiverMock.getReceivedMessages()).thenReturn(registrationMessages);

        // Set mocks as ambassador members through reflection or setters
        ambassador = new CarmaCloudMessageAmbassador(new AmbassadorParameter("carmacloud",
            temporaryFolder.newFile("carmacloud/carma-cloud_config.json")));

        ambassador.setRtiAmbassador(rtiMock);
        ambassador.setFederateDescriptor(handleMock);
        FieldSetter.setField(ambassador, ambassador.getClass().getDeclaredField("carmaCloudRegistrationReceiver"), receiverMock);
        FieldSetter.setField(ambassador, ambassador.getClass().getDeclaredField("carmaCloudInstanceManager"), instanceManagerMock);
    }

    @Test
    public void testInitialize() throws InternalFederateException, IllegalValueException {
        // Test initialize method
        ambassador.initialize(0, 100 * TIME.SECOND);
        verify(rtiMock, times(1)).requestAdvanceTime(eq(0L), eq(0L), eq((byte) 1));
        // cleanup threads and sockets
        ambassador.close();
    }

//    @Test
//    public void testProcessInteraction() throws InternalFederateException {
//        Interaction interactionMock = mock(Interaction.class);
//        ambassador.processInteraction(interactionMock);
//        verify(interactionMock).getTypeId();
//        verify(interactionMock).getTime();
//    }

    @Test
    public void testProcessTimeAdvanceGrant() throws InternalFederateException, IllegalValueException, NoSuchFieldException, SecurityException
    {
        //Test processTimeAdvanceGrant for CARMA Cloud Registration
        ambassador.processTimeAdvanceGrant(10);
        // Verify received messages were attempted to be pulled from CARMA Cloud Registration Receiver mock
        verify(receiverMock, times(1)).getReceivedMessages();
        verify(rtiMock, times(1)).requestAdvanceTime(eq(1000000L), eq(0L), eq((byte) 2));
    }
}
