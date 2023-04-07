/*
 * Copyright (c) 2021 Old Dominion University. All rights reserved.
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 */

package org.eclipse.mosaic.fed.infrastructure.ambassador;

import org.junit.Test;

import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import org.junit.Before;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import com.google.gson.Gson;

public class InfrastructureTimeInterfaceTest {
    @Mock
    private InfrastructureInstanceManager manager;
    @Mock
    private InfrastructureInstance infrastructureInstanceMock1;
    @Mock
    private InfrastructureInstance infrastructureInstanceMock2;
    private InfrastructureTimeInterface infrastructureTimeInterface;

    @Before
    public void setup() {
        MockitoAnnotations.initMocks(this);
        infrastructureTimeInterface = new InfrastructureTimeInterface(manager);
    }

    @Test
    public void testTimesyncUpdate() throws IOException {
        InfrastructureTimeMessage infrastructureTimeMessage = new InfrastructureTimeMessage();
        infrastructureTimeMessage.setSeq(1);
        infrastructureTimeMessage.setTimestep((long) 20230406);

        Map<String, InfrastructureInstance> managed_instance_map = new HashMap<>();
        managed_instance_map.put("instance1", infrastructureInstanceMock1);
        managed_instance_map.put("instance2", infrastructureInstanceMock2);

        when(manager.getManagedInstances()).thenReturn(managed_instance_map);

        infrastructureTimeInterface.onTimeStepUpdate(infrastructureTimeMessage);

        infrastructureTimeMessage.setSeq(2);
        infrastructureTimeMessage.setTimestep((long) 20230407);

        infrastructureTimeInterface.onTimeStepUpdate(infrastructureTimeMessage);

        Gson gson = new Gson();
        String temp = gson.toJson(infrastructureTimeMessage);
        byte[] t_byte = temp.getBytes();

        verify(infrastructureInstanceMock1, times(1)).sendTimeSyncMsgs(eq((byte[]) t_byte));
        verify(infrastructureInstanceMock2, times(1)).sendTimeSyncMsgs(eq((byte[]) t_byte));

        verify(infrastructureInstanceMock1, times(2)).sendTimeSyncMsgs(any());
        verify(infrastructureInstanceMock2, times(2)).sendTimeSyncMsgs(any());

    }
}
