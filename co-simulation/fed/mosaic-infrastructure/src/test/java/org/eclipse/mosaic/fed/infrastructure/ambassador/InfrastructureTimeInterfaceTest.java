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
