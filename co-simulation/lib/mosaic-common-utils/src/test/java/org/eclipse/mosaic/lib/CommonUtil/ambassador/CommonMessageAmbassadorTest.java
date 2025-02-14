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
package org.eclipse.mosaic.lib.CommonUtil.ambassador;

import java.io.File;
import java.io.IOException;

import org.eclipse.mosaic.lib.CommonUtil.configuration.CommonConfiguration;
import org.eclipse.mosaic.lib.util.junit.TestFileRule;
import org.eclipse.mosaic.rti.TIME;
import org.eclipse.mosaic.rti.api.RtiAmbassador;
import org.eclipse.mosaic.rti.api.parameters.AmbassadorParameter;
import org.eclipse.mosaic.rti.api.parameters.FederateDescriptor;
import org.eclipse.mosaic.rti.config.CLocalHost;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.RuleChain;
import org.junit.rules.TemporaryFolder;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;


public class CommonMessageAmbassadorTest {
    private final TemporaryFolder temporaryFolder = new TemporaryFolder();

    private final TestFileRule testFileRule = new TestFileRule(temporaryFolder).basedir("carma")
            .with("carma_config.json", "/carma_config.json");

    @Rule
    public RuleChain chain = RuleChain.outerRule(temporaryFolder).around(testFileRule);

    private RtiAmbassador rtiMock;

    private CommonMessageAmbassador ambassador;

    private CommonInstanceManager instanceManager;

    @Before
    public void setup() throws IOException {

        rtiMock = mock(RtiAmbassador.class);

        FederateDescriptor handleMock = mock(FederateDescriptor.class);

        File workingDir = temporaryFolder.getRoot();

        CLocalHost testHostConfig = new CLocalHost();

        testHostConfig.workingDirectory = workingDir.getAbsolutePath();

        when(handleMock.getHost()).thenReturn(testHostConfig);

        when(handleMock.getId()).thenReturn("carma");

        ambassador = new CommonMessageAmbassador(
                new AmbassadorParameter("carma", testFileRule.get("carma_config.json")),instanceManager, CommonRegistrationMessage.class, CommonConfiguration.class);

        ambassador.setRtiAmbassador(rtiMock);

        ambassador.setFederateDescriptor(handleMock);
    }

    @Test
    public void initialize() throws Throwable {

        // RUN
        ambassador.initialize(0, 100 * TIME.SECOND);
        // ASSERT
        verify(rtiMock, times(1)).requestAdvanceTime(eq(0L), eq(0L), eq((byte) 1));
    }
    
}
