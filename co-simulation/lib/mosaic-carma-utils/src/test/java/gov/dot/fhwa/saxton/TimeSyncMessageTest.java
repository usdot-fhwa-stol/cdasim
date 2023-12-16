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
package gov.dot.fhwa.saxton;

import static org.junit.Assert.assertEquals;

import org.junit.Before;
import org.junit.Test;

public class TimeSyncMessageTest {
     @Before
    public void setUp() throws Exception {
    }

    @Test
    public void testGetterSetterConstructor() {
        TimeSyncMessage msg = new TimeSyncMessage(1000, 10);
  
        assertEquals( 10, msg.getSeq());
        assertEquals(1000, msg.getTimestep());

        msg.setSeq(20);
        msg.setTimestep(2000);

        assertEquals( 20, msg.getSeq());
        assertEquals(2000, msg.getTimestep());

    }
}
