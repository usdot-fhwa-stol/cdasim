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
package org.eclipse.mosaic.lib.objects.detector;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class OrientationTest {
    @Test
    public void testEquals() {
        Orientation orientation = new Orientation(23, 55.3, 5);
        assertEquals(23, orientation.getYaw(), 0.01);
        assertEquals(55.3, orientation.getPitch(), 0.01);
        assertEquals(5, orientation.getRoll(), 0.01);
    }

    @Test
    public void testGetPitch() {
        Orientation orientation = new Orientation(23, 55.3, 5);
        Orientation orientation1 = new Orientation(23, 55.3, 5);
        assertEquals(orientation, orientation1);
        assertEquals(orientation.hashCode(), orientation1.hashCode());
    }
}
