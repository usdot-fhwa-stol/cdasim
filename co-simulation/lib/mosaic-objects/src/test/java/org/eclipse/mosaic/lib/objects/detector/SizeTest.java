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

public class SizeTest {
    @Test
    public void testGetterSetterConstructor() {
        Size size = new Size(23, 55.3, 5);
        assertEquals(23, size.getLength(), 0.01);
        assertEquals(55.3, size.getHeight(), 0.01);
        assertEquals(5, size.getWidth(), 0.01);
    }
    @Test
    public void testEquals(){
        Size size = new Size(23, 55.3, 5);
        Size size1 = new Size(23, 55.3, 5);
        assertEquals(size, size1);
        assertEquals(size.hashCode(), size1.hashCode());
    }
}
