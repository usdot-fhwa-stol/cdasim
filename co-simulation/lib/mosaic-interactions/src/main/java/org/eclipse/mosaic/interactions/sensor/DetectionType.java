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
package org.eclipse.mosaic.interactions.sensor;


public enum DetectionType {
    CAR("CAR"),
    VAN("VAN"),
    TRUCK("TRUCK"),
    BUS("BUS"),
    MOTORCYCLE("MOTORCYCLE"),
    CYCLIST("CYCLIST"),
    PEDESTRIAN("PEDESTRIAN");
    
    
    public final String label;
    
    /**
     * Default constructor.
     *
     * @param name String
     */
    DetectionType(String name) {
        this.label = name;
    }
    
    /**
     * Returns the enum mapped from an String name.
     *
     * @param name string.
     * @return the enum mapped from String name.
     */
    public static DetectionType fromName(String name) {
        for (DetectionType type: DetectionType.values()) {
            if (type.label.equals(name)) {
                return type;
            }
        }
        throw new IllegalArgumentException("Unknown DetectionType name " + name);
    }

    public String getLabel(){
        return label;
    }
}
