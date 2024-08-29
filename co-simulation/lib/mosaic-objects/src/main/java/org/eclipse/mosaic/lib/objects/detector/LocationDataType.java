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



import org.eclipse.mosaic.lib.objects.detector.gson.LocationDataTypeAdapter;

import com.google.gson.annotations.JsonAdapter;

@JsonAdapter(LocationDataTypeAdapter.class)
public enum LocationDataType {
    CARTESIAN("CARTESIAN"),
    WGS84("WGS84");
    
    
    public final String label;
    
    /**
     * Default constructor.
     *
     * @param label String
     */
    LocationDataType(String label) {
        this.label = label;
    }
    
    /**
     * Returns the enum mapped from an String name.
     *
     * @param label string.
     * @return the enum mapped from String name.
     */
    public static LocationDataType fromLabel(String label) {
        for (LocationDataType type: LocationDataType.values()) {
            if (type.label.equalsIgnoreCase(label)) {
                return type;
            }
        }
        throw new IllegalArgumentException("Unknown DetectorType label " + label);
    }

    public String getLabel(){
        return label;
    }
}
