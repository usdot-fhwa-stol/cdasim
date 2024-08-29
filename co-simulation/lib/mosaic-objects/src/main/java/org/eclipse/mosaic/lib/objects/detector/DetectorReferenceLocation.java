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

import org.eclipse.mosaic.lib.geo.CartesianPoint;

public class DetectorReferenceLocation {

    private LocationDataType type;

    private CartesianPoint location;

    private Orientation orientation;

    public DetectorReferenceLocation(LocationDataType locationDataType, CartesianPoint location,
            Orientation orientation) {
        this.type = locationDataType;
        this.location = location;
        this.orientation = orientation;
    }

    public CartesianPoint getLocation() {
        return location;
    }

    public void setLocation(CartesianPoint location) {
        this.location = location;
    }

    public Orientation getOrientation() {
        return orientation;
    }

    public void setOrientation(Orientation orientation) {
        this.orientation = orientation;
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + ((type == null) ? 0 : type.hashCode());
        result = prime * result + ((location == null) ? 0 : location.hashCode());
        result = prime * result + ((orientation == null) ? 0 : orientation.hashCode());
        return result;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj)
            return true;
        if (obj == null)
            return false;
        if (getClass() != obj.getClass())
            return false;
        DetectorReferenceLocation other = (DetectorReferenceLocation) obj;
        if (type != other.type)
            return false;
        if (location == null) {
            if (other.location != null)
                return false;
        } else if (!location.equals(other.location))
            return false;
        if (orientation == null) {
            if (other.orientation != null)
                return false;
        } else if (!orientation.equals(other.orientation))
            return false;
        return true;
    }

    @Override
    public String toString() {
        return "DetectorReferenceLocation [locationDataType=" + type + ", location=" + location
                + ", orientation=" + orientation + "]";
    }
    

}
