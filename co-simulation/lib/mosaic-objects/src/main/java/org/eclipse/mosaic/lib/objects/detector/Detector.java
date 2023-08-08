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

import java.io.Serializable;

import org.eclipse.mosaic.lib.geo.CartesianPoint;

public class Detector implements Serializable {
    private static final long serialVersionUID = 1L;

    private String sensorId;
    private DetectorType type;
    private Orientation orientation;
    private CartesianPoint location;

    /**
     * Sensor/Detector constructor
     * 
     * @param sensorId unique string ID of sensor/detector.
     * @param type of the sensor/detector.
     * @param orientation of the sensor/detector. 
     * @param location of the sensor/detector.
     */
    public Detector(String sensorId, DetectorType type, Orientation orientation, CartesianPoint location) {
        this.sensorId = sensorId;
        this.type = type;
        this.orientation = orientation;
        this.location = location;
    }

    /**
     * Get unique String sensor ID.
     * @return
     */
    public String getSensorId() {
        return sensorId;
    }

    /**
     * Set unique String sensor ID.
     * @param sensorId
     */
    public void setSensorId(String sensorId) {
        this.sensorId = sensorId;
    }

    /**
     * Get sensor/detector {@link DetectorType}.
     * @return
     */
    public DetectorType getType() {
        return type;
    }

    /**
     * Set the sensor/detector {@link DetectorType}.
     * @param type
     */
    public void setType(DetectorType type) {
        this.type = type;
    }

    /**
     * Get sensor/detector {@link Orientation}.
     * @return
     */
    public Orientation getOrientation() {
        return orientation;
    }

    /**
     * Set sensor/detector {@link Orientation}.
     * @param orientation
     */
    public void setOrientation(Orientation orientation) {
        this.orientation = orientation;
    }

    /**
     * Get sensor/detector location.
     * @return {@link CartesianPoint}.
     */
    public CartesianPoint getLocation() {
        return location;
    }
    /**
     * Set sensor/detector location
     * @param point {@link CartesianPoint}.
     */
    public void setLocation(CartesianPoint point) {
        this.location = point;
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + ((sensorId == null) ? 0 : sensorId.hashCode());
        result = prime * result + ((type == null) ? 0 : type.hashCode());
        result = prime * result + ((orientation == null) ? 0 : orientation.hashCode());
        result = prime * result + ((location == null) ? 0 : location.hashCode());
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
        Detector other = (Detector) obj;
        if (sensorId == null) {
            if (other.sensorId != null)
                return false;
        } else if (!sensorId.equals(other.sensorId))
            return false;
        if (type != other.type)
            return false;
        if (orientation == null) {
            if (other.orientation != null)
                return false;
        } else if (!orientation.equals(other.orientation))
            return false;
        if (location == null) {
            if (other.location != null)
                return false;
        } else if (!location.equals(other.location))
            return false;
        return true;
    }

    @Override
    public String toString() {
        return "Detector [sensorId=" + sensorId + ", type=" + type + ", orientation=" + orientation + ", location="
                + location + "]";
    }
    

    
   
}
