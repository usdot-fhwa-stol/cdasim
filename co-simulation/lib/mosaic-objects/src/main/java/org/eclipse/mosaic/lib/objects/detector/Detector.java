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
    private DetectorReferenceLocation ref;

    /**
     * Sensor/Detector constructor
     * 
     * @param sensorId unique string ID of sensor/detector.
     * @param type of the sensor/detector.
     * @param ref reference location of sensor
     */
    public Detector(String sensorId, DetectorType type,
            DetectorReferenceLocation ref) {
        this.sensorId = sensorId;
        this.type = type;
        this.ref = ref;
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


    public DetectorReferenceLocation getRef() {
        return ref;
    }


    public void setRef(DetectorReferenceLocation ref) {
        this.ref = ref;
    }


    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + ((sensorId == null) ? 0 : sensorId.hashCode());
        result = prime * result + ((type == null) ? 0 : type.hashCode());
        result = prime * result + ((ref == null) ? 0 : ref.hashCode());
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
        if (ref == null) {
            if (other.ref != null)
                return false;
        } else if (!ref.equals(other.ref))
            return false;
        return true;
    }


    @Override
    public String toString() {
        return "Detector [sensorId=" + sensorId + ", type=" + type  + ", ref=" + ref + "]";
    }

    

    
    

    
   
}
