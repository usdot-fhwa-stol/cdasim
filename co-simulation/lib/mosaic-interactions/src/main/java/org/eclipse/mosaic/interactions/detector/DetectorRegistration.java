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
package org.eclipse.mosaic.interactions.detector;

import org.eclipse.mosaic.lib.objects.detector.Detector;
import org.eclipse.mosaic.rti.api.Interaction;

public class DetectorRegistration extends Interaction {
    public static final String TYPE_ID = createTypeIdentifier(DetectorRegistration.class);

    private Detector detector;

    private String infrastructureId;
    /**
     * Constructor
     * @param time for interaction.
     * @param sensor to register.
     */
    public DetectorRegistration(long time, Detector sensor, String infrastructureId) {
        super(time);
        this.detector = sensor;
        this.infrastructureId = infrastructureId;
    }
    /**
     * Getter for sensor/detector information.
     * @return 
     */
    public Detector getDetector() {
        return detector;
    }
    /**
     * Setter for sensor/detection information.
     * @param sensor
     */
    public void setDetector(Detector sensor) {
        this.detector = sensor;
    }
    public String getInfrastructureId() {
        return infrastructureId;
    }
    public void setInfrastructureId(String infrastructureId) {
        this.infrastructureId = infrastructureId;
    }
    @Override
    public int hashCode() {
        final int prime = 31;
        int result = super.hashCode();
        result = prime * result + ((detector == null) ? 0 : detector.hashCode());
        result = prime * result + ((infrastructureId == null) ? 0 : infrastructureId.hashCode());
        return result;
    }
    @Override
    public boolean equals(Object obj) {
        if (this == obj)
            return true;
        if (!super.equals(obj))
            return false;
        if (getClass() != obj.getClass())
            return false;
        DetectorRegistration other = (DetectorRegistration) obj;
        if (detector == null) {
            if (other.detector != null)
                return false;
        } else if (!detector.equals(other.detector))
            return false;
        if (infrastructureId == null) {
            if (other.infrastructureId != null)
                return false;
        } else if (!infrastructureId.equals(other.infrastructureId))
            return false;
        return true;
    }
    
    
    
    
}
