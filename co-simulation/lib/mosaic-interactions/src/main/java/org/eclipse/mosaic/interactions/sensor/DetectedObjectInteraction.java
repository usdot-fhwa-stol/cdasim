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

import org.eclipse.mosaic.rti.api.Interaction;

public class DetectedObjectInteraction extends Interaction {

    public static final String TYPE_ID = createTypeIdentifier(DetectedObjectInteraction.class);

    private DetectedObject detectedObject;

    public DetectedObjectInteraction(long time, DetectedObject detectedObject) {
        super(time);
        this.detectedObject = detectedObject;
    }

    public DetectedObject getDetectedObject() {
        return detectedObject;
    }

    public void setDetectedObject(DetectedObject detectedObject) {
        this.detectedObject = detectedObject;
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = super.hashCode();
        result = prime * result + ((detectedObject == null) ? 0 : detectedObject.hashCode());
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
        DetectedObjectInteraction other = (DetectedObjectInteraction) obj;
        if (detectedObject == null) {
            if (other.detectedObject != null)
                return false;
        } else if (!detectedObject.equals(other.detectedObject))
            return false;
        return true;
    }

    
    
    
}
