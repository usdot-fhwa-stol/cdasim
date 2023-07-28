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
import java.util.Arrays;

import org.eclipse.mosaic.lib.geo.CartesianPoint;
import org.eclipse.mosaic.lib.math.Vector3d;

public final class DetectedObject implements Serializable {

    private static final long serialVersionUID = 1L;

    private DetectionType type;

    private double confidence;

    private String sensorId;

    private String projString;

    private String objectId;

    private CartesianPoint position;

    private Double[] positionCovariance = new Double[9];

    private Vector3d velocity;

    private Double[] velocityCovariance = new Double[9];

    private Vector3d angularVelocity;

    private Double[] angularVelocityCovariance = new Double[9];

    private Size size;

    public DetectedObject(DetectionType type, double confidence, String sensorId, String projString, String objectId,
            CartesianPoint position, Vector3d velocity, Vector3d angularVelocity, Size size) {
        this.type = type;
        this.confidence = confidence;
        this.sensorId = sensorId;
        this.projString = projString;
        this.objectId = objectId;
        this.position = position;
        this.velocity = velocity;
        this.angularVelocity = angularVelocity;
        this.size = size;
    }

    public DetectionType getType() {
        return type;
    }

    public String getProjString() {
        return projString;
    }

    public Double[] getPositionCovariance() {
        return positionCovariance;
    }

    public void setPositionCovariance(Double[] positionCovariance) {
        this.positionCovariance = positionCovariance;
    }

    public Double[] getVelocityCovariance() {
        return velocityCovariance;
    }

    public void setVelocityCovariance(Double[] velocityCovariance) {
        this.velocityCovariance = velocityCovariance;
    }

    public Double[] getAngularVelocityCovariance() {
        return angularVelocityCovariance;
    }

    public void setAngularVelocityCovariance(Double[] angularVelocityCovariance) {
        this.angularVelocityCovariance = angularVelocityCovariance;
    }


    public double getConfidence() {
        return confidence;
    }

    public String getSensorId() {
        return sensorId;
    }


    public String getObjectId() {
        return objectId;
    }


    public CartesianPoint getPosition() {
        return position;
    }


    public Vector3d getVelocity() {
        return velocity;
    }

    public Vector3d getAngularVelocity() {
        return angularVelocity;
    }


    public Size getSize() {
        return size;
    }
    
    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + ((type == null) ? 0 : type.hashCode());
        long temp;
        temp = Double.doubleToLongBits(confidence);
        result = prime * result + (int) (temp ^ (temp >>> 32));
        result = prime * result + ((sensorId == null) ? 0 : sensorId.hashCode());
        result = prime * result + ((projString == null) ? 0 : projString.hashCode());
        result = prime * result + ((objectId == null) ? 0 : objectId.hashCode());
        result = prime * result + ((position == null) ? 0 : position.hashCode());
        result = prime * result + Arrays.hashCode(positionCovariance);
        result = prime * result + ((velocity == null) ? 0 : velocity.hashCode());
        result = prime * result + Arrays.hashCode(velocityCovariance);
        result = prime * result + ((angularVelocity == null) ? 0 : angularVelocity.hashCode());
        result = prime * result + Arrays.hashCode(angularVelocityCovariance);
        result = prime * result + ((size == null) ? 0 : size.hashCode());
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
        DetectedObject other = (DetectedObject) obj;
        if (type != other.type)
            return false;
        if (Double.doubleToLongBits(confidence) != Double.doubleToLongBits(other.confidence))
            return false;
        if (sensorId == null) {
            if (other.sensorId != null)
                return false;
        } else if (!sensorId.equals(other.sensorId))
            return false;
        if (projString == null) {
            if (other.projString != null)
                return false;
        } else if (!projString.equals(other.projString))
            return false;
        if (objectId == null) {
            if (other.objectId != null)
                return false;
        } else if (!objectId.equals(other.objectId))
            return false;
        if (position == null) {
            if (other.position != null)
                return false;
        } else if (!position.equals(other.position))
            return false;
        if (!Arrays.equals(positionCovariance, other.positionCovariance))
            return false;
        if (velocity == null) {
            if (other.velocity != null)
                return false;
        } else if (!velocity.equals(other.velocity))
            return false;
        if (!Arrays.equals(velocityCovariance, other.velocityCovariance))
            return false;
        if (angularVelocity == null) {
            if (other.angularVelocity != null)
                return false;
        } else if (!angularVelocity.equals(other.angularVelocity))
            return false;
        if (!Arrays.equals(angularVelocityCovariance, other.angularVelocityCovariance))
            return false;
        if (size == null) {
            if (other.size != null)
                return false;
        } else if (!size.equals(other.size))
            return false;
        return true;
    }

}
