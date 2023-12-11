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

    private int objectId;

    private CartesianPoint position;

    private Double[][] positionCovariance = new Double[3][3];

    private Vector3d velocity;

    private Double[][] velocityCovariance = new Double[3][3];

    private Vector3d angularVelocity;

    private Double[][] angularVelocityCovariance = new Double[3][3];

    private Size size;

    private int timestamp;
    /**
     * Constructor for Detected Object information.
     * 
     * @param type              {@link DetectionType}.
     * @param confidence        in detection type classification.
     * @param sensorId          of sensor/detector reporting object detection
     * @param projString        containing information about reference frame in 
     *                          which kinematic information is reported.
     * @param objectId          unique string ID of detected object (only guaranteed 
     *                          unique among other detected objects reported by the 
     *                          same sensor).
     * @param position          position of detected object relative to sensor/detector
     *                          frame.
     * @param velocity          velocity of detected object in sensor/detector frame.
     * @param angularVelocity   angular velocity of detected object in sensor/detector frame.
     * @param size              size of object including height,width and length.
     */
    public DetectedObject(DetectionType type, double confidence, String sensorId, String projString, int objectId,
            CartesianPoint position, Vector3d velocity, Vector3d angularVelocity, Size size, int timestamp) {
        this.type = type;
        this.confidence = confidence;
        this.sensorId = sensorId;
        this.projString = projString;
        this.objectId = objectId;
        this.position = position;
        this.velocity = velocity;
        this.angularVelocity = angularVelocity;
        this.size = size;
        this.timestamp = timestamp;
    }

    /**
     * Getter for {@link DetectionType}
     * @return
     */
    public DetectionType getType() {
        return type;
    }

    /**
     * Getter for projection string which describes how to translate 
     * Detected Object position,velocity, and angular velocity from a
     * sensor/detector relative map reference frame to other reference frames.
     * @return
     */
    public String getProjString() {
        return projString;
    }

    public void setPositionCovariance(Double[][] positionCovariance) {
        this.positionCovariance = positionCovariance;
    }

    public void setVelocityCovariance(Double[][] velocityCovariance) {
        this.velocityCovariance = velocityCovariance;
    }

    public void setAngularVelocityCovariance(Double[][] angularVelocityCovariance) {
        this.angularVelocityCovariance = angularVelocityCovariance;
    }

    /**
     * Getter for confidence value in DetectionType classification.
     * @return
     */
    public double getConfidence() {
        return confidence;
    }

    /**
     * Getter for sensor ID reporting detected object.
     * @return
     */
    public String getSensorId() {
        return sensorId;
    }

    /**
     * Getter for String object ID.
     * @return
     */
    public int getObjectId() {
        return objectId;
    }

    /**
     * Getter for object location.
     * @return
     */
    public CartesianPoint getPosition() {
        return position;
    }

    public Double[][] getPositionCovariance() {
        return positionCovariance;
    }

    public Double[][] getVelocityCovariance() {
        return velocityCovariance;
    }

    public Double[][] getAngularVelocityCovariance() {
        return angularVelocityCovariance;
    }

    /**
     * Getter for object velocity.
     * @return
     */
    public Vector3d getVelocity() {
        return velocity;
    }

    /**
     * Getter for object angular velocity
     * @return
     */
    public Vector3d getAngularVelocity() {
        return angularVelocity;
    }

    /**
     * Getter for object size
     * @return
     */
    public Size getSize() {
        return size;
    }
    
    /**
     * Setter for object {@link DetectionType}
     * @param type
     */
    public void setType(DetectionType type) {
        this.type = type;
    }

    /**
     * Setter for confidence in detected object {@link DetectionType} classification.
     * @param confidence
     */
    public void setConfidence(double confidence) {
        this.confidence = confidence;
    }

    /**
     * Setter for String sensor ID reporting DetectedObject.
     * @param sensorId
     */
    public void setSensorId(String sensorId) {
        this.sensorId = sensorId;
    }

    /**
     * Setter for projection string to translate object kinematic information from
     * reference frame to geodetic cordinates.
     * @param projString
     */
    public void setProjString(String projString) {
        this.projString = projString;
    }

    /**
     * Setter for detected object unique string ID.
     * @param objectId
     */
    public void setObjectId(int objectId) {
        this.objectId = objectId;
    }

    /**
     * Setter for detected object position.
     * @param position
     */
    public void setPosition(CartesianPoint position) {
        this.position = position;
    }

    /**
     * Setter for detected object velocity.
     * @param velocity {@link Vector3d}
     */
    public void setVelocity(Vector3d velocity) {
        this.velocity = velocity;
    }
    /**
     * Setter for detected object angular velocity.
     * @param angularVelocity {@link Vector3d}
     */
    public void setAngularVelocity(Vector3d angularVelocity) {
        this.angularVelocity = angularVelocity;
    }

    /**
     * Setter for object size.
     * @param size {@link Size}
     */
    public void setSize(Size size) {
        this.size = size;
    }

    public int getTimestamp() {
        return timestamp;
    }

    public void setTimestamp(int timestamp) {
        this.timestamp = timestamp;
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
        result = prime * result + objectId;
        result = prime * result + ((position == null) ? 0 : position.hashCode());
        result = prime * result + Arrays.deepHashCode(positionCovariance);
        result = prime * result + ((velocity == null) ? 0 : velocity.hashCode());
        result = prime * result + Arrays.deepHashCode(velocityCovariance);
        result = prime * result + ((angularVelocity == null) ? 0 : angularVelocity.hashCode());
        result = prime * result + Arrays.deepHashCode(angularVelocityCovariance);
        result = prime * result + ((size == null) ? 0 : size.hashCode());
        result = prime * result + timestamp;
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
        if (objectId != other.objectId)
            return false;
        if (position == null) {
            if (other.position != null)
                return false;
        } else if (!position.equals(other.position))
            return false;
        if (!Arrays.deepEquals(positionCovariance, other.positionCovariance))
            return false;
        if (velocity == null) {
            if (other.velocity != null)
                return false;
        } else if (!velocity.equals(other.velocity))
            return false;
        if (!Arrays.deepEquals(velocityCovariance, other.velocityCovariance))
            return false;
        if (angularVelocity == null) {
            if (other.angularVelocity != null)
                return false;
        } else if (!angularVelocity.equals(other.angularVelocity))
            return false;
        if (!Arrays.deepEquals(angularVelocityCovariance, other.angularVelocityCovariance))
            return false;
        if (size == null) {
            if (other.size != null)
                return false;
        } else if (!size.equals(other.size))
            return false;
        if (timestamp != other.timestamp)
            return false;
        return true;
    }

    @Override
    public String toString() {
        return "DetectedObject [type=" + type + ", confidence=" + confidence + ", sensorId=" + sensorId
                + ", projString=" + projString + ", objectId=" + objectId + ", position=" + position
                + ", positionCovariance=" + Arrays.deepToString(positionCovariance) + ", velocity=" + velocity
                + ", velocityCovariance=" + Arrays.deepToString(velocityCovariance) + ", angularVelocity=" + angularVelocity
                + ", angularVelocityCovariance=" + Arrays.deepToString(angularVelocityCovariance) + ", size=" + size.toString()
                + ", timestamp=" + timestamp + "]";
    }
}
