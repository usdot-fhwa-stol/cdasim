/*
 * Copyright (C) 2024 LEIDOS.
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

package org.eclipse.mosaic.lib.objects.vehicle;
import java.io.Serializable;

import org.apache.commons.lang3.builder.EqualsBuilder;
import org.apache.commons.lang3.builder.HashCodeBuilder;
import org.apache.commons.lang3.builder.ToStringBuilder;

import com.google.gson.annotations.Expose;
import com.google.gson.annotations.SerializedName;

import static org.apache.commons.lang3.builder.ToStringStyle.SHORT_PREFIX_STYLE;

import java.io.Serializable;

public class MsgerVehicleStatus implements Serializable {
    
    // Inner classes for vehicle_pose and vehicle_twist
    public static class VehiclePose implements Serializable {
        private double x;
        private double y;
        private double z;

        public VehiclePose(double x, double y, double z) {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        // Getters and setters
        public double getX() { return x; }
        public void setX(double x) { this.x = x; }
        
        public double getY() { return y; }
        public void setY(double y) { this.y = y; }
        
        public double getZ() { return z; }
        public void setZ(double z) { this.z = z; }
    }

    public static class VehicleTwist implements Serializable {
        private float x;
        private float y;
        private float z;

        public VehicleTwist(float x, float y, float z) {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        // Getters and setters
        public float getX() { return x; }
        public void setX(float x) { this.x = x; }
        
        public float getY() { return y; }
        public void setY(float y) { this.y = y; }
        
        public float getZ() { return z; }
        public void setZ(float z) { this.z = z; }
    }

    // Fields for vehicle status
    @SerializedName("vehicle_pose")
    private VehiclePose vehiclePose;

    @SerializedName("vehicle_twist")
    private VehicleTwist vehicleTwist;

    @SerializedName("siren_active")
    private boolean sirenActive;

    @SerializedName("light_active")
    private boolean lightActive;

    // Constructor
    public MsgerVehicleStatus(VehiclePose vehiclePose, VehicleTwist vehicleTwist, 
                              boolean sirenActive, boolean lightActive) {
        this.vehiclePose = vehiclePose;
        this.vehicleTwist = vehicleTwist;
        this.sirenActive = sirenActive;
        this.lightActive = lightActive;
    }

    // Getters and setters
    public VehiclePose getVehiclePose() { return vehiclePose; }
    public void setVehiclePose(VehiclePose vehiclePose) { this.vehiclePose = vehiclePose; }

    public VehicleTwist getVehicleTwist() { return vehicleTwist; }
    public void setVehicleTwist(VehicleTwist vehicleTwist) { this.vehicleTwist = vehicleTwist; }

    public boolean isSirenActive() { return sirenActive; }
    public void setSirenActive(boolean sirenActive) { this.sirenActive = sirenActive; }

    public boolean isLightActive() { return lightActive; }
    public void setLightActive(boolean lightActive) { this.lightActive = lightActive; }
}