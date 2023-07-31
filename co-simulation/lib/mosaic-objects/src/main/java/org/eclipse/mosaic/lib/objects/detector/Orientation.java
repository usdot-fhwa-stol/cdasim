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

public class Orientation implements Serializable{
    private static final long serialVersionUID = 1L;

    private double yaw;
    
    private double pitch;

    private double roll;

    public Orientation( double yaw, double pitch, double roll) {
        this.yaw = yaw;
        this.pitch = pitch;
        this.roll = roll;
    }

    public double getYaw() {
        return yaw;
    }


    public double getPitch() {
        return pitch;
    }

    public double getRoll() {
        return roll;
    }


    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        long temp;
        temp = Double.doubleToLongBits(pitch);
        result = prime * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(yaw);
        result = prime * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(roll);
        result = prime * result + (int) (temp ^ (temp >>> 32));
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
        Orientation other = (Orientation) obj;
        if (Double.doubleToLongBits(pitch) != Double.doubleToLongBits(other.pitch))
            return false;
        if (Double.doubleToLongBits(yaw) != Double.doubleToLongBits(other.yaw))
            return false;
        if (Double.doubleToLongBits(roll) != Double.doubleToLongBits(other.roll))
            return false;
        return true;
    }

    
    
}
