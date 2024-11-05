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

package org.eclipse.mosaic.lib.objects.trafficevent;

import java.io.Serializable;

import org.apache.commons.lang3.builder.EqualsBuilder;
import org.apache.commons.lang3.builder.HashCodeBuilder;
import org.apache.commons.lang3.builder.ToStringBuilder;

import com.google.gson.annotations.Expose;
import com.google.gson.annotations.SerializedName;

import static org.apache.commons.lang3.builder.ToStringStyle.SHORT_PREFIX_STYLE;

public class MsgerTrafficEvent implements Serializable{
    
    @Expose
    @SerializedName("up_track")
    private float upTrack;

    @Expose
    @SerializedName("down_track")
    private float downTrack;

    @Expose
    @SerializedName("minimum_gap")
    private float minimumGap;

    @Expose
    @SerializedName("advisory_speed")
    private float advisorySpeed;
    
    private String vehicleId;

    public MsgerTrafficEvent(String vehicleId, float upTrack, float downTrack, float minimumGap, float advisorySpeed){
        this.vehicleId = vehicleId;
        this.upTrack = upTrack;
        this.downTrack = downTrack;
        this.minimumGap = minimumGap;
        this.advisorySpeed = advisorySpeed;
    }

    public String getVehicleId() {
        return vehicleId;
    }
    
    public float getUpTrack(){
        return this.upTrack;
    }

    public float getDownTrack(){
        return this.downTrack;
    }

    public float getMinimumGap(){
        return this.minimumGap;
    }

    public float getAdvisorySpeed(){
        return this.advisorySpeed;
    }

    @Override
    public int hashCode() {
        return new HashCodeBuilder(7, 37)
                .append(vehicleId)
                .append(upTrack)
                .append(downTrack)
                .append(minimumGap)
                .append(advisorySpeed)
                .toHashCode();
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == null) {
            return false;
        }
        if (obj == this) {
            return true;
        }
        if (obj.getClass() != getClass()) {
            return false;
        }

        MsgerTrafficEvent other = (MsgerTrafficEvent) obj;
        return new EqualsBuilder()
                .append(this.vehicleId, other.vehicleId)
                .append(this.upTrack, other.upTrack)
                .append(this.downTrack, other.downTrack)
                .append(this.minimumGap, other.minimumGap)
                .append(this.advisorySpeed, other.advisorySpeed)
                .isEquals();
    }

    @Override
    public String toString() {
        return new ToStringBuilder(this, SHORT_PREFIX_STYLE)
                .appendSuper(super.toString())
                .append("vehicleId", vehicleId)
                .append("upTrack", upTrack)
                .append("downTrack", downTrack)
                .append("minimumGap", minimumGap)
                .append("advisorySpeed", advisorySpeed)
                .toString();
    }
}
