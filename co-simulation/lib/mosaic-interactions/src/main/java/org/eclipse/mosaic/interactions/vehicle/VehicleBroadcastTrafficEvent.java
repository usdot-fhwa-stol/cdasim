package org.eclipse.mosaic.interactions.vehicle;

import static org.apache.commons.lang3.builder.ToStringStyle.SHORT_PREFIX_STYLE;

import java.util.Arrays;

import org.eclipse.mosaic.rti.api.Interaction;

import edu.umd.cs.findbugs.annotations.SuppressWarnings;
import org.apache.commons.lang3.builder.EqualsBuilder;
import org.apache.commons.lang3.builder.HashCodeBuilder;
import org.apache.commons.lang3.builder.ToStringBuilder;

public final class VehicleBroadcastTrafficEvent extends Interaction{
    
    private static final long serialVersionUID = 1L;
    private String vehicleId;
    private float upTrack;
    private float downTrack;
    private float minimumGap;
    private float advisorySpeed;
    /**
     * String identifying the type of this interaction.
     */
    public final static String TYPE_ID = createTypeIdentifier(VehicleBroadcastTrafficEvent.class);
    
    public VehicleBroadcastTrafficEvent(long time, String vehicleId, String parameterName, String parameterValue){
        super(time);
        // Parse `parameterValue` formatted as "up_track;down_track;minimum_gap;advisory_speed" (e.g., "10;20;30;40")
        String[] values = parameterValue.split(";");
        if (values.length == 4) {
            try {
                this.upTrack = Float.parseFloat(values[0]);
                this.downTrack = Float.parseFloat(values[1]);
                this.minimumGap = Float.parseFloat(values[2]);
                this.advisorySpeed = Float.parseFloat(values[3]);
            } catch (NumberFormatException e) {
                throw new IllegalArgumentException("Invalid format for parameterValue, expected four floating-point numbers separated by semicolons", e);
            }
        } else {
            throw new IllegalArgumentException("parameterValue must contain four parts separated by semicolons");
        }
    }

    public String getVehicleId(){
        return this.vehicleId;
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
        return new HashCodeBuilder(3, 23)
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

        VehicleBroadcastTrafficEvent other = (VehicleBroadcastTrafficEvent) obj;
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

