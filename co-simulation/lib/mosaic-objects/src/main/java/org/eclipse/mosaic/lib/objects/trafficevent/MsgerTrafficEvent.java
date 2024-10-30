package org.eclipse.mosaic.lib.objects.trafficevent;

import static org.apache.commons.lang3.builder.ToStringStyle.SHORT_PREFIX_STYLE;

import java.io.Serializable;

import org.apache.commons.lang3.builder.EqualsBuilder;
import org.apache.commons.lang3.builder.HashCodeBuilder;
import org.apache.commons.lang3.builder.ToStringBuilder;
import org.eclipse.mosaic.lib.objects.v2x.V2xReceiverInformation;

public class MsgerTrafficEvent implements Serializable{
    
    private float upTrack;
    private float downTrack;
    private float minimumGap;
    private float advisorySpeed;

    public MsgerTrafficEvent(float upTrack, float downTrack, float minimumGap, float advisorySpeed){
        this.upTrack = upTrack;
        this.downTrack = downTrack;
        this.minimumGap = minimumGap;
        this.advisorySpeed = advisorySpeed;
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
                .append("upTrack", upTrack)
                .append("downTrack", downTrack)
                .append("minimumGap", minimumGap)
                .append("advisorySpeed", advisorySpeed)
                .toString();
    }
}
