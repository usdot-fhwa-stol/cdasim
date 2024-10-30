package org.eclipse.mosaic.interactions.application;

import static org.apache.commons.lang3.builder.ToStringStyle.SHORT_PREFIX_STYLE;

import org.eclipse.mosaic.rti.api.Interaction;

import org.apache.commons.lang3.builder.EqualsBuilder;
import org.apache.commons.lang3.builder.HashCodeBuilder;
import org.apache.commons.lang3.builder.ToStringBuilder;

public final class MsgerRequesetTrafficEvent extends Interaction{
    
    private static final long serialVersionUID = 1L;
    private String vehicleId;
    private String parameterName;
    /**
     * String identifying the type of this interaction.
     */
    public final static String TYPE_ID = createTypeIdentifier(MsgerRequesetTrafficEvent.class);
    
    public MsgerRequesetTrafficEvent(long time, String vehicleId, String parameterName){
        super(time);
    }

    public String vehicleId(){
        return this.vehicleId;
    }

    public String getParameterName(){
        return this.parameterName;
    }

    @Override
    public int hashCode() {
        return new HashCodeBuilder(3, 23)
                .append(vehicleId)
                .append(parameterName)
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

        MsgerRequesetTrafficEvent other = (MsgerRequesetTrafficEvent) obj;
        return new EqualsBuilder()
                .append(this.vehicleId, other.vehicleId)
                .append(this.parameterName, other.parameterName)
                .isEquals();
    }

    @Override
    public String toString() {
        return new ToStringBuilder(this, SHORT_PREFIX_STYLE)
                .appendSuper(super.toString())
                .append("vehicleId", vehicleId)
                .append("upTrack", parameterName)
                .toString();
    }

}

