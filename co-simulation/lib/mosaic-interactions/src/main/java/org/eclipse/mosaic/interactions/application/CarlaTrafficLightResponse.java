/*
 * Copyright (C) 2025.
 */

package org.eclipse.mosaic.interactions.application;

import static org.apache.commons.lang3.builder.ToStringStyle.SHORT_PREFIX_STYLE;

import javax.annotation.concurrent.Immutable;

import org.apache.commons.lang3.builder.EqualsBuilder;
import org.apache.commons.lang3.builder.HashCodeBuilder;
import org.apache.commons.lang3.builder.ToStringBuilder;
import org.eclipse.mosaic.rti.api.Interaction;

@Immutable
public final class CarlaTrafficLightResponse extends Interaction {

    private static final long serialVersionUID = 1L;

    public static final String TYPE_ID = createTypeIdentifier(CarlaTrafficLightResponse.class);

    private final String trafficLightId;
    private final String state; // e.g., "Red", "Yellow", "Green"
    private final Double timerSeconds; // optional

    public CarlaTrafficLightResponse(long time, String trafficLightId, String state, Double timerSeconds) {
        super(time);
        this.trafficLightId = trafficLightId;
        this.state = state;
        this.timerSeconds = timerSeconds;
    }

    @Override
    public String getTypeId() { return TYPE_ID; }

    public String getTrafficLightId() { return trafficLightId; }
    public String getState() { return state; }
    public Double getTimerSeconds() { return timerSeconds; }

    @Override
    public int hashCode() {
        return new HashCodeBuilder(31, 61)
            .append(trafficLightId)
            .append(state)
            .append(timerSeconds)
            .toHashCode();
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == null) return false;
        if (obj == this) return true;
        if (obj.getClass() != getClass()) return false;
        CarlaTrafficLightResponse other = (CarlaTrafficLightResponse) obj;
        return new EqualsBuilder()
            .append(this.trafficLightId, other.trafficLightId)
            .append(this.state, other.state)
            .append(this.timerSeconds, other.timerSeconds)
            .isEquals();
    }

    @Override
    public String toString() {
        return new ToStringBuilder(this, SHORT_PREFIX_STYLE)
            .appendSuper(super.toString())
            .append("trafficLightId", trafficLightId)
            .append("state", state)
            .append("timerSeconds", timerSeconds)
            .toString();
    }
}


