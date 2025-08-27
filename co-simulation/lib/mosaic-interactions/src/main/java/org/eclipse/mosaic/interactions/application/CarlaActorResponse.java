/*
 * Copyright (C) 2025.
 */

package org.eclipse.mosaic.interactions.application;

import static org.apache.commons.lang3.builder.ToStringStyle.SHORT_PREFIX_STYLE;

import java.util.Collections;
import java.util.List;
import java.util.Map;

import javax.annotation.concurrent.Immutable;

import org.apache.commons.lang3.builder.EqualsBuilder;
import org.apache.commons.lang3.builder.HashCodeBuilder;
import org.apache.commons.lang3.builder.ToStringBuilder;
import org.eclipse.mosaic.rti.api.Interaction;

@Immutable
public final class CarlaActorResponse extends Interaction {

    private static final long serialVersionUID = 1L;

    public static final String TYPE_ID = createTypeIdentifier(CarlaActorResponse.class);

    private final String actorId;
    private final List<Double> location; // optional
    private final List<Double> rotation; // optional
    private final List<Double> velocity; // optional
    private final Map<String, Object> properties; // optional

    public CarlaActorResponse(long time, String actorId, List<Double> location, List<Double> rotation,
                              List<Double> velocity, Map<String, Object> properties) {
        super(time);
        this.actorId = actorId;
        this.location = location != null ? Collections.unmodifiableList(location) : null;
        this.rotation = rotation != null ? Collections.unmodifiableList(rotation) : null;
        this.velocity = velocity != null ? Collections.unmodifiableList(velocity) : null;
        this.properties = properties != null ? Collections.unmodifiableMap(properties) : null;
    }

    @Override
    public String getTypeId() { return TYPE_ID; }

    public String getActorId() { return actorId; }
    public List<Double> getLocation() { return location; }
    public List<Double> getRotation() { return rotation; }
    public List<Double> getVelocity() { return velocity; }
    public Map<String, Object> getProperties() { return properties; }

    @Override
    public int hashCode() {
        return new HashCodeBuilder(23, 53)
            .append(actorId)
            .append(location)
            .append(rotation)
            .append(velocity)
            .append(properties)
            .toHashCode();
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == null) return false;
        if (obj == this) return true;
        if (obj.getClass() != getClass()) return false;
        CarlaActorResponse other = (CarlaActorResponse) obj;
        return new EqualsBuilder()
            .append(this.actorId, other.actorId)
            .append(this.location, other.location)
            .append(this.rotation, other.rotation)
            .append(this.velocity, other.velocity)
            .append(this.properties, other.properties)
            .isEquals();
    }

    @Override
    public String toString() {
        return new ToStringBuilder(this, SHORT_PREFIX_STYLE)
            .appendSuper(super.toString())
            .append("actorId", actorId)
            .append("location", location)
            .append("rotation", rotation)
            .append("velocity", velocity)
            .append("properties", properties)
            .toString();
    }
}


