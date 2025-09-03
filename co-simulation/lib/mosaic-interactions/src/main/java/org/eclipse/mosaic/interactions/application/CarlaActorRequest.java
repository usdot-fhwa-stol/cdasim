/*
 * Copyright (c) 2021 Old Dominion University. All rights reserved.
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
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
public final class CarlaActorRequest extends Interaction {

    private static final long serialVersionUID = 1L;

    public static final String TYPE_ID = createTypeIdentifier(CarlaActorRequest.class);

    public enum Action { CREATE, UPDATE, DESTROY }

    private final Action action;
    private final String actorId;
    private final String actorType; // used for CREATE
    private final List<Double> location; // optional
    private final List<Double> rotation; // optional
    private final List<Double> velocity; // optional
    private final Map<String, Object> properties; // optional

    public CarlaActorRequest(long time, Action action, String actorId, String actorType,
                             List<Double> location, List<Double> rotation, List<Double> velocity,
                             Map<String, Object> properties) {
        super(time);
        this.action = action;
        this.actorId = actorId;
        this.actorType = actorType;
        this.location = location != null ? Collections.unmodifiableList(location) : null;
        this.rotation = rotation != null ? Collections.unmodifiableList(rotation) : null;
        this.velocity = velocity != null ? Collections.unmodifiableList(velocity) : null;
        this.properties = properties != null ? Collections.unmodifiableMap(properties) : null;
    }

    @Override
    public String getTypeId() { return TYPE_ID; }

    public Action getAction() { return action; }
    public String getActorId() { return actorId; }
    public String getActorType() { return actorType; }
    public List<Double> getLocation() { return location; }
    public List<Double> getRotation() { return rotation; }
    public List<Double> getVelocity() { return velocity; }
    public Map<String, Object> getProperties() { return properties; }

    @Override
    public int hashCode() {
        return new HashCodeBuilder(17, 37)
            .append(action)
            .append(actorId)
            .append(actorType)
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
        CarlaActorRequest other = (CarlaActorRequest) obj;
        return new EqualsBuilder()
            .append(this.action, other.action)
            .append(this.actorId, other.actorId)
            .append(this.actorType, other.actorType)
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
            .append("action", action)
            .append("actorId", actorId)
            .append("actorType", actorType)
            .append("location", location)
            .append("rotation", rotation)
            .append("velocity", velocity)
            .append("properties", properties)
            .toString();
    }
}


