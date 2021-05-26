/*
 * Copyright (c) 2020 Fraunhofer FOKUS and others. All rights reserved.
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contact: mosaic@fokus.fraunhofer.de
 */
//create a carla traci request interaction
package org.eclipse.mosaic.interactions.application;

import static org.apache.commons.lang3.builder.ToStringStyle.SHORT_PREFIX_STYLE;

import org.eclipse.mosaic.rti.api.Interaction;

import com.google.common.io.BaseEncoding;
import org.apache.commons.lang3.builder.EqualsBuilder;
import org.apache.commons.lang3.builder.HashCodeBuilder;
import org.apache.commons.lang3.builder.ToStringBuilder;

import javax.annotation.concurrent.Immutable;

/**
 * This extension of {@link Interaction} is used to send a TraCI Command to SUMO
 * TraCI. The request will be handled by TraCI and trigger a
 * {@link CarlaTraciResponse}.
 */
@Immutable
public final class CarlaTraciRequest extends Interaction {

    private static final long serialVersionUID = 1L;

    /**
     * String identifying the type of this interaction.
     */
    public final static String TYPE_ID = createTypeIdentifier(CarlaTraciRequest.class);

    private final int commandLength; // command length
    private final byte[] command;

    /**
     * Constructor for {@link CarlaTraciRequest}.
     *
     * @param time          Timestamp of this interaction, unit: [ns]
     * @param commandLength The number of bytes of the request command.
     * @param command       Byte array representation of the command to be sent from
     *                      CARLA.
     */
    public CarlaTraciRequest(final long time, final int commandLength, final byte[] command) {
        super(time);
        this.commandLength = commandLength;
        this.command = command.clone();
    }

    public byte[] getCommand() {
        return command.clone();
    }

    public int getCommandLength() {
        return commandLength;
    }

    @Override
    public int hashCode() {
        return new HashCodeBuilder(15, 31).append(commandLength).append(command).toHashCode();
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

        CarlaTraciRequest other = (CarlaTraciRequest) obj;
        return new EqualsBuilder().append(this.commandLength, other.commandLength).append(this.command, other.command)
                .isEquals();
    }

    @Override
    public String toString() {
        return new ToStringBuilder(this, SHORT_PREFIX_STYLE).appendSuper(super.toString())
                .append("commandLength", commandLength).append("command", BaseEncoding.base16().encode(command))
                .toString();
    }
}
