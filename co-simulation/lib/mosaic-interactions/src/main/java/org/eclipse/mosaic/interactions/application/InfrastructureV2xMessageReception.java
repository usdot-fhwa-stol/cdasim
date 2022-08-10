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

import org.eclipse.mosaic.rti.api.Interaction;

import org.apache.commons.lang3.builder.EqualsBuilder;
import org.apache.commons.lang3.builder.HashCodeBuilder;
import org.apache.commons.lang3.builder.ToStringBuilder;

import javax.annotation.Nullable;

/**
 * This extension of {@link Interaction} is intended to be used to send V2X
 * messages received by a RSU to Infrastructure message ambassador.
 *
 */
public final class InfrastructureV2xMessageReception extends Interaction {

    private static final long serialVersionUID = 1L;

    /**
     * String identifying the type of this interaction.
     */
    public final static String TYPE_ID = createTypeIdentifier(InfrastructureV2xMessageReception.class);

    /**
     * The identity of the RSU that receives this message.
     */
    private final String receiverID;

    /**
     * message content.
     */
    private final String message;

    /**
     * Constructor for a {@link InfrastructureV2xMessageReception}.
     * 
     * @param time       Timestamp of this interaction, unit:[ns].
     * @param message    Message content.
     * @param receiverID Identifier of the RSU which receives this V2X message.
     */
    public InfrastructureV2xMessageReception(long time, @Nullable String message, String receiverID) {
        super(time);
        this.message = message;
        this.receiverID = receiverID;
    }

    /**
     * Return message content.
     *
     * @return the message content.
     */
    public String getMessage() {
        return message;
    }

    /**
     * Return the ID of receiver.
     * 
     * @return ID of RSU which receives V2X message.
     */
    public final String getReceiverID() {
        return receiverID;
    }

    @Override
    public int hashCode() {
        return new HashCodeBuilder(7, 97).append(message).append(receiverID).toHashCode();
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

        InfrastructureV2xMessageReception other = (InfrastructureV2xMessageReception) obj;
        return new EqualsBuilder().append(this.message, other.message).append(this.receiverID, other.receiverID)
                .isEquals();
    }

    @Override
    public String toString() {
        return new ToStringBuilder(this, SHORT_PREFIX_STYLE).appendSuper(super.toString()).append("message", message)
                .append("receiverID", receiverID).toString();
    }
}
