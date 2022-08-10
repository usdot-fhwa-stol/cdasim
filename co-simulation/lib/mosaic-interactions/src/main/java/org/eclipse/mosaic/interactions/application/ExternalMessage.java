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
 * This extension of {@link Interaction} is intended to be used to send external
 * messages from outside MOSAIC to MOSAIC applications.
 *
 */
public final class ExternalMessage extends Interaction {

    private static final long serialVersionUID = 1L;

    /**
     * String identifying the type of this interaction.
     */
    public final static String TYPE_ID = createTypeIdentifier(ExternalMessage.class);

    /**
     * The sender ID.
     */
    private final String senderID;

    /**
     * External message content.
     */
    private final String message;

    /**
     * Constructor for a {@link ExternalMessage}.
     * 
     * @param time     Timestamp of this interaction, unit:[ns].
     * @param message  Message content.
     * @param senderID Identifier of the entity which sends this message.
     * 
     */
    public ExternalMessage(long time, @Nullable String message, String senderID) {
        super(time);
        this.message = message;
        this.senderID = senderID;
    }

    /**
     * Return external message content.
     *
     * @return the message content.
     */
    public String getMessage() {
        return message;
    }

    /**
     * Return the ID of sender.
     * 
     * @return ID of entity which sends this message.
     */
    public final String getSenderID() {
        return senderID;
    }

    @Override
    public int hashCode() {
        return new HashCodeBuilder(7, 97).append(message).append(senderID).toHashCode();
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

        ExternalMessage other = (ExternalMessage) obj;
        return new EqualsBuilder().append(this.message, other.message).append(this.senderID, other.senderID).isEquals();
    }

    @Override
    public String toString() {
        return new ToStringBuilder(this, SHORT_PREFIX_STYLE).appendSuper(super.toString()).append("message", message)
                .append("senderID", senderID).toString();
    }
}
