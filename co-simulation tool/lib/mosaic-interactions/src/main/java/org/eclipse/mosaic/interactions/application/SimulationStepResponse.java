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
import com.google.common.io.BaseEncoding;
import org.apache.commons.lang3.builder.EqualsBuilder;
import org.apache.commons.lang3.builder.HashCodeBuilder;
import org.apache.commons.lang3.builder.ToStringBuilder;

import javax.annotation.concurrent.Immutable;

/**
 * This extension of {@link Interaction} holds the response for a
 * {@link SimulationStep}. It is sent by the CARLA and will usually be handled
 * by the CarlaAmbassador that sent the request.
 */
@Immutable
public final class SimulationStepResponse extends Interaction {

    private static final long serialVersionUID = 1L;

    /**
     * String identifying the type of this interaction.
     */
    public final static String TYPE_ID = createTypeIdentifier(SimulationStepResponse.class);

    /**
     * The response of SUMO.
     */
    private final byte[] resultRawMessage;
    private final int resultLength;// resultLength

    /**
     * Constructor for {@link SimulationStepResponse}.
     *
     * @param time             Timestamp of this interaction, unit: [ns]
     * @param carlaTraciResult The result of the request stored as
     *                         {@link resultRawMessage}.
     */
    public SimulationStepResponse(final long time, final int resultLength, final byte[] resultRawMessage) {
        super(time);
        this.resultLength = resultLength;
        this.resultRawMessage = resultRawMessage.clone();
    }

    public byte[] getResult() {
        return resultRawMessage;
    }

    public int getResultLength() {
        return resultLength;// resultRawMessage.length;
    }

    @Override
    public int hashCode() {
        return new HashCodeBuilder(7, 97).append(resultLength).append(resultRawMessage).toHashCode();
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

        SimulationStepResponse other = (SimulationStepResponse) obj;
        return new EqualsBuilder().append(this.resultLength, other.resultLength)
                .append(this.resultRawMessage, other.resultRawMessage).isEquals();
    }

    @Override
    public String toString() {
        return new ToStringBuilder(this, SHORT_PREFIX_STYLE).appendSuper(super.toString())
                .append("resultLength", resultLength).append("result", BaseEncoding.base16().encode(resultRawMessage))
                .toString();
    }
}
