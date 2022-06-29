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

package org.eclipse.mosaic.lib.objects.v2x;

import org.eclipse.mosaic.lib.geo.GeoPoint;
import org.eclipse.mosaic.lib.objects.ToDataOutput;
import org.eclipse.mosaic.lib.util.SerializationUtils;

import java.io.DataOutput;
import java.io.IOException;
import java.io.Serializable;

/**
 * External V2X message content.
 */
public class ExternalV2xContent implements ToDataOutput, Serializable {

    private static final long serialVersionUID = 1L;

    /**
     * Current time stamp of the sending node. Unit: [ns].
     */
    private final long time;

    /**
     * GPS-position of the sending node.
     */
    private final GeoPoint senderPosition;

    /**
     * The message content.
     */
    private final String message;

    /**
     * An extended container which can hold additional information.
     */
    private final String extendedContainer;

    /**
     * Constructor for a {@link ExternalV2xContent}.
     * 
     * @param time           Timestamp when this external V2x message is sent.
     * @param senderPosition Geo position of the sender.
     * @param message        Message content.
     */
    public ExternalV2xContent(final long time, final GeoPoint senderPosition, final String message) {
        this(time, senderPosition, message, null);
    }

    /**
     * Constructor for a {@link ExternalV2xContent}.
     * 
     * @param time              Timestamp when this external V2x message is sent.
     * @param senderPosition    Geo position of the sender.
     * @param message           Message content.
     * @param extendedContainer external container for additional information.
     */
    public ExternalV2xContent(final long time, final GeoPoint senderPosition, final String message,
            final String extendedContainer) {
        this.time = time;
        this.senderPosition = senderPosition;
        this.message = message;
        this.extendedContainer = extendedContainer;
    }

    /**
     * Constructor for a {@link ExternalV2xContent}.
     * 
     * @param externalV2xContent externalV2XContent.
     */
    ExternalV2xContent(final ExternalV2xContent externalV2xContent) {
        this(externalV2xContent.getTime(), externalV2xContent.getSenderPosition(), externalV2xContent.getMessage(),
                externalV2xContent.getExtendedContainer());
    }

    /**
     * Return time stamp of the message.
     * 
     * @return time.
     */
    public long getTime() {
        return time;
    }

    /**
     * Return the position of the sender.
     * 
     * @return the sender position.
     */
    public GeoPoint getSenderPosition() {
        return senderPosition;
    }

    /**
     * Return the message.
     * 
     * @return the message.
     */
    public String getMessage() {
        return message;
    }

    /**
     * Return the additional information in the extendedContainer.
     * 
     * @return extendedContainer.
     */
    public String getExtendedContainer() {
        return extendedContainer;
    }

    /**
     * Write this object to a {@link DataOutput}.
     *
     * @param dataOutput the {@link DataOutput}.
     */
    @Override
    public void toDataOutput(DataOutput dataOutput) throws IOException {

        dataOutput.writeLong(time);
        SerializationUtils.encodeGeoPoint(dataOutput, senderPosition);

        dataOutput.writeBoolean(message != null);
        if (message != null) {
            dataOutput.writeUTF(message);
        }

        dataOutput.writeBoolean(extendedContainer != null);
        if (extendedContainer != null) {
            dataOutput.writeUTF(extendedContainer);
        }
    }
}
