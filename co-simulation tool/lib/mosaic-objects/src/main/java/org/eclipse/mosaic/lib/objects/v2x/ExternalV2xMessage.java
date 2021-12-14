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

import javax.annotation.Nonnull;
import javax.annotation.concurrent.Immutable;

/**
 * External Communication V2X Message.
 */
@Immutable
public class ExternalV2xMessage extends V2xMessage {

    private static final long serialVersionUID = 1L;

    /**
     * The encoded message.
     */
    private final EncodedPayload payload;

    /**
     * The content of V2X message.
     */
    private final ExternalV2xContent externalV2xContent;

    /**
     * The constructor of ExternalV2xMessage.
     * 
     * @param messageRouting     The message routing object contains a source and a
     *                           target address for a message to be routed.
     * @param externalV2xContent The message content.
     */
    public ExternalV2xMessage(final MessageRouting messageRouting, ExternalV2xContent externalV2xContent) {
        super(messageRouting);
        this.externalV2xContent = externalV2xContent;

        // The minimal assumed length of this message payload [in bytes].
        long minimalLength = 200;

        this.payload = new EncodedPayload(externalV2xContent, minimalLength);
    }

    /**
     * Constructor for ExternalV2xMessage
     * 
     * @param messageRouting     The message routing object contains a source and a
     *                           target address for a message to be routed.
     * @param externalV2xMessage The external V2X message.
     */
    public ExternalV2xMessage(final MessageRouting messageRouting, final ExternalV2xMessage externalV2xMessage) {
        this(messageRouting, externalV2xMessage.externalV2xContent);
    }

    /**
     * Return the encoded payload.
     * 
     * @return payload.
     */
    @Override
    @Nonnull
    public EncodedPayload getPayLoad() {
        return payload;
    }

    /**
     * Return the message time.
     *
     * @return the time. Unit: [ns].
     */
    public long getTime() {
        return this.externalV2xContent.getTime();
    }

    /**
     * Return the coordinates of the sending node.
     *
     * @return the coordinate.
     */
    public GeoPoint getSenderPosition() {
        return this.externalV2xContent.getSenderPosition();
    }

    /**
     * Return the content of message.
     *
     * @return the message.
     */
    public String getMessage() {
        return this.externalV2xContent.getMessage();
    }

    /**
     * Return the extended container.
     *
     * @return the additional information.
     */
    public String getExtendedContainer() {
        return this.externalV2xContent.getExtendedContainer();
    }

}
