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

package org.eclipse.mosaic.lib.objects.v2x.etsi;

import org.eclipse.mosaic.lib.enums.SensorType;
import org.eclipse.mosaic.lib.geo.GeoPoint;
import org.eclipse.mosaic.lib.geo.GeoPolygon;
import org.eclipse.mosaic.lib.objects.ToDataOutput;
import org.eclipse.mosaic.lib.util.SerializationUtils;

import java.io.DataInput;
import java.io.DataOutput;
import java.io.IOException;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

public class DenmContent implements ToDataOutput, Serializable {

    private static final long serialVersionUID = 1L;

    /**
     * Current time stamp of the sending node. Unit: [ns].
     */
    private final long time;

    /**
     * GPS-position of the sending node
     */
    private final GeoPoint senderPosition;

    /**
     * 
     */
    private final String eventRoadId;

    /**
     * type of the warning
     */
    private final SensorType warningType;

    /**
     * strength of the causing event
     */
    private final int eventStrength;

    /**
     * resulting speed because of the event
     */
    private final float causedSpeed;

    private final float senderDeceleration;

    /**
     * The location of the event
     */
    private final GeoPoint eventLocation;

    /**
     * The area in which the event is occurring
     */
    private final GeoPolygon eventArea;

    /**
     * An extended container which can hold additional information
     */
    private final String extendedContainer;

    public DenmContent(final long time, final GeoPoint senderPosition, final String eventRoadId, final SensorType warningType, final int eventStrength, final float causedSpeed, final float senderDeceleration) {
        this(time, senderPosition, eventRoadId, warningType, eventStrength, causedSpeed, senderDeceleration, null, null, null);
    }

    public DenmContent(final long time, final GeoPoint senderPosition, final String eventRoadId, final SensorType warningType, final int eventStrength, final float causedSpeed, final float senderDeceleration, final GeoPoint eventLocation, final GeoPolygon eventArea, final String extendedContainer) {
        this.time = time;
        this.senderPosition = senderPosition;
        this.eventRoadId = eventRoadId;
        this.warningType = warningType;
        this.eventStrength = eventStrength;
        this.causedSpeed = causedSpeed;
        this.senderDeceleration = senderDeceleration;
        this.eventLocation = eventLocation;
        this.eventArea = eventArea;
        this.extendedContainer = extendedContainer;
    }

    public DenmContent(DataInput din) throws IOException {
        this.time = din.readLong();
        this.senderPosition = SerializationUtils.decodeGeoPoint(din);

        if (din.readBoolean()) {
            this.eventRoadId = din.readUTF();
        } else {
            this.eventRoadId = null;
        }

        this.warningType = SensorType.fromId(din.readByte());
        this.eventStrength = din.readByte();
        this.causedSpeed = din.readFloat();
        this.senderDeceleration = din.readFloat();

        if (din.readBoolean()) {
            this.eventLocation = SerializationUtils.decodeGeoPoint(din);
        } else {
            this.eventLocation = null;
        }

        int listLength = din.readInt();
        List<GeoPoint> coordinates = new ArrayList<>();
        for (int i = 0; i < listLength; i++) {
            coordinates.add(SerializationUtils.decodeGeoPoint(din));
        }

        if (!coordinates.isEmpty()) {
            this.eventArea = new GeoPolygon(coordinates);
        } else {
            this.eventArea = null;
        }

        if (din.readBoolean()) {
            this.extendedContainer = din.readUTF();
        } else {
            this.extendedContainer = null;
        }

    }

    DenmContent(final DenmContent denm) {
        this(denm.getTime(), denm.getSenderPosition(), denm.getEventRoadId(), denm.getWarningType(), denm.getEventStrength(), denm.getCausedSpeed(), denm.getSenderDeceleration(), denm.getEventLocation(), denm.getEventArea(), denm.getExtendedContainer());
    }

    public long getTime() {
        return time;
    }

    public GeoPoint getSenderPosition() {
        return senderPosition;
    }

    public String getEventRoadId() {
        return eventRoadId;
    }

    public SensorType getWarningType() {
        return warningType;
    }

    public int getEventStrength() {
        return eventStrength;
    }

    public float getCausedSpeed() {
        return causedSpeed;
    }

    public float getSenderDeceleration() {
        return senderDeceleration;
    }

    public GeoPoint getEventLocation() {
        return eventLocation;
    }

    public GeoPolygon getEventArea() {
        return eventArea;
    }

    public String getExtendedContainer() {
        return extendedContainer;
    }

    @Override
    public void toDataOutput(DataOutput dataOutput) throws IOException {
        dataOutput.writeLong(time);
        SerializationUtils.encodeGeoPoint(dataOutput, senderPosition);

        dataOutput.writeBoolean(eventRoadId != null);
        if (eventRoadId != null) {
            dataOutput.writeUTF(eventRoadId);
        }
        dataOutput.writeByte(warningType.id);
        dataOutput.writeByte(eventStrength);
        dataOutput.writeFloat(causedSpeed);
        dataOutput.writeFloat(senderDeceleration);
        dataOutput.writeBoolean(eventLocation != null);
        if (eventLocation != null) {
            SerializationUtils.encodeGeoPoint(dataOutput, eventLocation);
        }
        int eventAreaSize = this.eventArea == null ? 0 : this.eventArea.getVertices().size();
        dataOutput.writeInt(eventAreaSize);
        if (eventArea != null) {
            for (GeoPoint vertice: eventArea.getVertices()) {
                SerializationUtils.encodeGeoPoint(dataOutput, vertice);
            }
        }

        dataOutput.writeBoolean(extendedContainer != null);
        if (extendedContainer != null) {
            dataOutput.writeUTF(extendedContainer);
        }
    }
}
