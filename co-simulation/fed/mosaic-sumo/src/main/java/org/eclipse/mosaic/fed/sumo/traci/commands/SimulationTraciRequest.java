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

package org.eclipse.mosaic.fed.sumo.traci.commands;

import org.eclipse.mosaic.fed.sumo.traci.AbstractTraciCommand;
import org.eclipse.mosaic.fed.sumo.traci.TraciCommandException;
import org.eclipse.mosaic.fed.sumo.traci.TraciConnection;
import org.eclipse.mosaic.fed.sumo.traci.TraciVersion;
import org.eclipse.mosaic.fed.sumo.traci.complex.Status;
import org.eclipse.mosaic.fed.sumo.traci.reader.ByteArrayReader;
import org.eclipse.mosaic.fed.sumo.traci.writer.ByteArrayTraciWriter;
import org.eclipse.mosaic.lib.objects.traffic.SumoTraciResult;
import org.eclipse.mosaic.rti.api.InternalFederateException;

/**
 * Implementation for a byte array message from the simulation.
 */
public class SimulationTraciRequest extends AbstractTraciCommand<SumoTraciResult> {

    private String msgId = null;

    /**
     * Creates a new {@link SimulationTraciRequest} object.
     */
    public SimulationTraciRequest() {
        super(TraciVersion.LOWEST);

        write()
                .writeComplex(new ByteArrayTraciWriter());

        read()
                .readComplex(new ByteArrayReader());
    }

    public SumoTraciResult execute(TraciConnection traciConnection, String messageId, byte[] messageContent)
            throws TraciCommandException, InternalFederateException {
        this.msgId = messageId;
        try {
            return super.executeAndReturn(traciConnection, (Object) messageContent).orElse(
                    new SumoTraciResult(
                            messageId,
                            (byte) Status.STATUS_ERR,
                            "TraCI return value couldn't be extracted.",
                            new byte[0]
                    )
            );
        } catch (TraciCommandException e) {
            throw e;
        }
    }

    @Override
    protected SumoTraciResult constructResult(Status status, Object... objects) {
        return new SumoTraciResult(msgId, status.getResultType(), status.getDescription(), (byte[]) objects[0]);
    }
}
