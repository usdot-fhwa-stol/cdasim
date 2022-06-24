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

package org.eclipse.mosaic.fed.sumo.traci.reader;

import org.eclipse.mosaic.fed.sumo.traci.AbstractTraciResultReader;
import org.eclipse.mosaic.fed.sumo.traci.complex.AbstractSubscriptionResult;
import org.eclipse.mosaic.fed.sumo.traci.constants.CommandVariableSubscriptions;

import java.io.DataInputStream;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

public class AllSubscriptionsTraciReader extends AbstractTraciResultReader<AbstractSubscriptionResult> {

    private final Map<Integer, AbstractSubscriptionTraciReader<?>> childReader = new HashMap<>();

    /**
     * Creates a new {@link AllSubscriptionsTraciReader} object.
     */
    public AllSubscriptionsTraciReader() {
        super(null);

        childReader.put(CommandVariableSubscriptions.RESPONSE_SUBSCRIBE_VEHICLE_VALUES, new VehicleSubscriptionTraciReader());
        childReader.put(CommandVariableSubscriptions.RESPONSE_SUBSCRIBE_INDUCTION_LOOP_VALUES, new InductionLoopSubscriptionTraciReader());
        childReader.put(CommandVariableSubscriptions.RESPONSE_SUBSCRIBE_LANE_AREA_VALUES, new LaneAreaSubscriptionTraciReader());
        childReader.put(CommandVariableSubscriptions.RESPONSE_SUBSCRIBE_TRAFFIC_LIGHT_VALUES, new TrafficLightSubscriptionReader());
    }

    @Override
    protected AbstractSubscriptionResult readFromStream(DataInputStream in) throws IOException {
        int type = readUnsignedByte(in);

        AbstractSubscriptionTraciReader<?> subscriptionReader = childReader.get(type);
        AbstractSubscriptionResult result = subscriptionReader.read(in, totalBytesLeft - numBytesRead);
        this.numBytesRead += subscriptionReader.getNumberOfBytesRead();
        return result;
    }
}
