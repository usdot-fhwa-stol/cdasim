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
import org.eclipse.mosaic.fed.sumo.traci.constants.CommandChangeRouteState;
import org.eclipse.mosaic.fed.sumo.traci.constants.TraciDatatypes;
import org.eclipse.mosaic.fed.sumo.traci.writer.ListTraciWriter;
import org.eclipse.mosaic.fed.sumo.traci.writer.StringTraciWriter;
import org.eclipse.mosaic.rti.api.InternalFederateException;

import java.util.List;

/**
 * This class represents the traci command which adds a route to the scenario.
 */
public class RouteAdd extends AbstractTraciCommand<List<String>> {

    /**
     * Creates a new {@link RouteAdd} traci command, which will
     * add a new route following the given edge-id list.
     * Access needs to be public, because command is called using Reflection.
     *
     * @see <a href="https://sumo.dlr.de/docs/TraCI/Change_Route_State.html">Route State Change</a>
     */
    @SuppressWarnings("WeakerAccess")
    public RouteAdd() {
        super(TraciVersion.LOWEST);

        write()
                .command(CommandChangeRouteState.COMMAND)
                .variable(CommandChangeRouteState.VAR_ADD)
                .writeStringParam()
                .writeByte(TraciDatatypes.STRING_LIST)
                .writeComplex(new ListTraciWriter<>(new StringTraciWriter()));
    }

    /**
     * This method executes the command with the given arguments and adds a new route.
     *
     * @param traciCon Connection to Traci.
     * @param routeId  Id of the route.
     * @param edges    Route consisting of edges.
     * @throws TraciCommandException     if the status code of the response is ERROR. The TraCI connection is still available.
     * @throws InternalFederateException if some serious error occurs during writing or reading. The TraCI connection is shut down.
     */
    public void execute(TraciConnection traciCon, String routeId, List<String> edges) throws TraciCommandException, InternalFederateException {
        super.execute(traciCon, routeId, edges);
    }

    @Override
    protected List<String> constructResult(Status status, Object... objects) {
        return null;
    }
}
