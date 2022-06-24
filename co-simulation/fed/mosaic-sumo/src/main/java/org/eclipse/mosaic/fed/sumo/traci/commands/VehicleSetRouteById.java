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
import org.eclipse.mosaic.fed.sumo.traci.constants.CommandChangeVehicleValue;
import org.eclipse.mosaic.rti.api.InternalFederateException;

/**
 * This class represents the traci command which allows to change the vehicles route to the route with the given id.
 */
public class VehicleSetRouteById extends AbstractTraciCommand<Void> {

    /**
     * Creates a new {@link VehicleSetRouteById} object.
     *
     * @see <a href="https://sumo.dlr.de/docs/TraCI/Change_Vehicle_State.html">Vehicle State Change</a>
     */
    public VehicleSetRouteById() {
        super(TraciVersion.LOWEST);

        write()
                .command(CommandChangeVehicleValue.COMMAND)
                .variable(CommandChangeVehicleValue.VAR_CHANGE_ROUTE_BY_ID)
                .writeVehicleIdParam()
                .writeStringParamWithType(); // route id;
    }

    /**
     * This method executes the command with the given arguments in order to set the vehicles route to the route with the given id.
     *
     * @param traciCon  Connection to Traci.
     * @param vehicleId The Id of the vehicle to change the route.
     * @param routeId   The Id of the route.
     * @throws TraciCommandException     if the status code of the response is ERROR. The TraCI connection is still available.
     * @throws InternalFederateException if some serious error occurs during writing or reading. The TraCI connection is shut down.
     */
    public void execute(TraciConnection traciCon, String vehicleId, String routeId) throws TraciCommandException, InternalFederateException {
        super.execute(traciCon, vehicleId, routeId);
    }

    @Override
    protected Void constructResult(Status status, Object... objects) {
        return null;
    }
}
