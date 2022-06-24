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
import org.eclipse.mosaic.fed.sumo.traci.constants.TraciDatatypes;
import org.eclipse.mosaic.rti.api.InternalFederateException;

public class VehicleAdd extends AbstractTraciCommand<Void> {

    /**
     * Creates a {@link VehicleAdd} command.
     * Access needs to be public, because command is called using Reflection.
     *
     * @see <a href="https://sumo.dlr.de/docs/TraCI/Change_Vehicle_State.html">Vehicle State Change</a>
     */
    @SuppressWarnings("WeakerAccess")
    public VehicleAdd() {
        super(TraciVersion.HIGHEST);

        write()
                .command(CommandChangeVehicleValue.COMMAND)
                .variable(CommandChangeVehicleValue.VAR_ADD)
                .writeVehicleIdParam() // vehicle id
                .writeByte(TraciDatatypes.COMPOUND) // compound
                .writeInt(14) // number of elements (=14)
                .writeStringParamWithType() // route id
                .writeStringParamWithType() // vehicle type id
                .writeStringWithType("now") // depart time
                .writeStringParamWithType() // depart lane
                .writeStringParamWithType() // depart position
                .writeStringParamWithType() // depart speed
                .writeStringWithType("current") // arrival lane
                .writeStringWithType("max") // arrival position
                .writeStringWithType("current") // arrival speed
                .writeStringWithType("") // from taz
                .writeStringWithType("") // to taz
                .writeStringParamWithType() // line (for public transport)
                .writeIntParamWithType() // person capacity
                .writeIntParamWithType(); // person number

    }

    /**
     * This method executes the command with the given arguments in order to add a vehicle with its specific properties.
     *
     * @param con             Connection to Traci.
     * @param vehicleId       Id of the vehicle.
     * @param routeId         Id of the route.
     * @param vehicleType     Type of the vehicle.
     * @param departLane      Lane of the departure. Possible values: [int] or random", "free", "allowed", "best", "first"
     * @param departPosition  Position of the departure. Possible values: [int] or "random", "free", "base", "last", "random free"
     * @param departSpeed     Speed at departure. Possible Values: [double] or "max", "random"
     * @param line            Line for public transport.
     * @param personCapacity  The amount of persons a vehicle can hold.
     * @param personNumber    The amount of people in the vehicle.
     * @throws TraciCommandException     If the status code of the response is ERROR. The TraCI connection is still available.
     * @throws InternalFederateException If some serious error occurs during writing or reading. The TraCI connection is shut down.
     */
    public void execute(TraciConnection con, String vehicleId, String routeId, String vehicleType,
                        String departLane, String departPosition, String departSpeed,
                        String line, int personCapacity, int personNumber) throws TraciCommandException, InternalFederateException {

        super.execute(con,
                vehicleId,
                routeId,
                vehicleType,
                departLane,
                departPosition,
                departSpeed,
                line,
                personCapacity,
                personNumber
        );
    }

    /**
     * This method executes the command with the given arguments in order to add a vehicle with its specific properties.
     * This overload of {@link #execute} uses some default values
     *
     * @param con            Connection to TraCI.
     * @param vehicleId      Id of the vehicle.
     * @param routeId        Id of the route.
     * @param vehicleType    Type of the vehicle.
     * @param departLane     Lane of the departure. Possible values: [int] or random", "free", "allowed", "best", "first"
     * @param departPosition Position of the departure. Possible values: [int] or "random", "free", "base", "last", "random free"
     * @param departSpeed    Speed at departure. Possible Values: [double] or "max", "random"
     * @throws TraciCommandException     If the status code of the response is ERROR. The TraCI connection is still available.
     * @throws InternalFederateException If some serious error occurs during writing or reading. The TraCI connection is shut down.
     */
    public void execute(TraciConnection con, String vehicleId, String routeId, String vehicleType,
                        String departLane, String departPosition, String departSpeed)
            throws TraciCommandException, InternalFederateException {

        this.execute(con,
                vehicleId,
                routeId,
                vehicleType,
                departLane,
                departPosition,
                departSpeed,
                "", // default for line
                0, // default for personCapacity
                0 // default for personNumber
        );
    }

    @Override
    protected Void constructResult(Status status, Object... objects) {
        return null;
    }
}
