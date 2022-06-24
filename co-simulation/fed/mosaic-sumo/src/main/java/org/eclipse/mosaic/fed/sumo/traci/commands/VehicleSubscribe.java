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

import static org.eclipse.mosaic.fed.sumo.traci.constants.CommandRetrieveVehicleState.VAR_ACCELERATION;
import static org.eclipse.mosaic.fed.sumo.traci.constants.CommandRetrieveVehicleState.VAR_ANGLE;
import static org.eclipse.mosaic.fed.sumo.traci.constants.CommandRetrieveVehicleState.VAR_DISTANCE;
import static org.eclipse.mosaic.fed.sumo.traci.constants.CommandRetrieveVehicleState.VAR_EMISSIONS_CO;
import static org.eclipse.mosaic.fed.sumo.traci.constants.CommandRetrieveVehicleState.VAR_EMISSIONS_CO2;
import static org.eclipse.mosaic.fed.sumo.traci.constants.CommandRetrieveVehicleState.VAR_EMISSIONS_ELECTRICITY;
import static org.eclipse.mosaic.fed.sumo.traci.constants.CommandRetrieveVehicleState.VAR_EMISSIONS_FUEL;
import static org.eclipse.mosaic.fed.sumo.traci.constants.CommandRetrieveVehicleState.VAR_EMISSIONS_HC;
import static org.eclipse.mosaic.fed.sumo.traci.constants.CommandRetrieveVehicleState.VAR_EMISSIONS_NOX;
import static org.eclipse.mosaic.fed.sumo.traci.constants.CommandRetrieveVehicleState.VAR_EMISSIONS_PMX;
import static org.eclipse.mosaic.fed.sumo.traci.constants.CommandRetrieveVehicleState.VAR_LANE_INDEX;
import static org.eclipse.mosaic.fed.sumo.traci.constants.CommandRetrieveVehicleState.VAR_LANE_POSITION;
import static org.eclipse.mosaic.fed.sumo.traci.constants.CommandRetrieveVehicleState.VAR_LATERAL_LANE_POSITION;
import static org.eclipse.mosaic.fed.sumo.traci.constants.CommandRetrieveVehicleState.VAR_LEADER;
import static org.eclipse.mosaic.fed.sumo.traci.constants.CommandRetrieveVehicleState.VAR_MIN_GAP;
import static org.eclipse.mosaic.fed.sumo.traci.constants.CommandRetrieveVehicleState.VAR_POSITION_3D;
import static org.eclipse.mosaic.fed.sumo.traci.constants.CommandRetrieveVehicleState.VAR_ROAD_ID;
import static org.eclipse.mosaic.fed.sumo.traci.constants.CommandRetrieveVehicleState.VAR_ROUTE_ID;
import static org.eclipse.mosaic.fed.sumo.traci.constants.CommandRetrieveVehicleState.VAR_SIGNAL_STATES;
import static org.eclipse.mosaic.fed.sumo.traci.constants.CommandRetrieveVehicleState.VAR_SLOPE;
import static org.eclipse.mosaic.fed.sumo.traci.constants.CommandRetrieveVehicleState.VAR_SPEED;
import static org.eclipse.mosaic.fed.sumo.traci.constants.CommandRetrieveVehicleState.VAR_STOP_STATE;

import org.eclipse.mosaic.fed.sumo.config.CSumo;
import org.eclipse.mosaic.fed.sumo.traci.AbstractTraciCommand;
import org.eclipse.mosaic.fed.sumo.traci.SumoVersion;
import org.eclipse.mosaic.fed.sumo.traci.TraciCommandException;
import org.eclipse.mosaic.fed.sumo.traci.TraciConnection;
import org.eclipse.mosaic.fed.sumo.traci.TraciVersion;
import org.eclipse.mosaic.fed.sumo.traci.complex.Status;
import org.eclipse.mosaic.fed.sumo.traci.constants.CommandVariableSubscriptions;
import org.eclipse.mosaic.fed.sumo.traci.constants.SumoVar;
import org.eclipse.mosaic.fed.sumo.traci.constants.TraciDatatypes;
import org.eclipse.mosaic.rti.TIME;
import org.eclipse.mosaic.rti.api.InternalFederateException;

import com.google.common.collect.Lists;

import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.function.Predicate;

/**
 * This class represents the traci command which allows to subscribe the vehicle to the application.
 * Several options for vehicle subscription are implemented in this class.
 */
public class VehicleSubscribe extends AbstractTraciCommand<Void> {

    /**
     * A helper class in order to extract the subscription parameters from the sumo configuration file.
     *
     * @param sumoConfiguration SUMO configuration file.
     * @return Collection of the extracted parameters.
     */
    private static Collection<SumoVar> extractSubscriptionCodesFromConfiguration(CSumo sumoConfiguration) {
        final Collection<String> subscriptionCategories;
        if (sumoConfiguration == null || sumoConfiguration.subscriptions == null) {
            subscriptionCategories =
                    Lists.newArrayList(
                            CSumo.SUBSCRIPTION_SIGNALS,
                            CSumo.SUBSCRIPTION_EMISSIONS,
                            CSumo.SUBSCRIPTION_ROAD_POSITION
                    );
        } else {
            subscriptionCategories = Lists.newArrayList(sumoConfiguration.subscriptions);
        }

        /* mandatory subscription codes */
        final List<SumoVar> subscriptionCodes = Lists.newArrayList(
                VAR_SPEED, VAR_ANGLE, VAR_SLOPE, VAR_POSITION_3D, VAR_ROUTE_ID, VAR_STOP_STATE, VAR_ACCELERATION
        );

        if (subscriptionCategories.contains(CSumo.SUBSCRIPTION_ROAD_POSITION)) {
            Collections.addAll(
                    subscriptionCodes,
                    VAR_ROAD_ID,
                    VAR_LANE_INDEX,
                    VAR_LANE_POSITION,
                    VAR_DISTANCE,
                    VAR_LATERAL_LANE_POSITION);
        }

        if (subscriptionCategories.contains(CSumo.SUBSCRIPTION_EMISSIONS)) {
            Collections.addAll(
                    subscriptionCodes,
                    VAR_EMISSIONS_CO2,
                    VAR_EMISSIONS_CO,
                    VAR_EMISSIONS_HC,
                    VAR_EMISSIONS_PMX,
                    VAR_EMISSIONS_NOX,
                    VAR_EMISSIONS_FUEL,
                    VAR_EMISSIONS_ELECTRICITY
            );
        }

        if (subscriptionCategories.contains(CSumo.SUBSCRIPTION_SIGNALS)) {
            Collections.addAll(subscriptionCodes, VAR_SIGNAL_STATES);
        }
        if (subscriptionCategories.contains(CSumo.SUBSCRIPTION_LEADER)) {
            Collections.addAll(subscriptionCodes, VAR_LEADER, VAR_MIN_GAP);
        }
        return subscriptionCodes;
    }

    /**
     * Creates a new {@link VehicleSubscribe} object.
     * Access needs to be public, because command is called using Reflection.
     *
     * @param traciConnection Connection to Traci.
     * @see <a href="https://sumo.dlr.de/docs/TraCI/Object_Variable_Subscription.html">Variable Subscription</a>
     */
    @SuppressWarnings("WeakerAccess")
    public VehicleSubscribe(TraciConnection traciConnection) {
        this(traciConnection, (CSumo) null);
    }

    /**
     * Creates a new {@link VehicleSubscribe} object.
     *
     * @param traciConnection   Connection to Traci.
     * @param sumoConfiguration The sumo configuration file.
     */
    public VehicleSubscribe(TraciConnection traciConnection, CSumo sumoConfiguration) {
        this(traciConnection, extractSubscriptionCodesFromConfiguration(sumoConfiguration));
    }

    /**
     * Creates a new {@link VehicleSubscribe} object.
     * Access needs to be public, because command is called using Reflection.
     *
     * @param traciConnection   Connection to Traci.
     * @param subscriptionCodes The parameters for an applicable configuration.
     */
    @SuppressWarnings("WeakerAccess")
    public VehicleSubscribe(TraciConnection traciConnection, Collection<SumoVar> subscriptionCodes) {
        super(TraciVersion.LOWEST);

        final SumoVersion currentVersion = traciConnection.getCurrentVersion();
        final Predicate<SumoVar> isSubscriptionVarApplicable =
                subscriptionVar -> subscriptionVar.isAvailable(currentVersion.getTraciVersion());

        final int subscriptionSize = (int) subscriptionCodes.stream().filter(isSubscriptionVarApplicable).count();

        TraciCommandWriterBuilder write = write()
                .command(CommandVariableSubscriptions.COMMAND_SUBSCRIBE_VEHICLE_VALUES)
                .writeDoubleParam() // start time
                .writeDoubleParam() // end time
                .writeVehicleIdParam()
                .writeByte(subscriptionSize);

        subscriptionCodes.stream()
                .filter(isSubscriptionVarApplicable)
                .forEach(subscriptionVar -> {

                    write.writeByte(subscriptionVar.var);

                    if (subscriptionVar instanceof SumoVar.WithParam) {
                        write.writeByte(TraciDatatypes.DOUBLE);
                        write.writeDouble(((SumoVar.WithParam) subscriptionVar).getValue());
                    }
                });

        read()
                .expectByte(CommandVariableSubscriptions.RESPONSE_SUBSCRIBE_VEHICLE_VALUES)
                .skipString()
                .expectByte(subscriptionSize)
                .skipRemaining();
    }

    /**
     * This method executes the command with the given arguments in order to subscribe the vehicle to the application.
     *
     * @param traciCon  Connection to Traci.
     * @param vehicleId The Id of the Vehicle.
     * @param startTime The time to subscribe the vehicle.
     * @param endTime   The end time of the subscription of the vehicle in the application.
     * @throws TraciCommandException     if the status code of the response is ERROR. The TraCI connection is still available.
     * @throws InternalFederateException if some serious error occurs during writing or reading. The TraCI connection is shut down.
     */
    public void execute(TraciConnection traciCon, String vehicleId, long startTime, long endTime) throws TraciCommandException, InternalFederateException {
        super.execute(traciCon, ((double) startTime) / TIME.SECOND, ((double) endTime) / TIME.SECOND, vehicleId);
    }

    @Override
    protected Void constructResult(Status status, Object... objects) {
        return null;
    }
}
