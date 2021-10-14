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

package org.eclipse.mosaic.interactions.vehicle;

import static org.apache.commons.lang3.builder.ToStringStyle.SHORT_PREFIX_STYLE;

import java.util.List;

import org.eclipse.mosaic.lib.objects.vehicle.VehicleDeparture;
import org.eclipse.mosaic.rti.api.Interaction;

import org.apache.commons.lang3.builder.EqualsBuilder;
import org.apache.commons.lang3.builder.HashCodeBuilder;
import org.apache.commons.lang3.builder.ToStringBuilder;

/**
 * This extension of {@link Interaction} is intended for the Phabmacs + SUMO
 * Controlling Ambassador (PhaSCA) to notify SUMO and Phabmacs of any vehicles
 * that are simulated externally.
 */
public final class VehicleFederateAssignment extends Interaction {

    private static final long serialVersionUID = 1L;

    /**
     * String identifying the type of this interaction.
     */
    public final static String TYPE_ID = createTypeIdentifier(VehicleFederateAssignment.class);

    /**
     * String identifying a simulated vehicle the interaction applies to.
     */
    private final String vehicleId;

    /**
     * String identifying the vehicle type.
     */
    private final String vehicleTypeId;

    /**
     * String identifying, which simulator this interaction is intended for.
     */
    private final String assignedFederateId;

    /**
     * The radius around the ego vehicle for which vehicle data is to be sent (ICOS
     * only).
     */
    private final double surroundingVehiclesRadius;

    /**
     * The vehicle departure information
     */
    private final VehicleDeparture vehicleDeparture;

    /**
     * Specify the applications to be used for this object. If none are specified,
     * none are used.
     */
    public List<String> applications;

    /**
     * Constructor for {@link VehicleFederateAssignment}.
     * 
     * @param time                      The current time at creation of the
     *                                  interaction object.
     * @param vehicleId                 String identifying a simulated vehicle the
     *                                  interaction applies to.
     ** @param assignedFederateId        String identifying which simulator this
     *                                  interaction is intended for.
     * @param surroundingVehiclesRadius The radius around the ego vehicle for which
     *                                  vehicle data is to be sent.
     * @param vehicleTypeId             Id of the vehicle type.
     * @param vehicleDeparture          Vehicle departure information.
     * @param applications              Applications used by this vehicle.
     */
    public VehicleFederateAssignment(long time, String vehicleId, String assignedFederateId,
            double surroundingVehiclesRadius, String vehicleTypeId, VehicleDeparture vehicleDeparture,
            List<String> applications) {
        super(time);
        this.vehicleId = vehicleId;
        this.assignedFederateId = assignedFederateId;
        this.surroundingVehiclesRadius = surroundingVehiclesRadius;
        this.vehicleTypeId = vehicleTypeId;
        this.vehicleDeparture = vehicleDeparture;
        this.applications = applications;
    }

    /**
     * Return vehicle ID.
     * 
     * @return vehicleId
     */
    public String getVehicleId() {
        return vehicleId;
    }

    /**
     * Return vehicle type ID
     * 
     * @return vehicleTypeId
     */
    public String getVehicleTypeId() {
        return vehicleTypeId;
    }

    /**
     * Return the federate this interaction is intended for.
     * 
     * @return assignedFederateId
     */
    public String getAssignedFederate() {
        return assignedFederateId;
    }

    /**
     * Return the radius around the ego vehicle for which vehicle data is to be sent
     * 
     * @return surroundingVehiclesRadius
     */
    public double getSurroundingVehiclesRadius() {
        return surroundingVehiclesRadius;
    }

    /**
     * Return vehicle departure information
     * 
     * @return vehicleDeparture
     */
    public VehicleDeparture getVehicleDeparture() {
        return vehicleDeparture;
    }

    /**
     * Return list of applications that the vehicle uses
     * 
     * @return application list
     */
    public List<String> getVehicleApplications() {
        return applications;
    }

    @Override
    public int hashCode() {
        return new HashCodeBuilder(3, 23).append(vehicleId).append(surroundingVehiclesRadius).append(assignedFederateId)
                .append(vehicleTypeId).append(vehicleDeparture).append(applications).toHashCode();
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

        VehicleFederateAssignment other = (VehicleFederateAssignment) obj;
        return new EqualsBuilder().append(this.vehicleId, other.vehicleId)
                .append(this.assignedFederateId, other.assignedFederateId)
                .append(this.surroundingVehiclesRadius, other.surroundingVehiclesRadius)
                .append(this.vehicleTypeId, other.vehicleTypeId).append(this.vehicleDeparture, other.vehicleDeparture)
                .append(this.applications, other.applications).isEquals();
    }

    @Override
    public String toString() {
        return new ToStringBuilder(this, SHORT_PREFIX_STYLE).appendSuper(super.toString())
                .append("vehicleId", vehicleId).append("assignedFederateId", assignedFederateId)
                .append("surroundingVehiclesRadius", surroundingVehiclesRadius).append("vehicleTypeId", vehicleTypeId)
                .append("vehicleDeparture", vehicleDeparture).append("vehicleApplications", applications).toString();
    }

}
