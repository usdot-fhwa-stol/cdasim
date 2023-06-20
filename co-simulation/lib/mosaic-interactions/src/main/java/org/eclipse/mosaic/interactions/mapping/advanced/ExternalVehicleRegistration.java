/*
 * Copyright (C) 2019-2022 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.eclipse.mosaic.interactions.mapping.advanced;

import org.apache.commons.lang3.builder.EqualsBuilder;
import org.apache.commons.lang3.builder.HashCodeBuilder;
import org.apache.commons.lang3.builder.ToStringBuilder;
import org.eclipse.mosaic.lib.objects.mapping.VehicleMapping;
import org.eclipse.mosaic.lib.objects.vehicle.VehicleType;
import org.eclipse.mosaic.rti.api.Interaction;

import java.util.List;

import static org.apache.commons.lang3.builder.ToStringStyle.SHORT_PREFIX_STYLE;

/**
 * This interaction is used to define vehicle's controlled externally (i.e. not directly by a MOSAIC application or
 * by a defined behavior and/or route in one of the attached simulators.
 */
public final class ExternalVehicleRegistration extends Interaction {
    private static final long serialVersionUID = 1L;

    /**
     * String identifying the type of this interaction.
     */
    public final static String TYPE_ID = createTypeIdentifier(ExternalVehicleRegistration.class);

    /**
     * The new vehicle.
     */
    private final VehicleMapping vehicleMapping;

    /**
     * Creates a new interaction that informs about a new added vehicle to the simulation.
     *
     * @param time         Timestamp of this interaction, unit: [ns]
     * @param name         vehicle identifier
     * @param group        vehicle group identifier
     * @param applications installed applications of the vehicle
     * @param vehicleType  vehicle type
     */
    public ExternalVehicleRegistration(final long time, final String name, final String group, final List<String> applications, final VehicleType vehicleType) {
        super(time);
        this.vehicleMapping = new VehicleMapping(name, group, applications, vehicleType);
    }

    public VehicleMapping getMapping() {
        return vehicleMapping;
    }

    @Override
    public int hashCode() {
        return new HashCodeBuilder(7, 19)
                .append(vehicleMapping)
                .toHashCode();
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

        ExternalVehicleRegistration rhs = (ExternalVehicleRegistration) obj;
        return new EqualsBuilder()
                .append(this.vehicleMapping, rhs.vehicleMapping)
                .isEquals();
    }

    @Override
    public String toString() {
        return new ToStringBuilder(this, SHORT_PREFIX_STYLE)
                .appendSuper(super.toString())
                .append("vehicleMapping", vehicleMapping)
                .toString();
    }
}
