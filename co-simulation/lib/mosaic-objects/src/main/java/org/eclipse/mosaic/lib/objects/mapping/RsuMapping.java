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

package org.eclipse.mosaic.lib.objects.mapping;

import org.eclipse.mosaic.lib.geo.CartesianPoint;
import org.eclipse.mosaic.lib.geo.GeoPoint;

import java.util.List;
import javax.annotation.concurrent.Immutable;

/**
 * A Road Side Unit (RSU) that is equipped with applications.
 */
@Immutable
public final class RsuMapping extends UnitMapping {

    private static final long serialVersionUID = 1L;

    private final GeoPoint position;

    private final CartesianPoint projectedPosition;

    /**
     * Creates a new RSU.
     * @param name The name of the RSU.
     * @param group The group name of the RSU.
     * @param applications The list of applications the RSU is equipped with.
     * @param position The static position of the RSU.
     */
    public RsuMapping(final String name, final String group, final List<String> applications, final GeoPoint position) {
        this(name, group, applications, position, null);
    }

    /**
     * Creates a new RSU.
     *
     * @param name The name of the RSU.
     * @param group The group name of the RSU.
     * @param applications The list of applications the RSU is equipped with.
     * @param position The static geographic position of the RSU.
     * @param projectedPosition The static position of the RSU in the scenario's Cartesian frame, or {@code null}
     *                          if it has not been provided by the registering federate.
     */
    public RsuMapping(final String name, final String group, final List<String> applications,
                      final GeoPoint position, final CartesianPoint projectedPosition) {
        super(name, group, applications);
        this.position = position;
        this.projectedPosition = projectedPosition;
    }

    public GeoPoint getPosition() {
        return position;
    }

    public CartesianPoint getProjectedPosition() {
        return projectedPosition;
    }
}
