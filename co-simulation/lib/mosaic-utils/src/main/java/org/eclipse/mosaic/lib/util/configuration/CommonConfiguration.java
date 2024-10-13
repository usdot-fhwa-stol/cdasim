/*
 * Copyright (c) 2021 Old Dominion University. All rights reserved.
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 */
package org.eclipse.mosaic.lib.util.configuration;

import java.util.List;

import org.eclipse.mosaic.lib.util.gson.TimeFieldAdapter;

import com.google.gson.annotations.JsonAdapter;

public class CommonConfiguration {
    
    private static final long serialVersionUID = 1479294781446446539L;

    /**
     * The time step that the CARMA message ambassador advances time. The default
     * value is 1000 (1s). Unit: [ms].
     */
    @JsonAdapter(TimeFieldAdapter.LegacyMilliSeconds.class)
    public Long updateInterval = 1000L;

    /**
     * Configruation for CARMA vehicles.
     */
    public List<CommonVehicleConfiguration> Vehicles;

    /**
     * ID of CARMA vehicle that sends external messages.
     */
    public String senderVehicleId;
}
