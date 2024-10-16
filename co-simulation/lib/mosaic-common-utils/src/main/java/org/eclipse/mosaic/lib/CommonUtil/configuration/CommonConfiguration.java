/*
 * Copyright (C) 2023 LEIDOS.
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
package org.eclipse.mosaic.lib.CommonUtil.configuration;

import java.util.List;

import org.eclipse.mosaic.lib.util.gson.TimeFieldAdapter;

import com.google.gson.annotations.JsonAdapter;

public class CommonConfiguration<V extends CommonVehicleConfiguration> {
    
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
    public List<V> Vehicles;

    /**
     * ID of CARMA vehicle that sends external messages.
     */
    public String senderVehicleId;
}
