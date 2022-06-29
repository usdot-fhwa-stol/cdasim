/*
 * Copyright (C) 2019-2021 LEIDOS.
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

package org.eclipse.mosaic.fed.carma.ambassador;

/**
 * Transformer for CARMA Vehicle IDs as reported by vehicles into the ID's used for the NS-3 nodes
 *
 * TODO: Placeholder for implementation pending updates to CARLA-MOSAIC integration
 */
public class CarmaIdTransformer {

    /**
     * Translate a CARMA Platform Host ID into the corresponding MOSAIC ID for the vehicle
     * @param carmaId The CARMA Platform Host ID from VehicleConfigParams.yml
     * @return The internal MOSAIC ID for the vehicle
     */
    public String carmaIdToNs3Id(String carmaId) {
        return "";
    }

    /**
     * Translate a MOSAIC internal ID to the CARMA platform Host ID
     * @param ns3Id The internal MOSAIC ID for the vehicle
     * @return The CARMA Platform Host ID for the vehicle
     */
    public String ns3IdToCarmaId(String ns3Id) {
        return "";
    }
}
