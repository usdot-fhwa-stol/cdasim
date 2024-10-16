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

import java.io.InputStreamReader;
import java.lang.reflect.Type;

import org.eclipse.mosaic.lib.geo.MutableCartesianPoint;
import org.eclipse.mosaic.lib.geo.MutableGeoPoint;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import org.junit.Test;

import com.google.common.reflect.TypeToken;
import com.google.gson.Gson;

public class CommonConfigurationTest {

    @Test
    public void readConfig_assertProperties() throws InstantiationException{

        String validConfig = "/carma_config.json";
        CommonConfiguration<CommonVehicleConfiguration> commonConfiguration = geCommonConfiguration(validConfig);

        assertNotNull(commonConfiguration); // assert that configuration is created
        assertEquals(Long.valueOf(100L), commonConfiguration.updateInterval);
        assertEquals("0", commonConfiguration.Vehicles.get(0).routeID);
        assertEquals(1, commonConfiguration.Vehicles.get(0).lane);
        assertEquals(Double.valueOf(0.0D), commonConfiguration.Vehicles.get(0).position, 0.001);
        assertEquals(Double.valueOf(0.0D), commonConfiguration.Vehicles.get(0).departSpeed, 0.001);
        assertEquals("vehicle.chevrolet.impala", commonConfiguration.Vehicles.get(0).vehicleType);
        assertEquals("org.eclipse.mosaic.app.tutorial.VehicleCommunicationApp",
        commonConfiguration.Vehicles.get(0).applications.get(0));
        assertEquals(new MutableGeoPoint(52.579272059028646, 13.467165499469328),
        commonConfiguration.Vehicles.get(0).geoPosition);
        assertEquals(new MutableCartesianPoint(501.62, 116.95, 0.0),
        commonConfiguration.Vehicles.get(0).projectedPosition);
        assertEquals(Double.valueOf(24.204351784500364D), commonConfiguration.Vehicles.get(0).heading);
        assertEquals(Double.valueOf(0.0), commonConfiguration.Vehicles.get(0).slope, 0.001);
        assertEquals("carma_0", commonConfiguration.senderVehicleId);
    }
    
    private CommonConfiguration geCommonConfiguration(String filePath) throws InstantiationException{
        Type type = new TypeToken<CommonConfiguration<CommonVehicleConfiguration>>() {}.getType();
        return new Gson().fromJson(new InputStreamReader(getClass().getResourceAsStream(filePath)), type);       
    }
}
