package org.eclipse.mosaic.lib.CommonUtil.configuration;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

import org.eclipse.mosaic.lib.geo.MutableCartesianPoint;
import org.eclipse.mosaic.lib.geo.MutableGeoPoint;
import org.eclipse.mosaic.lib.util.objects.ObjectInstantiation;

import org.junit.Test;

public class CommonConfigurationTest {

    @Test
    public void readConfig_assertProperties() throws InstantiationException{

        String validConfig = "/carma_config.json";
        CommonConfiguration commonConfiguration = geCommonConfiguration(validConfig);
    }
    
    private CommonConfiguration geCommonConfiguration(String filePath) throws InstantiationException{
        return new ObjectInstantiation<>(CommonConfiguration.class).read(getClass().getResourceAsStream(filePath));
    }
}
