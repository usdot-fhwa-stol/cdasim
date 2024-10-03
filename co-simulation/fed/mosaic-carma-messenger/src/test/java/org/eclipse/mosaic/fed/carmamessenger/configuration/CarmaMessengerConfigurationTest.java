package org.eclipse.mosaic.fed.carmamessenger.configuration;

import org.eclipse.mosaic.lib.geo.MutableCartesianPoint;
import org.eclipse.mosaic.lib.geo.MutableGeoPoint;
import org.eclipse.mosaic.lib.util.objects.ObjectInstantiation;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import org.junit.Test;

/**
 * Test for {@link CarmaMessengerConfiguration}.
 *
 */
public class CarmaMessengerConfigurationTest {
    /**
     * Test case using a properly formed json configuration and asserting, that all
     * values were properly deserialized.
     *
     * @throws InstantiationException if configuration couldn't be properly
     *                                deserialized, under normal circumstances this
     *                                should not occur.
     */
    @Test
    public void readConfig_assertProperties() throws InstantiationException {
        // SETUP + RUN
        String validConfig = "/carma_config.json";
        CarmaMessengerConfiguration carmaMessengerConfiguration = getCarmaConfiguration(validConfig);
        // ASSERT
        assertNotNull(carmaMessengerConfiguration); // assert that configuration is created
        assertEquals(Long.valueOf(100L), carmaMessengerConfiguration.updateInterval);
        assertEquals("0", carmaMessengerConfiguration.carmaVehicles.get(0).routeID);
        assertEquals(1, carmaMessengerConfiguration.carmaVehicles.get(0).lane);
        assertEquals(Double.valueOf(0.0D), carmaMessengerConfiguration.carmaVehicles.get(0).position, 0.001);
        assertEquals(Double.valueOf(0.0D), carmaMessengerConfiguration.carmaVehicles.get(0).departSpeed, 0.001);
        assertEquals("vehicle.chevrolet.impala", carmaMessengerConfiguration.carmaVehicles.get(0).vehicleType);
        assertEquals("org.eclipse.mosaic.app.tutorial.VehicleCommunicationApp",
        carmaMessengerConfiguration.carmaVehicles.get(0).applications.get(0));
        assertEquals(new MutableGeoPoint(52.579272059028646, 13.467165499469328),
        carmaMessengerConfiguration.carmaVehicles.get(0).geoPosition);
        assertEquals(new MutableCartesianPoint(501.62, 116.95, 0.0),
        carmaMessengerConfiguration.carmaVehicles.get(0).projectedPosition);
        assertEquals(Double.valueOf(24.204351784500364D), carmaMessengerConfiguration.carmaVehicles.get(0).heading);
        assertEquals(Double.valueOf(0.0), carmaMessengerConfiguration.carmaVehicles.get(0).slope, 0.001);
        assertEquals("carma_0", carmaMessengerConfiguration.senderCarmaVehicleId);
    }

    /**
     * Small helper class, which returns the instantiated object of a
     * json-configuration.
     *
     * @param filePath the path to the configuration.
     * @return the instantiated object.
     * @throws InstantiationException if there was an error during
     *                                deserialization/instantiation.
     */
    private CarmaMessengerConfiguration getCarmaConfiguration(String filePath) throws InstantiationException {
        return new ObjectInstantiation<>(CarmaMessengerConfiguration.class).read(getClass().getResourceAsStream(filePath));
    }
}
