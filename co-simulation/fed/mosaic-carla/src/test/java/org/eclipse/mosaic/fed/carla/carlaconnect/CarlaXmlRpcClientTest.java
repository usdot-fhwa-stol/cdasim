package org.eclipse.mosaic.fed.carla.carlaconnect;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyList;
import static org.mockito.ArgumentMatchers.anyString;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import java.net.MalformedURLException;
import java.net.URL;
import java.util.Arrays;
import java.util.List;
import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.client.XmlRpcClient;
import org.eclipse.mosaic.interactions.detector.DetectorRegistration;
import org.eclipse.mosaic.lib.geo.CartesianPoint;
import org.eclipse.mosaic.lib.math.Vector3d;
import org.eclipse.mosaic.lib.objects.detector.DetectedObject;
import org.eclipse.mosaic.lib.objects.detector.DetectionType;
import org.eclipse.mosaic.lib.objects.detector.Detector;
import org.eclipse.mosaic.lib.objects.detector.DetectorReferenceLocation;
import org.eclipse.mosaic.lib.objects.detector.DetectorType;
import org.eclipse.mosaic.lib.objects.detector.LocationDataType;
import org.eclipse.mosaic.lib.objects.detector.Orientation;
import org.eclipse.mosaic.lib.objects.detector.Size;
import org.junit.Before;
import org.junit.Test;
import org.mockito.internal.util.reflection.FieldSetter;


public class CarlaXmlRpcClientTest {
    // Mock XmlRpcClient    
    private XmlRpcClient mockClient;
    // CarlaXmlRpcClient under test
    private CarlaXmlRpcClient carlaConnection;

    /**
     * Initialize CarlaXmlRpcClient and setup mock
     * @throws NoSuchFieldException
     * @throws MalformedURLException
     */
    @Before
    public void setup() throws NoSuchFieldException, MalformedURLException, XmlRpcException{
        mockClient = mock(XmlRpcClient.class);
        URL xmlRpcServerUrl = new URL("http://127.0.0.1:8090/RPC2");
        carlaConnection = new CarlaXmlRpcClient(xmlRpcServerUrl);
        when( mockClient.execute(eq("connect"), any(Object[].class))).thenReturn(true);
        FieldSetter.setField(carlaConnection, carlaConnection.getClass().getDeclaredField("client"), mockClient);
        
        // Set mock after initialize since initialize overwrites member
    }

    /**
     * Test to see if createSensor runs without exception
     * @throws XmlRpcException
     */
    @Test
    public void testCreateSensor() throws XmlRpcException {
        // Create Detector Registration
        Detector detector = new Detector(
                    "sensor1",
                    DetectorType.SEMANTIC_LIDAR,
                    new DetectorReferenceLocation( LocationDataType.CARTESIAN,
                    CartesianPoint.xyz(0, 0, 0),
                    new Orientation(0, 0, 0)));
        DetectorRegistration registration = new DetectorRegistration(0, detector, "rsu_2");
        // Create request params
        List<Double> location = Arrays.asList(registration.getDetector().getRef().getLocation().getX(), registration.getDetector().getRef().getLocation().getY(), registration.getDetector().getRef().getLocation().getZ());
        List<Double> orientation = Arrays.asList(registration.getDetector().getRef().getOrientation().getPitch(), registration.getDetector().getRef().getOrientation().getRoll(), registration.getDetector().getRef().getOrientation().getYaw());
        Object[] params = new Object[]{registration.getInfrastructureId(), registration.getDetector().getSensorId(), location, orientation};
        // Tell mock to return sensor ID when following method is called with following parameters
        when( mockClient.execute("create_simulated_semantic_lidar_sensor", params)).thenReturn(registration.getSenderId());
        // Method has no return so verifying that it is successful is just verifying no exception is thrown
        carlaConnection.createSensor(registration);
        // Verify following method was called on mock
        verify( mockClient, times(1)).execute("create_simulated_semantic_lidar_sensor", params);
    }
    @Test
    public void testConnect() throws XmlRpcException, InterruptedException{
        carlaConnection.connect(2);
        verify(mockClient, times(1)).execute( eq("connect"), any(Object[].class));
    }

    @Test
    public void testConnectRetry() {
        try {
            when(mockClient.execute(eq("connect"), any(Object[].class))).thenThrow(new XmlRpcException(""));
            carlaConnection.connect(10);
            verify(mockClient, times(10)).execute( eq("connect"), any(Object[].class));
        }
        catch (Exception e) {
            String message = e.getMessage();
            assertTrue(message.startsWith("Failed to connect to XML RPC Server with config"));
            assertEquals(XmlRpcException.class,e.getClass());
        }
    }

    /**
     * Test GetDectedObjects
     * @throws XmlRpcException
     */
    @Test
    public void testGetDetectedObjects() throws XmlRpcException {
        // Create return JSON String
        String json = "[{"
                + "\"type\":\"CAR\","
                + "\"confidence\":0.7,"
                + "\"sensorId\":\"sensor1\","
                + "\"projString\":\"projection String2\","
                + "\"objectId\":100,"
                + "\"position\":"
                + "{"
                + "\"x\":-1.1,"
                + "\"y\":-2.0,"
                + "\"z\":-3.2"
                + "},"
                + "\"positionCovariance\":[[1.0,0.0,0.0],[1.0,0.0,0.0],[1.0,0.0,0.0]],"
                + "\"velocity\":"
                + "{"
                + "\"x\":1,"
                + "\"y\":1,"
                + "\"z\":1"
                + "},"
                + "\"velocityCovariance\":[[1.0,0.0,0.0],[1.0,0.0,0.0],[1.0,0.0,0.0]],"
                + "\"angularVelocity\":"
                + "{"
                + "\"x\":0.1,"
                + "\"y\":0.2,"
                + "\"z\":0.3"
                + "},"
                + "\"angularVelocityCovariance\":[[1.0,0.0,0.0],[1.0,0.0,0.0],[1.0,0.0,0.0]],"
                + "\"size\":"
                + "{"
                + "\"length\":2.0,"
                + "\"height\":1.0,"
                + "\"width\":0.5"
                + "},"
                + "\"timestamp\":100"
                + "},"
                + "{"
                + "\"type\":\"BUS\","
                + "\"confidence\":0.5,"
                + "\"sensorId\":\"sensor1\","
                + "\"projString\":\"projection String\","
                + "\"objectId\":101,"
                + "\"position\":"
                + "{"
                + "\"x\":1.1,"
                + "\"y\":2.0,"
                + "\"z\":3.2"
                + "},"
                + "\"positionCovariance\":[[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]],"
                + "\"velocity\":"
                + "{"
                + "\"x\":0.0,"
                + "\"y\":0.0,"
                + "\"z\":0.0"
                + "},"
                + "\"velocityCovariance\":[[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]],"
                + "\"angularVelocity\":"
                + "{"
                + "\"x\":0.0,"
                + "\"y\":0.0,"
                + "\"z\":0.0"
                + "},"
                + "\"angularVelocityCovariance\":[[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]],"
                + "\"size\":"
                + "{"
                + "\"length\":0.0,"
                + "\"height\":0.0,"
                + "\"width\":0.0"
                + "},"
                + "\"timestamp\":100"
                + "}"
                + "]";
        // Create request params
        Object[] params = new Object[]{"rsu_1", "sensorId_1"};
        // Tell mock to return sensor ID when following method is called with following parameters
        when( mockClient.execute("get_detected_objects", params)).thenReturn(json);
        // Method has no return so verifying that it is successful is just verifying no exception is thrown
        DetectedObject[] detectedObjects = carlaConnection.getDetectedObjects("rsu_1", "sensorId_1");
        // Verify following method was called on mock
        verify( mockClient, times(1)).execute("get_detected_objects", params);
        // Confirm JSON payload is correctly serialized
        assertEquals(2, detectedObjects.length);
        // Object 1 is CAR
        DetectedObject predictedCar = new DetectedObject(
                DetectionType.CAR,
                0.7,
                "sensor1",
                "projection String2",
                100,
                CartesianPoint.xyz(-1.1, -2, -3.2),
                new Vector3d(1, 1, 1),
                new Vector3d(.1, .2, .3),
                new Size(2, 1, .5),
                100);
        Double[][] covarianceMatrix =  { {1.0, 0.0, 0.0} , {1.0, 0.0, 0.0} , {1.0, 0.0, 0.0}};
        predictedCar.setPositionCovariance(covarianceMatrix);
        predictedCar.setVelocityCovariance(covarianceMatrix);
        predictedCar.setAngularVelocityCovariance(covarianceMatrix);
        assertEquals(predictedCar, detectedObjects[0]);

        DetectedObject predictedBus = new DetectedObject(
            DetectionType.BUS,
            0.5,
            "sensor1",
            "projection String",
            101,
            CartesianPoint.xyz(1.1, 2, 3.2),
            new Vector3d(0, 0, 0),
            new Vector3d(),
            new Size(0, 0, 0),
            100);
        Double[][] bus_covarianceMatrix =  { {0.0, 0.0, 0.0} , {0.0, 0.0, 0.0} , {0.0, 0.0, 0.0}};
        predictedBus.setPositionCovariance(bus_covarianceMatrix);
        predictedBus.setVelocityCovariance(bus_covarianceMatrix);
        predictedBus.setAngularVelocityCovariance(bus_covarianceMatrix);
        assertEquals(predictedBus, detectedObjects[1]);

    }

}
