/*
*Copyright (C) 2023 LEIDOS.
*
*Licensed under the Apache License, Version 2.0 (the "License"); you may not
*use this file except in compliance with the License. You may obtain a copy of
*the License at
*
*http://www.apache.org/licenses/LICENSE-2.0
*
*Unless required by applicable law or agreed to in writing, software
*distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
*WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
*License for the specific language governing permissions and limitations under
*the License.
*/
package org.eclipse.mosaic.fed.carla.carlaconnect;

import java.net.URL;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.client.XmlRpcClient;
import org.apache.xmlrpc.client.XmlRpcClientConfigImpl;
import org.apache.xmlrpc.client.XmlRpcClientException;
import org.eclipse.mosaic.interactions.detector.DetectedObjectInteraction;
import org.eclipse.mosaic.interactions.detector.DetectorRegistration;
import org.eclipse.mosaic.lib.objects.detector.DetectedObject;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.gson.Gson;

/**
 * This is a class uses xmlrpc to connect with CARLA CDASim adapter service
 */
public class CarlaXmlRpcClient{

    boolean isConnected;
    private static final String CREATE_SENSOR = "create_simulated_semantic_lidar_sensor";
    private static final String GET_SENSOR = "get_simulated_sensor";
    private static final String GET_DETECTED_OBJECTS = "get_detected_objects";

    private XmlRpcClient client;
    private final Logger log = LoggerFactory.getLogger(this.getClass());


    public CarlaXmlRpcClient(URL xmlRpcServerUrl) {
        initialize(xmlRpcServerUrl);
    }

    /**
     * This method is used to send a stop message to the python server side to shut down server from there
     */
    public void closeConnection()
    {
        log.info("carla connection server closing");
        isConnected = false;      
    }



    /**
     * need to getting a URL to connect to from ambassador
     * @param xmlRpcServerUrl
     */
    public void initialize(URL xmlRpcServerUrl)
    {
        XmlRpcClientConfigImpl config = new XmlRpcClientConfigImpl();   
        config.setServerURL(xmlRpcServerUrl);
        client = new XmlRpcClient();
        client.setConfig(config);
        isConnected = true;
    }


    public void createSensor(DetectorRegistration registration) throws XmlRpcException{
        List<Double> location = Arrays.asList(registration.getDetector().getLocation().getX(), registration.getDetector().getLocation().getY(), registration.getDetector().getLocation().getZ());
        List<Double> orientation = Arrays.asList(registration.getDetector().getOrientation().getPitch(), registration.getDetector().getOrientation().getRoll(), registration.getDetector().getOrientation().getYaw());

        if (isConnected) {
            Object[] params = new Object[]{registration.getInfrastructureId(), registration.getDetector().getSensorId(), location, orientation};
            Object result = client.execute(CREATE_SENSOR, params);
            log.info((String)result);
        }
        else {
            log.warn("XMLRpcClient is not connected to CARLA Adapter!");
        }
    }

    public DetectedObject[] getDetectedObjects(String infrastructureId ,String sensorId) throws XmlRpcException{
        if (isConnected) {
            Object[] params = new Object[]{infrastructureId, sensorId};
            Object result = client.execute(GET_DETECTED_OBJECTS, params);
            log.info("Detections from infrastructure {} sensor {} : {}", infrastructureId, sensorId, result);
            String jsonResult = (String)result;
            Gson gson = new Gson();
            DetectedObject[] parsedMessage = gson.fromJson(jsonResult,
                    DetectedObject[].class);
            return parsedMessage;
        }
        else {
            throw new XmlRpcException("XMLRpcClient is not connected to CARLA Adapter!");
            
        }
    }

}
