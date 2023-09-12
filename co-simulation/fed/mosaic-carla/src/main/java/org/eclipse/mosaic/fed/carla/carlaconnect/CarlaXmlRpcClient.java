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

import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.client.XmlRpcClient;
import org.apache.xmlrpc.client.XmlRpcClientConfigImpl;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * This is a class uses xmlrpc to connect with CARLA CDASim adapter service
 */
public class CarlaXmlRpcClient{

    boolean isConnected;
    private String registeredFunction = "test.echo";
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


    /**
    * This method uses xmlrpc to connect with CARLA CDASim adapter service
     * @throws XmlRpcException
    */
    public void requestCarlaList()
    {
        if(!isConnected)
        {
            
            try {
                throw new XmlRpcException("Server is not connected!");
            } 
            catch (XmlRpcException e) 
            {
                log.error("Server is not connected! {}", e.getMessage());
            }
        }
        try{          
            Object[] params = new Object[]{"Test " + java.time.LocalDateTime.now()};
            Object result = client.execute(registeredFunction, params);
            log.info((String)result);
        }
        catch (XmlRpcException x) 
        {
            log.error("Errors occurred with xmlrpc connection {}", x.getMessage());
            closeConnection();
        } 
        
    }
}
