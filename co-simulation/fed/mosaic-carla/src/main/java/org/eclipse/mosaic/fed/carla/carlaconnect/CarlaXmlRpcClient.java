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
import java.net.MalformedURLException;
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
    private XmlRpcClientConfigImpl config;
    private final Logger log = LoggerFactory.getLogger(this.getClass());

    public CarlaXmlRpcClient() {
        
    }

    /**
     * This method is used to send a stop message to the python server side to shut down server from there
     */
    public void closeConnection()
    {
        log.info("carla connection server closing");
        isConnected = false;
        try {
            Object[] params = new Object[]{"Stop Server"};
            String result =  (String)client.execute(registeredFunction, params);
            log.info(result);
        } 
        catch (XmlRpcException e) 
        {
            log.error("Errors occurred with {0}", e.getMessage());
        }
    }

    public void initialize()
    {
        try{           
            config = new XmlRpcClientConfigImpl();
            config.setServerURL(new URL("http://127.0.0.1:8090/RPC2"));
            client = new XmlRpcClient();
            client.setConfig(config);
            Object[] connectionTest = new Object[]{"Connection test"};
            Object result = (String)client.execute(registeredFunction, connectionTest);
            isConnected = true;

            if(!result.equals("Connected!"))
            {
               isConnected = false;
               log.error("Server is not connected!");
            }
        }
        catch (MalformedURLException m) 
        {
            log.error("Errors occurred with {0}", m.getMessage());
            isConnected = false;
        }
        catch (XmlRpcException x) 
        {
            log.error("Errors occurred with xmlrpc connection {0}", x.getMessage());
            isConnected = false;
        } 

    }


    /**
    * This method uses xmlrpc to connect with CARLA CDASim adapter service
    */
    public void requestCarlaList()
    {
        if(!isConnected)
        {
            log.error("Server is not connected!");
            return;
        }
        try{          
            Object[] params = new Object[]{"Test " + java.time.LocalDateTime.now()};
            Object result = client.execute(registeredFunction, params);
            log.info((String)result);
        }
        catch (XmlRpcException x) 
        {
            log.error("Errors occurred with xmlrpc connection {0}", x.getMessage());
            closeConnection();
        } 
        
    }
}
