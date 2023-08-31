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
import org.eclipse.mosaic.fed.carla.ambassador.CarlaAmbassador;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * This is a class uses xmlrpc to connect with CARLA CDASim adapter service
 */
public class CarlaMessageReceiver implements Runnable {

    boolean isRunning;
    CarlaAmbassador ambassador;
    private XmlRpcClient client;
    private final Logger log = LoggerFactory.getLogger(this.getClass());

    public CarlaMessageReceiver(CarlaAmbassador ambassador) {
        this.ambassador = ambassador;
    }



    /**
     * This method uses xmlrpc to connect with CARLA CDASim adapter service
     */
    @Override
    public void run(){
        XmlRpcClientConfigImpl config = new XmlRpcClientConfigImpl();
        
        try{
            //set up address for server
            config.setServerURL(new URL("http://127.0.0.1:8090/RPC2"));
            client = new XmlRpcClient();
            client.setConfig(config);
            isRunning = true;
            while(isRunning)
            {
                //used to pass arguments to python side, can be in any format
                Object[] params = new Object[]{new String("Test " + java.time.LocalDateTime.now())};

                //result is the return object from python side
                Object result = (Object) client.execute("test.echo", params);

                //pause the loop for 10 hz, change as needed
                Thread.sleep(100);
            }
        }
        catch (XmlRpcException XmlException) 
        {
            log.error("Errors occurred with xmlrpc connection ", XmlException.getMessage());
        }
        catch (InterruptedException e) 
        {
			log.error("Errors occurred with ", e.getMessage());
            Thread.currentThread().interrupt();   
		} 
        catch (MalformedURLException m) 
        {
            log.error("Errors occurred with ", m.getMessage());
        }

    }

    /**
     * This method is used to send a stop message to the python server side to shut down server from there
     */
    public synchronized void closeConnection()
    {
        log.info("carla connection server closing");
        isRunning = false;
        try {
            Object[] params = new Object[]{new String("Stop Server")};
            String result =  (String)client.execute("test.echo", params);
            log.info(result);
        } 
        catch (XmlRpcException e) 
        {
            log.error("Errors occurred with ", e.getMessage());
        }
    }
}
