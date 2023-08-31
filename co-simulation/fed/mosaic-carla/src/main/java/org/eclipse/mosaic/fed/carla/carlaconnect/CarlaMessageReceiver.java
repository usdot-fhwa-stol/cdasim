/*Copyright (C) 2023 LEIDOS.

Licensed under the Apache License, Version 2.0 (the "License"); you may not
use this file except in compliance with the License. You may obtain a copy of
the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
License for the specific language governing permissions and limitations under
the License.*/
package org.eclipse.mosaic.fed.carla.carlaconnect;

import java.net.URL;
import java.net.MalformedURLException;
import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.client.XmlRpcClient;
import org.apache.xmlrpc.client.XmlRpcClientConfigImpl;
import java.util.*;

public class CarlaMessageReceiver implements Runnable {

    public CarlaMessageReceiver() {

    }

    @Override
    public void run(){
        XmlRpcClientConfigImpl config = new XmlRpcClientConfigImpl();
        
        try{
            //set up address for server
            config.setServerURL(new URL("http://127.0.0.1:8090/RPC2"));
            XmlRpcClient client = new XmlRpcClient();
            client.setConfig(config);
            
            while(true)
            {
                //used to pass arguments to python side, can be in any format
                Object[] params = new Object[]{new String("Test " + java.time.LocalDateTime.now())};

                //result is the return object from python side
                Object result = (Object) client.execute("test.echo", params);

                //pause the loop for 10 sec, change as needed
                Thread.sleep(10000);
            }
        }
        catch (XmlRpcException XmlException) 
        {
            XmlException.printStackTrace();     
        }
        catch(MalformedURLException URLException)
        {
            URLException.printStackTrace();
        } 
        catch (InterruptedException InterruptedException) 
        {
			InterruptedException.printStackTrace();
		}

    }
}
