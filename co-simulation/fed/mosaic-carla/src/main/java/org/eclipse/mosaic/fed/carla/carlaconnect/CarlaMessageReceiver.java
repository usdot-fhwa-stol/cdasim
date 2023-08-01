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

import libpython_clj2.java_api;

import java.net.*;
import java.io.*;
import java.util.Arrays;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.eclipse.mosaic.fed.carla.ambassador.CarlaAmbassador;

public class CarlaMessageReceiver implements Runnable {

    public CarlaMessageReceiver() {

    }

    @Override
    public void run() {
       
        java_api.initialize(null);

        try(AutoCloseable locker = java_api.GILLocker()){

            Object dummy = java_api.importModule("DummySender");
            Object test = java_api.getAttr(dummy, "get_objects_in_frame");
            Object testprint = java_api.call(test);
            System.out.println(testprint);

        }
        catch (Exception e) {
            // TODO: handle exception       
        }

    }
}
