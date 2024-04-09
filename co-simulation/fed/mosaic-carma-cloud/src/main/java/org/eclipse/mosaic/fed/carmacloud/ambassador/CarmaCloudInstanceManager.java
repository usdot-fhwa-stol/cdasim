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

package org.eclipse.mosaic.fed.carmacloud.ambassador;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import gov.dot.fhwa.saxton.TimeSyncMessage;


/**
 * Session management class for Infrastructure instances communicating with
 * MOSAIC.
 * 
 * This class is responsible for managing instances of CARMA Cloud registered
 * with the MOSAIC system. It provides methods for registering new instances,
 * checking if instances are registered, and storing and retrieving instances
 * from a map.
 */
public class CarmaCloudInstanceManager
{
	private final Map<String, CarmaCloudInstance> managedInstances = new HashMap<>();
	private final Logger log = LoggerFactory.getLogger(this.getClass());


	/**
	 * Register a new CARMA Cloud instance with the MOSAIC system.
	 * 
	 * This method takes a CarmaCloudRegistrationMessage, converts it to a
	 * CarmaCloudInstance, and adds it to the managedInstances map 
	 * if it is not already present.
	 * 
	 * @param registration The CarmaCloudRegistrationMessage to be registered.
	 * 
	 */
	public void onNewRegistration(CarmaCloudRegistrationMessage registration)
	{
		if (!managedInstances.containsKey(registration.getId()))
		{
			CarmaCloudInstance tmp = new CarmaCloudInstance(registration.getId(), registration.getUrl());
			managedInstances.put(registration.getId(), tmp);
		}
		else
		{
			log.warn("Registration message received for already registered CARMA Cloud with ID: {}", registration.getId());
		}
			
	}


	/**
	 * This function is used to send out encoded time step update to all registered
	 * instances the manager has on the managed instances map
	 * 
	 * @param message This time message is used to store current seq and time step
	 *                from the ambassador side
	 * @throws IOException
	 */
	public void onTimeStepUpdate(TimeSyncMessage message)
		throws IOException
	{
		if (managedInstances.isEmpty()){
			log.debug("There are no registered instances");
		}
		else
		{
			for (CarmaCloudInstance currentInstance : managedInstances.values()){
				currentInstance.sendTimeSyncMsg(message);
				log.debug("Sent time message to CARMA-Cloud" + message.toString());
			}
		}
	}

	/**
	 * Returns Map of managed CARMA Cloud instances with CARMA Cloud ID as the 
	 * String Key.
	 * 
	 * @return map of managed CARMA Cloud instances.
	 */
	public Map<String, CarmaCloudInstance> getManagedInstances()
	{
		return managedInstances;
	}
}
