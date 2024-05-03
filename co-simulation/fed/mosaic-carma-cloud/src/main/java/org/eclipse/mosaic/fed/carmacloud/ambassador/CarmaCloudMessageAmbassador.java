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
import java.util.List;

import org.eclipse.mosaic.fed.carmacloud.configuration.CarmaCloudConfiguration;
import org.eclipse.mosaic.interactions.communication.V2xMessageReception;
import org.eclipse.mosaic.lib.util.objects.ObjectInstantiation;
import org.eclipse.mosaic.rti.api.AbstractFederateAmbassador;
import org.eclipse.mosaic.rti.api.IllegalValueException;
import org.eclipse.mosaic.rti.api.Interaction;
import org.eclipse.mosaic.rti.api.InternalFederateException;
import org.eclipse.mosaic.rti.api.parameters.AmbassadorParameter;
import org.eclipse.mosaic.rti.TIME;

import gov.dot.fhwa.saxton.TimeSyncMessage;

/**
 * Implementation of a {@link AbstractFederateAmbassador} for CarmaCloud
 * message ambassador.
 */
public class CarmaCloudMessageAmbassador extends AbstractFederateAmbassador
{
	/**
	 * Simulation time.
	 */
	long currentSimulationTime;

	/**
	 * CarmaCloudMessageAmbassador configuration file.
	 */
	CarmaCloudConfiguration carmaCloudConfiguration;

	private CarmaCloudRegistrationReceiver carmaCloudRegistrationReceiver;
	private final CarmaCloudInstanceManager carmaCloudInstanceManager = new CarmaCloudInstanceManager();
	private int timeSyncSeq;


	/**
	 * Create a new {@link CarmaCloudMessageAmbassador} object.
	 *
	 * @param ambassadorParameter includes parameters for the
	 *                            CarmaCloudMessageAmbassador.
	 */
	public CarmaCloudMessageAmbassador(AmbassadorParameter ambassadorParameter) throws RuntimeException
	{
		super(ambassadorParameter);
		try // load configuration file
		{
			carmaCloudConfiguration = new ObjectInstantiation<>(CarmaCloudConfiguration.class, log)
				.readFile(ambassadorParameter.configuration);
		}
		catch (InstantiationException e)
		{
			log.error("Configuration object could not be instantiated: ", e);
		}

		log.info("The update interval of CARMA Cloud message ambassador is {}.", carmaCloudConfiguration.updateInterval);
		
		if (carmaCloudConfiguration.updateInterval <= 0L)
		{
			throw new RuntimeException("Invalid update interval for CARMA Cloud message ambassador, should be > 0.");
		}
		log.info("CARMA Cloud message ambassador is generated.");
	}


	/**
	 * This method is called to tell the federate the start time and the end time.
	 *
	 * @param startTime Start time of the simulation run in nanoseconds.
	 * @param endTime   End time of the simulation run in nanoseconds.
	 * @throws InternalFederateException Exception is thrown if an error is occurred
	 *                                   while execute of a federate.
	 */
	@Override
	public void initialize(long startTime, long endTime)
		throws InternalFederateException
	{
		super.initialize(startTime, endTime);
		currentSimulationTime = startTime;

		carmaCloudRegistrationReceiver = new CarmaCloudRegistrationReceiver();
		carmaCloudRegistrationReceiver.init();
		new Thread(carmaCloudRegistrationReceiver).start();

		try
		{
			rti.requestAdvanceTime(currentSimulationTime, 0, (byte) 1);
		}
		catch (IllegalValueException e)
		{
			log.error("Error during advanceTime request", e);
			throw new InternalFederateException(e);
		}
	}


	/**
	 * This method is called by the AbstractFederateAmbassador when the RTI grants a
	 * time advance to the federate.Any unprocessed interactions are forwarded to
 the federate using the processInteraction method before this call is made.
	 *
	 * @param time The timestamp (in nanoseconds) indicating the time to which the federate can
	 *             advance its local time.
	 * @throws InternalFederateException
	 */
	@Override
	public synchronized void processTimeAdvanceGrant(long time)
		throws InternalFederateException
	{
		// Process the time advance only if the time is equal or greater than the next
		// simulation time step
		log.debug("Process time advance grant from {} to {}.", currentSimulationTime, time);
		if (time < currentSimulationTime)
		{
			return;
		}
		
		currentSimulationTime = time;
		try
		{
			// Handle any new carmaCloud registration requests
			List<CarmaCloudRegistrationMessage> newRegistrations = carmaCloudRegistrationReceiver.getReceivedMessages();
			for (CarmaCloudRegistrationMessage reg : newRegistrations)
			{
				log.info("Processing new registration request for  {}.", reg.getId());
				// Store new instance registration to carmaCloud instance manager
				carmaCloudInstanceManager.onNewRegistration(reg);
			}

			timeSyncSeq += 1;
			// nanoseconds to milliseconds for CarmaCloudTimeMessage
			TimeSyncMessage timeSyncMessage = new TimeSyncMessage(currentSimulationTime / 1000000L, timeSyncSeq);
			carmaCloudInstanceManager.onTimeStepUpdate(timeSyncMessage);

			// Advance the simulation time
			currentSimulationTime += carmaCloudConfiguration.updateInterval* TIME.MILLI_SECOND;

			// Request the next time advance from the RTI
			log.debug("Requesting timestep updated to  {}.", currentSimulationTime);
			rti.requestAdvanceTime(currentSimulationTime, 0, (byte) 2);
		}
		catch (IllegalValueException e)
		{
			throw new InternalFederateException(e);
		}
		catch (IOException e)
		{
			log.error("Error during updating timestep :", e);
		}
	}


	/**
	 * Return whether this federate is time constrained. Is set if the federate is
	 * sensitive towards the correct ordering of events. The federate ambassador
	 * will ensure that the message processing happens in time stamp order. If set
	 * to false, interactions will be processed in receive order.
	 *
	 * @return {@code true} if this federate is time constrained, else
	 *         {@code false}.
	 */
	@Override
	public boolean isTimeConstrained()
	{
		return true;
	}


	/**
	 * Return whether this federate is time regulating. Is set if the federate
	 * influences other federates and can prevent them from advancing their local
	 * time.
	 *
	 * @return {@code true} if this federate is time regulating, {@code false} else.
	 */
	@Override
	public boolean isTimeRegulating()
	{
		return false;
	}


	/**
	 * Test helper function to cleanup sockets and threads.
	 */
	protected void close()
	{
		carmaCloudRegistrationReceiver.stop();
	}
}
