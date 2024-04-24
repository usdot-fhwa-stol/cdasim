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

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.atomic.AtomicBoolean;

import com.google.gson.Gson;
import java.io.DataInputStream;
import java.net.ServerSocket;
import java.net.Socket;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Worker thread Runnable for operating a listen socket to receive outbound
 * CARMA Cloud Messages from CARMA Cloud instances
 * This {@link Runnable} instance will operate a UDP socket to subscribe to
 * packets from a CARMA Cloud adapter. 
 * Upon receiving a packet, it will be queued for the primary thread
 * to process the data once it ticks to a simulation processing step
 */
public class CarmaCloudRegistrationReceiver implements Runnable
{
	private static final int LISTEN_PORT = 1617; // which port for CARMA Cloud?
	private final AtomicBoolean running = new AtomicBoolean(false);
	private final Logger log = LoggerFactory.getLogger(this.getClass());
	private final Queue<CarmaCloudRegistrationMessage> rxQueue = new LinkedList<>();
	private ServerSocket srvr;


	/**
	 * Initialize the listen socket for messages from the CARMA Cloud instance adapter
	 * 
	 * @throws RuntimeException if socket instantiation fails
	 */
	public void init() throws RuntimeException
	{
		try
		{
			srvr = new ServerSocket(LISTEN_PORT);
		}
		catch (Exception oEx)
		{
			throw new RuntimeException("server socket instantiation failure", oEx);
		}
	}


	/**
	 * The main method of the worker thread. Listens for incoming messages and
	 * queues them for processing on the primary thread.
	 */
	@Override
	public void run()
	{
		running.set(true);
		try
		{
			while (running.get())
			{
				Socket oSock = srvr.accept();
        		DataInputStream oIn = new DataInputStream(oSock.getInputStream());

				// parse message
				CarmaCloudRegistrationMessage parsedMessage = 
					new Gson().fromJson(oIn.readUTF(), CarmaCloudRegistrationMessage.class);

				// Enqueue message for processing on main thread
				synchronized (rxQueue)
				{
					rxQueue.add(parsedMessage);
				}
				log.info("New CARMA Cloud instance '{}' received with CARMA Cloud Registration Receiver.", parsedMessage.getId());
			}
		}
		catch (Exception oEx)
		{
			log.error("Error occurred", oEx);
		}
	}


	/**
	 * Stop the runnable instance and close the listen socket.
	 */
	public void stop()
	{
		running.set(false);
		if (srvr != null)
		{
			try
			{
				srvr.close();
			}
			catch (Exception oEx)
			{
				log.error("Error occurred", oEx);
			}
		}
	}


	/**
	 * Query the current buffer of outbound messages. Clears the currently stored
	 * buffer once called. Thread-safe.
	 * 
	 * @return The list of received outbound message from all Infrastructure Device
	 *         instances since last call of this method
	 */
	public List<CarmaCloudRegistrationMessage> getReceivedMessages()
	{
		List<CarmaCloudRegistrationMessage> output = new ArrayList<>();
		synchronized (rxQueue)
		{
			output.addAll(rxQueue);
			rxQueue.clear();
		}
		return output;
	}
}
