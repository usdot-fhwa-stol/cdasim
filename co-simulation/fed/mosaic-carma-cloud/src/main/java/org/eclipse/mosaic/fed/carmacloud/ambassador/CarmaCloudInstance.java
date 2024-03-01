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

import java.io.DataOutputStream;
import java.io.IOException;
import java.net.URL;
import java.net.HttpURLConnection;
import com.google.gson.Gson;
import gov.dot.fhwa.saxton.TimeSyncMessage;


/**
 * CarmaCloudInstance class represents a physical instance of an
 * CARMA Cloud in the simulated environment.
 * It contains information about the instance such as its id and target address
 */
public class CarmaCloudInstance
{
	// unique simulation identifier for the CARMA Cloud instance
	private final String m_sCarmaCloudId;
	// URL endpoint where to send simulation time sync messages
	private final String m_sCarmaCloudUrl;


	/**
	 * Constructor for CarmaCloudInstance
	 * 
	 * @param sId          the ID of the CARMA Cloud instance.
	 * @param sUrl              the receive time synchronization message port of the infrastructure.
	 */ 
	public CarmaCloudInstance(String sId, String sUrl)
	{
		m_sCarmaCloudId = sId;
		m_sCarmaCloudUrl = sUrl;
	}


	/**
	 * Returns the URL endpoint of the CARMA Cloud instance
	 * 
	 * @return String URL endpoint of the CARMA Cloud instance
	 */
	public String getCarmaCloudUrl()
	{
		return m_sCarmaCloudUrl;
	}


	/**
	 * Returns the ID of the CARMA Cloud instance
	 * 
	 * @return String the ID of the CARMA Cloud instance
	 */
	public String getCarmaCloudId()
	{
			return m_sCarmaCloudId;
	}


	/**
	 * Sends time sync data to the CARMA Cloud Instance
	 * 
	 * @param oMsg the JSON time sync state to transmit
	 * @throws IOException if there is an issue with the underlying URL connection
	 */
	public void sendTimeSyncMsg(TimeSyncMessage oMsg)
		throws IOException
	{
		HttpURLConnection oHttp = (HttpURLConnection)new URL(m_sCarmaCloudUrl).openConnection();
		oHttp.setRequestMethod("POST");
		oHttp.setDoOutput(true);
		try (DataOutputStream oOut = new DataOutputStream(oHttp.getOutputStream()))
		{
			oOut.write(new Gson().toJson(oMsg).getBytes());
		}
		if (oHttp.getResponseCode() != 200)
			throw new IOException();
	}
}
