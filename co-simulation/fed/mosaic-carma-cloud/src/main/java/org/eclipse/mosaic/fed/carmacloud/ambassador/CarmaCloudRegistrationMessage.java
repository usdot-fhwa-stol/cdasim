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


/**
 * A message to be sent by CARMA Cloud instance when it registers with the
 * carmacloud-mosaic ambassador
 */
public class CarmaCloudRegistrationMessage
{
	// unique simulation identifier for the CARMA Cloud instance
	private String m_sCarmaCloudId;
	// URL endpoint where to send simulation time sync messages
	private String m_sCarmaCloudUrl;


	/**
	 * Constructor for a CarmaCloudRegistrationMessage
	 * 
	 * @param sId          the ID of the CARMA Cloud instance.
	 * @param sUrl         the receive time synchronization message port of the infrastructure.
	 */ 
	public CarmaCloudRegistrationMessage(String sId, String sUrl)
	{
		m_sCarmaCloudId = sId;
		m_sCarmaCloudUrl = sUrl;
	}


	/**
	 * Returns the URL endpoint of the CARMA Cloud instance
	 * 
	 * @return String URL endpoint of the CARMA Cloud instance
	 */
	public String getUrl()
	{
		return m_sCarmaCloudUrl;
	}


	/**
	 * Set the URL endpoint of the CARMA Cloud instance
	 * 
	 * @param sUrl          the receive time synchronization message endpoint of the infrastructure.
	 */
	public void setUrl(String sUrl)
	{
		m_sCarmaCloudUrl = sUrl;
	}


	/**
	 * Returns the ID of the CARMA Cloud instance
	 * 
	 * @return String the ID of the CARMA Cloud instance
	 */
	public String getId()
	{
			return m_sCarmaCloudId;
	}


	/**
	 * Set the ID of the CARMA Cloud instance
	 * 
	 * @param sId          the ID of the CARMA Cloud instance.
	 */
	public void setId(String sId)
	{
		m_sCarmaCloudId = sId;
	}


	@Override
	public String toString()
	{
		return String.format("CarmaCloudRegistrationMessage id %s url %s", m_sCarmaCloudId, m_sCarmaCloudUrl);
	}
}