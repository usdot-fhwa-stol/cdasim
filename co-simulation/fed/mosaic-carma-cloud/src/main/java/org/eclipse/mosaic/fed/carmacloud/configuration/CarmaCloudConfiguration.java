/*
 * Copyright (c) 2021 Old Dominion University. All rights reserved.
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 */

package org.eclipse.mosaic.fed.carmacloud.configuration;

import java.io.Serializable;
import com.google.gson.annotations.JsonAdapter;
import org.eclipse.mosaic.lib.util.gson.TimeFieldAdapter;


/**
 * The CARMA Cloud Message Ambassador configuration class.
 */
public class CarmaCloudConfiguration implements Serializable
{
	private static final long serialVersionUID = 1705520136000000000L;


	@JsonAdapter(TimeFieldAdapter.LegacyMilliSeconds.class)
	public Long updateInterval = 100L;
}
