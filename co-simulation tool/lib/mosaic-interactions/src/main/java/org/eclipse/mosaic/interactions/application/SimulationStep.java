/*
 * Copyright (c) 2020 Fraunhofer FOKUS and others. All rights reserved.
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contact: mosaic@fokus.fraunhofer.de
 */
//create carla simulation step cmd interaction
package org.eclipse.mosaic.interactions.application;

import org.eclipse.mosaic.rti.api.Interaction;

import javax.annotation.concurrent.Immutable;

/**
 * This extension of {@link Interaction} is used to send simulation step cmd to
 * SUMO ambassador. It will change the flag of simulation step in sumo and allow
 * sumo to advance sumo time.
 */
@Immutable
public final class SimulationStep extends Interaction {

    private static final long serialVersionUID = 1L;

    /**
     * String identifying the type of this interaction.
     */
    public final static String TYPE_ID = createTypeIdentifier(SimulationStep.class);

    /**
     * Constructor for {@link SimulationStep}.
     *
     * @param time Timestamp of this interaction, unit: [ns]
     */
    public SimulationStep(final long time) {
        super(time);
    }

}
