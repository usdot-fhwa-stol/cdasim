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

package org.eclipse.mosaic.fed.sumo.traci.commands;

import org.eclipse.mosaic.fed.sumo.traci.AbstractTraciCommand;
import org.eclipse.mosaic.fed.sumo.traci.TraciCommandException;
import org.eclipse.mosaic.fed.sumo.traci.TraciConnection;
import org.eclipse.mosaic.fed.sumo.traci.TraciVersion;
import org.eclipse.mosaic.fed.sumo.traci.complex.Status;
import org.eclipse.mosaic.fed.sumo.traci.constants.CommandSimulationControl;
import org.eclipse.mosaic.rti.api.InternalFederateException;


public class TraciSetOrder extends AbstractTraciCommand<Void> {

    public TraciSetOrder() {
        super(TraciVersion.LOWEST);

        write()
                .command(CommandSimulationControl.COMMAND_SET_ORDER)
                .writeIntParam();
    }

    public void execute(TraciConnection traciCon, int orderNum) throws TraciCommandException, InternalFederateException {
        super.execute(traciCon, orderNum);
    }

    @Override
    protected Void constructResult(Status status, Object... objects) {
        return null;
    }
}
