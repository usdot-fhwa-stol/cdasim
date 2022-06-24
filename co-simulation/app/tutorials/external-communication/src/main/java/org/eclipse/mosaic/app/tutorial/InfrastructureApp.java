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

package org.eclipse.mosaic.app.tutorial;

import org.eclipse.mosaic.fed.application.ambassador.simulation.communication.AdHocModuleConfiguration;
import org.eclipse.mosaic.fed.application.ambassador.simulation.communication.CamBuilder;
import org.eclipse.mosaic.fed.application.ambassador.simulation.communication.ReceivedAcknowledgement;
import org.eclipse.mosaic.fed.application.ambassador.simulation.communication.ReceivedV2xMessage;
import org.eclipse.mosaic.fed.application.app.AbstractApplication;
import org.eclipse.mosaic.fed.application.app.api.CommunicationApplication;
import org.eclipse.mosaic.fed.application.app.api.os.RoadSideUnitOperatingSystem;
import org.eclipse.mosaic.interactions.application.ExternalMessage;
import org.eclipse.mosaic.interactions.communication.V2xMessageTransmission;
import org.eclipse.mosaic.lib.enums.AdHocChannel;
import org.eclipse.mosaic.lib.objects.v2x.ExternalV2xContent;
import org.eclipse.mosaic.lib.objects.v2x.ExternalV2xMessage;
import org.eclipse.mosaic.lib.objects.v2x.MessageRouting;
import org.eclipse.mosaic.lib.objects.v2x.V2xMessage;
import org.eclipse.mosaic.lib.util.scheduling.Event;

/**
 * This class is used to send received external infrstructure messages to
 * network simulator.
 */
public class InfrastructureApp extends AbstractApplication<RoadSideUnitOperatingSystem>
        implements CommunicationApplication {

    /**
     * This method is called when the RSU that has been equipped with this
     * application enters the simulation.
     */
    @Override
    public void onStartup() {
        getLog().infoSimTime(this, "Initialize application.");
        getOs().getAdHocModule()
                .enable(new AdHocModuleConfiguration().addRadio().channel(AdHocChannel.CCH).power(50).create());

        getLog().infoSimTime(this, "Activated AdHocModule.");
    }

    /**
     * This method is used to process event.
     *
     * @param event The event to be processed.
     */
    @Override
    public void processEvent(Event event) throws Exception {
    }

    /**
     * This method is called when the simulation stops.
     */
    @Override
    public void onShutdown() {
        getLog().infoSimTime(this, "Shutdown application.");
    }

    /**
     * This method is called when the external message is sent by this RSU.
     * 
     * @param externalMessage The external message received.
     */
    @Override
    public void onExternalMessageReceived(ExternalMessage externalMessage) {
        String message = externalMessage.getMessage();
        getLog().infoSimTime(this, "{} received message: \"{}\".", getOs().getId(), message);

        final MessageRouting messageRouting = getOs().getAdHocModule().createMessageRouting()
                .viaChannel(AdHocChannel.CCH).topoBroadCast();
        ExternalV2xMessage externalV2xMessage = new ExternalV2xMessage(messageRouting,
                new ExternalV2xContent(getOs().getSimulationTime(), getOs().getPosition(), message));

        getLog().infoSimTime(this, "Sending external V2X message to network simulator.");
        getOs().getAdHocModule().sendV2xMessage(externalV2xMessage);
    }

    /**
     * This method is called when a CAM is build by different Simulation Units that
     * support CAM connectivity.
     *
     * @param camBuilder the builder for the CAM.
     */
    @Override
    public void onCamBuilding(CamBuilder camBuilder) {
    }

    /**
     * This method is called when a V2X message is transmitted.
     *
     * @param v2xMessageTransmission the container for the V2XMessage to be
     *                               transmitted.
     */
    @Override
    public void onMessageTransmitted(V2xMessageTransmission v2xMessageTransmission) {
    }

    /**
     * Receive an acknowledgement from a previously sent V2X Message.
     *
     * @param acknowledgedMessage the acknowledgement object which contains the sent
     *                            V2X Message and the acknowledgement status.
     */
    @Override
    public void onAcknowledgementReceived(ReceivedAcknowledgement acknowledgedMessage) {
    }

    /**
     * Receive a V2X Message.
     *
     * @param receivedV2xMessage the received message container.
     */
    @Override
    public void onMessageReceived(ReceivedV2xMessage receivedV2xMessage) {
        final V2xMessage v2xMessage = receivedV2xMessage.getMessage();

        // Only External V2X Messages are handled.
        if (!(v2xMessage instanceof ExternalV2xMessage)) {
            getLog().infoSimTime(this, "Ignoring message of type: {}", v2xMessage.getSimpleClassName());
            return;
        }

        final ExternalV2xMessage externalV2xMessage = (ExternalV2xMessage) v2xMessage;

        if (v2xMessage.getRouting().getSource().getSourceName().startsWith("carma")) {
            getLog().infoSimTime(this, "RSU rceived message from carma vehicle: {}.",
                    v2xMessage.getRouting().getSource().getSourceName());
        }
        if (v2xMessage.getRouting().getSource().getSourceName().startsWith("rsu")) {
            getLog().infoSimTime(this, "RSU received message from infrastruture: {}.",
                    v2xMessage.getRouting().getSource().getSourceName());
        }
        if (v2xMessage.getRouting().getSource().getSourceName().startsWith("carla")) {
            getLog().infoSimTime(this, "RSU received message from carla vehicle: {}.",
                    v2xMessage.getRouting().getSource().getSourceName());
        }

        getLog().infoSimTime(this, "External mesage is received by: " + getOs().getId());
        getLog().infoSimTime(this, "External message content is " + externalV2xMessage.getMessage());
    }

}
