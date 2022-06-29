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
import org.eclipse.mosaic.interactions.communication.V2xMessageTransmission;
import org.eclipse.mosaic.interactions.application.ExternalMessage;
import org.eclipse.mosaic.lib.enums.AdHocChannel;
import org.eclipse.mosaic.fed.application.app.AbstractApplication;
import org.eclipse.mosaic.fed.application.app.api.VehicleApplication;
import org.eclipse.mosaic.fed.application.app.api.CommunicationApplication;
import org.eclipse.mosaic.fed.application.app.api.os.VehicleOperatingSystem;
import org.eclipse.mosaic.lib.geo.GeoCircle;
import org.eclipse.mosaic.lib.geo.GeoPoint;
import org.eclipse.mosaic.lib.objects.v2x.ExternalV2xContent;
import org.eclipse.mosaic.lib.objects.v2x.ExternalV2xMessage;
import org.eclipse.mosaic.lib.objects.v2x.MessageRouting;
import org.eclipse.mosaic.lib.objects.v2x.V2xMessage;
import org.eclipse.mosaic.lib.util.scheduling.Event;
import org.eclipse.mosaic.lib.objects.vehicle.VehicleData;
import javax.annotation.Nonnull;
import javax.annotation.Nullable;

/**
 * Class implementing the application interface and fulfilling V2x communication
 * as well as sending external messages as V2X messages.
 */
public final class VehicleCommunicationApp extends AbstractApplication<VehicleOperatingSystem>
        implements VehicleApplication, CommunicationApplication {

    /**
     * This method is called by MOSAIC RTI when the vehicle that has been equipped
     * with this application enters the simulation. It is the first method called of
     * this class during a simulation.
     */
    @Override
    public void onStartup() {
        getLog().infoSimTime(this, "Initialize application.");
        getOs().getAdHocModule()
                .enable(new AdHocModuleConfiguration().addRadio().channel(AdHocChannel.CCH).power(50).create());

        getLog().infoSimTime(this, "Activated AdHocModule.");
    }

    /**
     * This method is called by MOSAIC RTI when the vehicle that has been equipped
     * with this application leaves the simulation. It is the last method called of
     * this class during a simulation.
     */
    @Override
    public void onShutdown() {
        getLog().infoSimTime(this, "Shutdown application.");
    }

    /**
     * This method is called by MOSAIC RTI when a previously triggered event is
     * handed over to the application by MOSAIC RTI for processing.
     * 
     * @param event The event to be processed.
     */
    @Override
    public void processEvent(Event event) throws Exception {

    }

    /**
     * Receive a V2X Message.
     *
     * @param receivedV2xMessage the received message container.
     */
    @Override
    public void onMessageReceived(ReceivedV2xMessage receivedV2xMessage) {
        final V2xMessage v2xMessage = receivedV2xMessage.getMessage();

        // Only External V2X Messages are handled
        if (!(v2xMessage instanceof ExternalV2xMessage)) {
            getLog().infoSimTime(this, "Ignoring message of type: {}", v2xMessage.getSimpleClassName());
            return;
        }

        final ExternalV2xMessage externalV2xMessage = (ExternalV2xMessage) v2xMessage;

        if (v2xMessage.getRouting().getSource().getSourceName().startsWith("carma")) {
            getLog().infoSimTime(this, "Vehicle rceived message from carma vehicle: {}.",
                    v2xMessage.getRouting().getSource().getSourceName());
        }
        if (v2xMessage.getRouting().getSource().getSourceName().startsWith("rsu")) {
            getLog().infoSimTime(this, "Vehicle received message from infrastruture: {}.",
                    v2xMessage.getRouting().getSource().getSourceName());
        }
        if (v2xMessage.getRouting().getSource().getSourceName().startsWith("carla")) {
            getLog().infoSimTime(this, "Vehicle received message from carla vehicle: {}.",
                    v2xMessage.getRouting().getSource().getSourceName());
        }

        getLog().infoSimTime(this, "External mesage is received by: " + getOs().getId());
        getLog().infoSimTime(this, "External message content is " + externalV2xMessage.getMessage());

    }

    /**
     * Is called when ever the vehicle has moved. That is, if the
     * {@link VehicleData} of the unit has been updated.
     *
     * @param previousVehicleData the previous state of the vehicle
     * @param updatedVehicleData  the updated state of the vehicle
     */
    @Override
    public void onVehicleUpdated(@Nullable VehicleData previousVehicleData, @Nonnull VehicleData updatedVehicleData) {
        if (!isValidStateAndLog()) {
            return;
        }
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
     * Receive an external Message.
     *
     * @param externalMessage the received message container.
     */
    @Override
    public void onExternalMessageReceived(ExternalMessage externalMessage) {
        String message = externalMessage.getMessage();
        getLog().infoSimTime(this, "Received message: \"{}\".", message);

        // failsafe
        if (getOs().getVehicleData() == null) {
            getLog().infoSimTime(this, "No vehicleInfo given, skipping.");
            return;
        }

        // longLat of the vehicle that sends V2X message.
        GeoPoint vehicleLongLat = getOs().getPosition();

        // Region with a radius around the coordinates of the car.
        GeoCircle dest = new GeoCircle(vehicleLongLat, 3000);

        // A MessageRouting object contains a source and a target address for a message
        // to be routed.
        MessageRouting messageRouting = getOs().getAdHocModule().createMessageRouting().geoBroadCast(dest);

        // generate a v2x message
        ExternalV2xMessage externalV2xMessage = new ExternalV2xMessage(messageRouting,
                new ExternalV2xContent(getOs().getSimulationTime(), vehicleLongLat, message));

        getLog().infoSimTime(this, "Sending external V2X message to network simulator.");

        getOs().getAdHocModule().sendV2xMessage(externalV2xMessage);

    }

}
