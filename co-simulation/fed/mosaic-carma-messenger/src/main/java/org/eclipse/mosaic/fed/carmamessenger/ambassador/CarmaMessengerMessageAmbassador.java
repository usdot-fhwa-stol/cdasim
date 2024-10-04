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
package org.eclipse.mosaic.fed.carmamessenger.ambassador;

import java.io.IOException;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.eclipse.mosaic.fed.application.ambassador.SimulationKernel;
import org.eclipse.mosaic.fed.carma.ambassador.CarmaMessageAmbassador;
import org.eclipse.mosaic.fed.carmamessenger.configuration.CarmaMessengerConfiguration;
import org.eclipse.mosaic.interactions.communication.AdHocCommunicationConfiguration;
import org.eclipse.mosaic.interactions.communication.V2xMessageTransmission;
import org.eclipse.mosaic.interactions.mapping.advanced.ExternalVehicleRegistration;
import org.eclipse.mosaic.interactions.traffic.VehicleUpdates;
import org.eclipse.mosaic.lib.enums.AdHocChannel;
import org.eclipse.mosaic.lib.enums.DriveDirection;
import org.eclipse.mosaic.lib.geo.CartesianPoint;
import org.eclipse.mosaic.lib.geo.GeoPoint;
import org.eclipse.mosaic.lib.misc.Tuple;
import org.eclipse.mosaic.lib.objects.addressing.IpResolver;
import org.eclipse.mosaic.lib.objects.communication.AdHocConfiguration;
import org.eclipse.mosaic.lib.objects.communication.InterfaceConfiguration;
import org.eclipse.mosaic.lib.objects.road.IRoadPosition;
import org.eclipse.mosaic.lib.objects.road.SimpleRoadPosition;
import org.eclipse.mosaic.lib.objects.vehicle.Consumptions;
import org.eclipse.mosaic.lib.objects.vehicle.Emissions;
import org.eclipse.mosaic.lib.objects.vehicle.VehicleBatteryState;
import org.eclipse.mosaic.lib.objects.vehicle.VehicleConsumptions;
import org.eclipse.mosaic.lib.objects.vehicle.VehicleData;
import org.eclipse.mosaic.lib.objects.vehicle.VehicleEmissions;
import org.eclipse.mosaic.lib.objects.vehicle.VehicleSensors;
import org.eclipse.mosaic.lib.objects.vehicle.VehicleSignals;
import org.eclipse.mosaic.lib.objects.vehicle.VehicleType;
import org.eclipse.mosaic.lib.objects.vehicle.sensor.DistanceSensor;
import org.eclipse.mosaic.lib.objects.vehicle.sensor.RadarSensor;
import org.eclipse.mosaic.lib.util.objects.ObjectInstantiation;
import org.eclipse.mosaic.rti.TIME;
import org.eclipse.mosaic.rti.api.IllegalValueException;
import org.eclipse.mosaic.rti.api.InternalFederateException;
import org.eclipse.mosaic.rti.api.parameters.AmbassadorParameter;

import gov.dot.fhwa.saxton.CarmaV2xMessage;
import gov.dot.fhwa.saxton.CarmaV2xMessageReceiver;
import gov.dot.fhwa.saxton.TimeSyncMessage;

public class CarmaMessengerMessageAmbassador extends CarmaMessageAmbassador{
    /**
     * Simulation time.
     */
    long currentSimulationTime;

    /**
     * CarmaMessageAmbassador configuration file.
     */
    CarmaMessengerConfiguration carmaMessengerConfiguration;

    private CarmaMessengerRegistrationReceiver carmaMessengerRegistrationReceiver;
    private Thread registrationRxBackgroundThread;
    private CarmaV2xMessageReceiver v2xMessageReceiver;
    private Thread v2xRxBackgroundThread;
    private CarmaMessengerInstanceManager carmaMessengerInstanceManager = new CarmaMessengerInstanceManager();
    private int timeSyncSeq = 0;


    /**
     * Create a new {@link CarmaMessengerMessageAmbassador} object.
     *
     * @param ambassadorParameter includes parameters for the
     *                            CarmaMessageAmbassador.
     */
    public CarmaMessengerMessageAmbassador(AmbassadorParameter ambassadorParameter) {
        super(ambassadorParameter);

        try {
            // Read the CARMA message ambassador configuration file
            carmaMessengerConfiguration = new ObjectInstantiation<>(CarmaMessengerConfiguration.class, log)
                    .readFile(ambassadorParameter.configuration);
        } catch (InstantiationException e) {
            log.error("Configuration object could not be instantiated: ", e);
        }

        log.info("The update interval of CARMA message ambassador is " + carmaMessengerConfiguration.updateInterval + " .");

        // Check the CARMA update interval
        if (carmaMessengerConfiguration.updateInterval <= 0) {
            throw new RuntimeException("Invalid update interval for CARMA message ambassador, should be >0.");
        }
        log.info("CARMA message ambassador is generated.");
    }

    /**
     * This method is called to tell the federate the start time and the end time.
     * 
     * @param startTime Start time of the simulation run in nano seconds.
     * @param endTime   End time of the simulation run in nano seconds.
     * @throws InternalFederateException Exception is thrown if an error is occurred
     *                                   while execute of a federate.
     */
    @Override
    public void initialize(long startTime, long endTime) throws InternalFederateException {
        currentSimulationTime = startTime;
        try {
            rti.requestAdvanceTime(currentSimulationTime, 0, (byte) 1);
        } catch (IllegalValueException e) {
            log.error("Error during advanceTime request", e);
            throw new InternalFederateException(e);
        }

        // Initialize listener socket and thread for CARMA Registration messages
        carmaMessengerRegistrationReceiver = new CarmaMessengerRegistrationReceiver();
        carmaMessengerRegistrationReceiver.init();
        registrationRxBackgroundThread = new Thread(carmaMessengerRegistrationReceiver);
        registrationRxBackgroundThread.start();

        // Initialize listener socket and thread for CARMA NS-3 Adapter messages
        v2xMessageReceiver = new CarmaV2xMessageReceiver();
        v2xMessageReceiver.init();
        v2xRxBackgroundThread = new Thread(v2xMessageReceiver);
        v2xRxBackgroundThread.start();
    }

        /**
     * This method is called by the AbstractFederateAmbassador when a time advance
     * has been granted by the RTI. Before this call is placed, any unprocessed
     * interaction is forwarded to the federate using the processInteraction method.
     *
     * @param time The timestamp towards which the federate can advance it local
     *             time.
     */
    @Override
    public synchronized void processTimeAdvanceGrant(long time) throws InternalFederateException {

        if (time < currentSimulationTime) {
            // process time advance only if time is equal or greater than the next
            // simulation time step
            return;
        }
        log.info("Carma message ambassador processing timestep to {}.", time);

        try {
            List<CarmaMessengerRegistrationMessage> newRegistrations = carmaMessengerRegistrationReceiver.getReceivedMessengerMessages();
            for (CarmaMessengerRegistrationMessage reg : newRegistrations) {
                carmaMessengerInstanceManager.onNewRegistration(reg);
                onDsrcRegistrationRequest(reg.getCarlaVehicleRole());
            }
            // Set current simulation time to most recent time update
            currentSimulationTime = time;
            if (currentSimulationTime == 0) {
                // For the first timestep, clear the message receive queues.
                v2xMessageReceiver.getReceivedMessages(); // Automatically empties the queues.
            } else {
                List<Tuple<InetAddress, CarmaV2xMessage>> newMessages = v2xMessageReceiver.getReceivedMessages();
                for (Tuple<InetAddress, CarmaV2xMessage> msg : newMessages) {
                    V2xMessageTransmission msgInt = carmaMessengerInstanceManager.onV2XMessageTx(msg.getA(), msg.getB(), currentSimulationTime);
                    log.debug("Generated a message with ID: {}", msgInt.getMessageId());
                    SimulationKernel.SimulationKernel.getV2xMessageCache().putItem(currentSimulationTime, msgInt.getMessage());
                    rti.triggerInteraction(msgInt);
                }
            }
            // Time Syncmessage in nano seconds
            TimeSyncMessage timeSyncMessage = new TimeSyncMessage(currentSimulationTime, timeSyncSeq);
            carmaMessengerInstanceManager.onTimeStepUpdate(timeSyncMessage);
            // Increment time 
            currentSimulationTime += carmaMessengerConfiguration.updateInterval * TIME.MILLI_SECOND;
            timeSyncSeq += 1;
           
            rti.requestAdvanceTime(currentSimulationTime, 0, (byte) 2);
        } catch (IllegalValueException e) {
            log.error("Error during advanceTime(" + time + ")", e);
            throw new InternalFederateException(e);
        } catch (UnknownHostException e) {
            log.error("Error during advanceTime(" + time + ")", e);
            throw new InternalFederateException(e);
        } catch (IOException e) {
            log.error("Error during advanceTime(" + time + ")", e);
            throw new InternalFederateException(e);
        }
         
    }

    private void onDsrcRegistrationRequest(String vehicleId) throws UnknownHostException {
        ExternalVehicleRegistration tempRegistration = new ExternalVehicleRegistration(
                currentSimulationTime,
                vehicleId,
                "carma",
                null,
                new VehicleType("carma"));

        try {
            // Trigger RTI interaction to MOSAIC to exchange the Ad-Hoc configuration
            this.rti.triggerInteraction(tempRegistration);
        } catch (InternalFederateException | IllegalValueException e) {
            // Log error message if there was an issue with the RTI interaction
            log.error(e.getMessage());
        }

        VehicleSignals tmpSignals  = new VehicleSignals(
                false,
                false,
                false,
                false,
                false);

        VehicleEmissions tmpEmissions = new VehicleEmissions(
                new Emissions(
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0
                ),
                new Emissions(
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0
                ));

        VehicleBatteryState tmpBattery = new VehicleBatteryState("", currentSimulationTime);
        IRoadPosition tmpPos = new SimpleRoadPosition("", 0, 0.0, 0.0);
        VehicleSensors tmpSensors = new VehicleSensors(
                new DistanceSensor(0.0,
                        0.0,
                        0.0,
                        0.0),
                new RadarSensor(0.0));
        VehicleConsumptions tmpConsumptions = new VehicleConsumptions(
                new Consumptions(0.0, 0.0),
                new Consumptions(0.0, 0.0));
        VehicleData tmpVehicle = new VehicleData.Builder(currentSimulationTime, vehicleId)
                .position(GeoPoint.ORIGO, CartesianPoint.ORIGO)
                .movement(0.0, 0.0, 0.0)
                .consumptions(tmpConsumptions)
                .emissions(tmpEmissions)
                .electric(tmpBattery)
                .laneArea("")
                .sensors(tmpSensors)
                .road(tmpPos)
                .signals(tmpSignals)
                .orientation(DriveDirection.FORWARD, 0.0, 0.0)
                .stopped(false)
                .route("")
                .create();
        VehicleUpdates tempUpdates = new VehicleUpdates(
                currentSimulationTime,
                new ArrayList<>(Arrays.asList(tmpVehicle)),
                new ArrayList<>(),
                new ArrayList<>());

        try {
            // Trigger RTI interaction to MOSAIC to exchange the Ad-Hoc configuration
            this.rti.triggerInteraction(tempUpdates);
        } catch (InternalFederateException | IllegalValueException e) {
            // Log error message if there was an issue with the RTI interaction
            log.error(e.getMessage());
        }


        // Create an InterfaceConfiguration object to represent the configuration of the
        // Ad-Hoc interface
        // TODO: Replace the transmit power of the ad-hoc interface (in dBm) if necessary
        // TODO: Replace the communication range of the ad-hoc interface (in meters) if necessary
        Inet4Address vehAddress = IpResolver.getSingleton().registerHost(vehicleId);
        log.info("Assigned registered comms device " + vehicleId + " with IP address " + vehAddress.toString());
        InterfaceConfiguration interfaceConfig = new InterfaceConfiguration.Builder(AdHocChannel.CCH)
                .ip(vehAddress)
                .subnet(IpResolver.getSingleton().getNetMask())
                .power(50)
                .radius(300.0)
                .create();

        // Create an AdHocConfiguration object to associate the Ad-Hoc interface
        // configuration with the infrastructure instance's ID
        AdHocConfiguration adHocConfig = new AdHocConfiguration.Builder(vehicleId)
                .addInterface(interfaceConfig)
                .create();

        // Create an AdHocCommunicationConfiguration object to specify the time and
        // Ad-Hoc configuration for exchange with another vehicle or component
        AdHocCommunicationConfiguration communicationConfig = new AdHocCommunicationConfiguration(currentSimulationTime,
                adHocConfig);
        log.info("Communications comms device " + vehicleId + " with IP address " + vehAddress.toString() + " success!");
        try {
            // Trigger RTI interaction to MOSAIC to exchange the Ad-Hoc configuration
            this.rti.triggerInteraction(communicationConfig);
        } catch (InternalFederateException | IllegalValueException e) {
            // Log error message if there was an issue with the RTI interaction
            log.error(e.getMessage());
        }
    }
}
