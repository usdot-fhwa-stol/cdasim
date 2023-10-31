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

package org.eclipse.mosaic.fed.carla.ambassador;

import com.google.common.collect.Lists;
import org.apache.commons.codec.binary.Hex;
import org.apache.commons.lang3.StringUtils;
import org.apache.xmlrpc.XmlRpcException;
import org.eclipse.mosaic.fed.carla.carlaconnect.CarlaConnection;
import org.eclipse.mosaic.fed.carla.carlaconnect.CarlaXmlRpcClient;
import org.eclipse.mosaic.fed.carla.config.CarlaConfiguration;
import org.eclipse.mosaic.fed.sumo.traci.constants.CommandSimulationControl;
import org.eclipse.mosaic.fed.sumo.traci.writer.ListTraciWriter;
import org.eclipse.mosaic.fed.sumo.traci.writer.StringTraciWriter;
import org.eclipse.mosaic.interactions.application.*;
import org.eclipse.mosaic.interactions.detector.DetectedObjectInteraction;
import org.eclipse.mosaic.interactions.detector.DetectorRegistration;
import org.eclipse.mosaic.lib.objects.detector.DetectedObject;
import org.eclipse.mosaic.lib.util.ProcessLoggingThread;
import org.eclipse.mosaic.lib.util.objects.ObjectInstantiation;
import org.eclipse.mosaic.rti.TIME;
import org.eclipse.mosaic.rti.api.*;
import org.eclipse.mosaic.rti.api.federatestarter.ExecutableFederateExecutor;
import org.eclipse.mosaic.rti.api.federatestarter.NopFederateExecutor;
import org.eclipse.mosaic.rti.api.parameters.AmbassadorParameter;
import org.eclipse.mosaic.rti.config.CLocalHost;

import javax.annotation.Nonnull;
import java.io.File;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.PriorityBlockingQueue;
import java.util.concurrent.TimeUnit;
import java.net.MalformedURLException;
import java.net.URL;

/**
 * Implementation of a {@link AbstractFederateAmbassador} for the vehicle
 * simulator CARLA. It is used to visualize the traffic simulation in 3D
 * environment.
 */
public class CarlaAmbassador extends AbstractFederateAmbassador {

    /**
     * Connection between CARLA federate and CARLA simulator.
     */
    private CarlaConnection carlaConnection = null;

    /**
     * Connection between CARLA federate and CARLA simulator with xmlrpc connection.
     */
    private CarlaXmlRpcClient carlaXmlRpcClient = null;

    /**
     * Command used to start CARLA simulator.
     */
    FederateExecutor federateExecutor = null;

    /**
     * Simulation time.
     */
    long nextTimeStep;

    /**
     * CARLA configuration file
     */
    CarlaConfiguration carlaConfig;

    /**
     * flag for simulation step
     */
    boolean isSimulationStep = false;

    /**
     * Sleep after each connection try. Unit: [ms].
     */
    private final static long SLEEP_AFTER_ATTEMPT = 1000L;

    /**
     * Maximum amount of attempts to connect to CARLA simulator.
     */
    private int connectionAttempts = 5;

    /**
     * Maximum amount of attempts to connect to CARLA simulator.
     */
     private int executedTimes = 0;

    /**
     * Carla simulator client port
     */
    private int carlaSimulatorClientPort = -1;

    /**
     * The process for running the connection bridge client
     */
    private Process connectionProcess = null;

    /**
     * Queue for temporary storage of V2X messages that CARLA vehicles receive
     */
    private final PriorityBlockingQueue<CarlaV2xMessageReception> carlaV2xInteractionQueue = new PriorityBlockingQueue<>();

    private List<DetectorRegistration> registeredDetectors = new ArrayList<>();

    /**
     * Creates a new {@link CarlaAmbassador} object.
     *
     * @param ambassadorParameter includes parameters for the CARLA Ambassador.
     */
    public CarlaAmbassador(AmbassadorParameter ambassadorParameter) {
        super(ambassadorParameter);
        try {
            // read the CARLA configuration file
            carlaConfig = new ObjectInstantiation<>(CarlaConfiguration.class, log)
                    .readFile(ambassadorParameter.configuration);
        } catch (InstantiationException e) {
            log.error("Configuration object could not be instantiated: ", e);
        }

        log.info("carlaConfig.updateInterval: " + carlaConfig.updateInterval);

        // check the carla configuration
        checkConfiguration();
    }

    /**
     * Check the updateInterval is validated.
     */
    private void checkConfiguration() {
        if (carlaConfig.updateInterval <= 0) {
            throw new RuntimeException("Invalid carla interval, should be >0");
        }
    }

    /**
     * Creates and sets new federate executor.
     *
     * @param host name of the host (as specified in /etc/hosts.json)
     * @param port port number to be used by this federate
     * @param os   operating system enum
     * @return FederateExecutor.
     */
    @Nonnull
    @Override
    public FederateExecutor createFederateExecutor(String host, int port, CLocalHost.OperatingSystem os) {
        // CARLA needs to start the federate by itself, therefore we need to store the
        // federate starter locally and use it later
        federateExecutor = new ExecutableFederateExecutor(descriptor, getCarlaExecutable("CarlaUE4"),
                getProgramArguments(port));
        this.carlaSimulatorClientPort = port;
        return new NopFederateExecutor();
    }

    /**
     * Get CARLA simulator executable file location
     *
     * @param executable the name of carla executable file
     * @return the path to CarlaUE4 executable file
     */
    String getCarlaExecutable(String executable) {
        String carlaHome = null;
        if (carlaConfig.carlaUE4Path != null) {
            carlaHome = carlaConfig.carlaUE4Path;
            log.info("use carla path from configuration file: " + carlaHome);
        }
        else if (System.getenv("CARLA_HOME") != null) {
            carlaHome = System.getenv("CARLA_HOME");
            log.info("use carla path from environmental variable: " + carlaHome);
        }
        if (StringUtils.isNotBlank(carlaHome)) {
            boolean isWindows = System.getProperty("os.name").toLowerCase().startsWith("windows");
            if (isWindows) {
                executable += ".exe";
            } else {
                executable += ".sh";
            }
            return carlaHome + File.separator + executable;
        }
        return executable;
    }

    /**
     * This method is called to tell the federate the start time and the end time.
     * It is also used to start CARLA, and connect to CARLA.
     *
     * @param startTime Start time of the simulation run in nano seconds.
     * @param endTime   End time of the simulation run in nano seconds.
     * @throws InternalFederateException Exception is thrown if an error is occurred
     *                                   while execute of a federate.
     */
    @Override
    public void initialize(long startTime, long endTime) throws InternalFederateException {
        super.initialize(startTime, endTime);

        nextTimeStep = startTime;
        try {
            rti.requestAdvanceTime(nextTimeStep, 0, (byte) 1);
        } catch (IllegalValueException e) {
            log.error("Error during advanceTime request", e);
            throw new InternalFederateException(e);
        }
        //initialize CarlaXmlRpcClient
        //set the connected server URL
        try{
            URL xmlRpcServerUrl = new URL(carlaConfig.carlaCDASimAdapterUrl);
            carlaXmlRpcClient = new CarlaXmlRpcClient(xmlRpcServerUrl);
            carlaXmlRpcClient.initialize();
        }
        catch (MalformedURLException m) 
        {
            log.error("Errors occurred with {}", m.getMessage());
        }
        // Start the CARLA simulator
        startCarlaLocal();

    }

    /**
     * Connects to CARLA simulator using the given host and port.
     *
     * @param host host on which CARLA simulator is running.
     * @param port port on which CARLA client is listening.
     */
    @Override
    public void connectToFederate(String host, int port) {
        // Start the Carla connection server
        String bridgePath = null;
        int carlaConnectionPort = 8913;
        
        if (carlaConfig.carlaConnectionPort != 0)
            carlaConnectionPort = carlaConfig.carlaConnectionPort; // set the carla connection port

        // get the connection bridge file
        if (carlaConfig.bridgePath != null) {
            bridgePath = carlaConfig.bridgePath;
            log.info("Use connection bridge path from configuration file: " + carlaConfig.bridgePath);
        } else {
            log.error("Could not find connection bridge.");
            return;
        }
        if (carlaConnection == null) {
            // start the carla connection

            carlaConnection = new CarlaConnection("localhost", carlaConnectionPort, this);
            Thread carlaThread = new Thread(carlaConnection);
            carlaThread.start();
        }

        String[] bridgePathArray = bridgePath.split(";");

        String path = bridgePathArray[0];
        String command = bridgePathArray[1];

        // check the current operating system
        boolean isWindows = System.getProperty("os.name").toLowerCase().startsWith("windows");

        if (isWindows) {
            command = "cmd.exe /c start " + command;
        } else {
            command = "sh " + command;
        }
        // connect carla client
        while (connectionAttempts-- > 0) {
            boolean connected = true;

            try {
                connectionProcess = Runtime.getRuntime().exec(command, null, new File(path));
            } catch (Exception ex) {
                ex.printStackTrace();
                if (connectionAttempts == 0) {
                    log.info("Maximum connection attempts reached and connecting to CARLA simulator failed.");
                } else {
                    log.warn("Error while connecting to CARLA simulator. Retrying.");
                }

                try {
                    Thread.sleep(SLEEP_AFTER_ATTEMPT);
                } catch (InterruptedException e) {
                    log.error("Could not execute Thread.sleep({}). Reason: {}", SLEEP_AFTER_ATTEMPT, e.getMessage());
                }
                connected = false;
            }

            if (connected) {
                log.info("Client connected");
                break;
            }
        }
    }

    @Override
    public void connectToFederate(String host, InputStream in, InputStream err) {
        this.connectToFederate(host, carlaSimulatorClientPort);
    }

    /**
     * Starts the CARLA binary locally.
     */
    void startCarlaLocal() throws InternalFederateException {
        if (!descriptor.isToStartAndStop()) {
            return;
        }

        File dir = new File(descriptor.getHost().workingDirectory, descriptor.getId());
        log.info("Start Federate local");
        log.info("Directory: " + dir);

        try {
            Process p = federateExecutor.startLocalFederate(dir);
            connectToFederate("localhost", p.getInputStream(), p.getErrorStream());
            // read error output of process in an extra thread
            new ProcessLoggingThread(log, p.getInputStream(), "carla", ProcessLoggingThread.Level.Info).start();
            new ProcessLoggingThread(log, p.getErrorStream(), "carla", ProcessLoggingThread.Level.Error).start();

        } catch (FederateExecutor.FederateStarterException e) {
            log.error("Error while executing command: {}", federateExecutor.toString());
            throw new InternalFederateException("Error while starting Carla: " + e.getLocalizedMessage());
        }
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

        if (time < nextTimeStep) {
            // process time advance only if time is equal or greater than the next
            // simulation time step
            return;
        }

        try {

            // if the simulation step received from CARLA, advance CARLA federate local
            // simulation time
            if (isSimulationStep) {
                nextTimeStep += carlaConfig.updateInterval * TIME.MILLI_SECOND;
                isSimulationStep = false;
            }
            // TODO: What is this. Why are we request a time advance based on this counter and 
            // what is it counting. It is labeled as counting the times we attempt to connect to 
            // CARLA but it seems to increment every time processTimeAdvanceGrant is called
            rti.requestAdvanceTime(nextTimeStep + this.executedTimes, 0, (byte) 2);
            this.executedTimes++;
            List<DetectedObjectInteraction> detectedObjectInteractions = new ArrayList<DetectedObjectInteraction>();
            // Get all detections from all currently registered detectors.
            for (DetectorRegistration registration: registeredDetectors ) {
                DetectedObject[] detections = carlaXmlRpcClient.getDetectedObjects( registration.getInfrastructureId() , registration.getDetector().getSensorId());
                for (DetectedObject detected: detections) {
                    detectedObjectInteractions.add(new DetectedObjectInteraction(time, detected));
                }
            }
            // trigger all detection interactions
            for (DetectedObjectInteraction detectionInteraction: detectedObjectInteractions) {
                this.rti.triggerInteraction(detectionInteraction);
            }
        } catch (IllegalValueException e) {
            log.error("Error during advanceTime(" + time + ")", e);
        }
        catch (XmlRpcException e) {
            log.error("Failed to connect to CARLA Adapter : ", e);
            carlaXmlRpcClient.closeConnection();
        }
    }

    /**
     * This method is called by the time management service to signal that the
     * simulation is finished.
     */
    @Override
    public void finishSimulation() throws InternalFederateException {
        log.info("Closing CARLA connection.");

        if (carlaConnection != null) {
            carlaConnection.closeSocket();
        }

        if (federateExecutor != null) {
            try {
                federateExecutor.stopLocalFederate();
            } catch (FederateExecutor.FederateStarterException e) {
                log.warn("Could not properly stop federate");
            }
        }

        if (connectionProcess != null) {
            try {

                connectionProcess.waitFor(10, TimeUnit.SECONDS);
            } catch (InterruptedException e) {
                log.warn("Something went wrong when stopping a process", e);
            } finally {
                connectionProcess.destroy();
            }
        }
        log.info("Finished simulation");
    }

    /**
     * get the CARLA command arguments
     *
     * @param port CARLA simulator client port
     * @return the list of CARLA command arguments
     */
    List<String> getProgramArguments(int port) {

        List<String> args = Lists.newArrayList("-carla-rpc-port", Integer.toString(port));

        return args;
    }

    /**
     * Returns whether this federate is time constrained. Is set if the federate is
     * sensitive towards the correct ordering of events. The federate ambassador
     * will ensure that the message processing happens in time stamp order. If set
     * to false, interactions will be processed will be in receive order.
     *
     * @return {@code true} if this federate is time constrained, else {@code false}
     */
    @Override
    public boolean isTimeConstrained() {
        return true;
    }

    /**
     * Returns whether this federate is time regulating. Is set if the federate
     * influences other federates and can prevent them from advancing their local
     * time.
     *
     * @return {@code true} if this federate is time regulating, {@code false} else
     */
    @Override
    public boolean isTimeRegulating() {
        return true;
    }

    /**
     * Trigger a new CarlaTraciRequest, SimulationStep or ExternalMessage
     * interaction
     *
     * @param length  command length
     * @param command command
     */
    public synchronized void triggerInteraction(int length, byte[] command) throws InternalFederateException {
        try {
            // trigger interaction based on the command type simulation step or not
            if (command[5] == CommandSimulationControl.COMMAND_SIMULATION_STEP) {
                rti.triggerInteraction(new SimulationStep(this.nextTimeStep));
                isSimulationStep = true;
                // log.debug("trigger simulation step interaction at time: " +
                // this.nextTimeStep);
            } else if (command[5] == 0x0d) {
                // send received V2X message to CARLA simulator
                sendReceivedV2xMessageToCarla();
                // log.debug("Carla ambassador sends V2X messages to bridge client.");
            } else if (command[5] == 0x2f) {
                // receive message from CARLA simulator
                String[] message = processReceivedV2xMessageFromCarla(length, command);
                if (message != null) {
                    rti.triggerInteraction(new ExternalMessage(this.nextTimeStep, message[1], message[0]));
                    // log.debug("received message from CARLA simulator: message is sent by {};
                    // message: {}", message[0],
                    // message[1]);
                }
            } else if (command[5] == 0x85) {
                log.info("Received vehicle add command from CARLA " + Hex.encodeHex(command));
            } else {
                rti.triggerInteraction(new CarlaTraciRequest(this.nextTimeStep, length, command));
            }

        } catch (IllegalValueException e) {
            throw new InternalFederateException(e);
        }
    }

    /**
     * This method is called by the {@link AbstractFederateAmbassador}s whenever the
     * federate can safely process interactions in its incoming interaction queue.
     * The decision when it is safe to process such an interaction depends on the
     * policies TimeRegulating and TimeConstrained that has to be set by the
     * federate.
     *
     * @param interaction the interaction to be processed
     */
    @Override
    public void processInteraction(Interaction interaction) {
        String type = interaction.getTypeId();
        long interactionTime = interaction.getTime();
        log.trace("Process interaction with type '{}' at time: {}", type, interactionTime);
        if (interaction.getTypeId().equals(CarlaTraciResponse.TYPE_ID)) {
            this.receiveInteraction((CarlaTraciResponse) interaction);
        } else if (interaction.getTypeId().equals(SimulationStepResponse.TYPE_ID)) {
            this.receiveInteraction((SimulationStepResponse) interaction);
        } else if (interaction.getTypeId().equals(CarlaV2xMessageReception.TYPE_ID)) {
            this.receiveInteraction((CarlaV2xMessageReception) interaction);
        }
        else if (interaction.getTypeId().equals(DetectorRegistration.TYPE_ID)) {
            this.receiveInteraction((DetectorRegistration) interaction);
        }
    }


    private void receiveInteraction(DetectorRegistration interaction) {
        try {
            carlaXmlRpcClient.createSensor(interaction);
            registeredDetectors.add(interaction);
        }
        catch(XmlRpcException e) {
            log.error("Error occurred attempting to create sensor : {}", interaction.getDetector(), e);
        }

    }

    /**
     * process the Carla traci response interaction
     *
     * @param interaction Carla Traci Response interaction
     */
    private void receiveInteraction(CarlaTraciResponse interaction) {
        try {
            // check the data output stream available
            if (carlaConnection.getDataOutputStream() != null) {
                carlaConnection.getDataOutputStream().write(interaction.getResult());
            }
        } catch (Exception e) {
            log.error("error occurs during process carla traci response interaction: {} ", e.getMessage());
        }
    }

    /**
     * process the traci response interaction
     *
     * @param interaction Simulation Step Response interaction
     */
    private void receiveInteraction(SimulationStepResponse interaction) {
        try {

            if (carlaConnection.getDataOutputStream() != null) {
                carlaConnection.getDataOutputStream().write(interaction.getResult());
                this.executedTimes = 0;
            }

        } catch (Exception e) {
            log.error("error occurs during process simulation step response interaction: {}", e.getMessage());
        }
    }

    /**
     * Process the CARLA vehicles receiving V2X message interaction
     *
     * @param interaction CarlaV2xMessageReception interaction
     */
    private void receiveInteraction(CarlaV2xMessageReception interaction) {
        log.info("{} received V2x message: {}.", interaction.getReceiverID(), interaction.getMessage());

        interactionQueue.add(interaction);
    }

    /**
     * Send received V2X message to CARLA simulator
     */
    private void sendReceivedV2xMessageToCarla() {
        List<String> v2xMessageSent = new ArrayList<>();
        int totoalBytesSent = 6;
        while (!carlaV2xInteractionQueue.isEmpty()) {
            if (carlaV2xInteractionQueue.peek().getTime() > nextTimeStep)
                break;
            CarlaV2xMessageReception carlaV2xMessageReception = carlaV2xInteractionQueue.poll();
            if (carlaV2xMessageReception != null) {
                String message = "Time: " + carlaV2xMessageReception.getTime() + "; Receiver ID: "
                        + carlaV2xMessageReception.getReceiverID() + "; Message: "
                        + carlaV2xMessageReception.getMessage() + ".";

                totoalBytesSent += message.length() + 4;

                v2xMessageSent.add(message);
            }
        }
        if (totoalBytesSent > 255) {
            totoalBytesSent += 4;
        }
        try {
            // send messages to client
            if (carlaConnection.getDataOutputStream() != null) {
                carlaConnection.getDataOutputStream().writeInt(totoalBytesSent + 11);
                carlaConnection.getDataOutputStream().write(new byte[] { 0x07, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x00 });
                if (totoalBytesSent - 4 > 255) {
                    carlaConnection.getDataOutputStream().writeByte(0);
                    carlaConnection.getDataOutputStream().writeInt(totoalBytesSent);
                } else {
                    carlaConnection.getDataOutputStream().writeByte(totoalBytesSent);
                }
                carlaConnection.getDataOutputStream().writeByte(0x0d);
                if (!v2xMessageSent.isEmpty()) {
                    ListTraciWriter<String> listTraci = new ListTraciWriter<String>(new StringTraciWriter());
                    listTraci.writeVariableArgument(carlaConnection.getDataOutputStream(), v2xMessageSent);
                } else {
                    carlaConnection.getDataOutputStream().writeInt(0);
                }
            }
        } catch (Exception e) {
            log.error("error occurs during sending messages to bridge: " + e.getMessage());
        }
    }

    /**
     * Process the received messages from CARLA simulator.
     *
     * @param length  the length of command
     * @param command received command
     * @return received external message
     */
    private String[] processReceivedV2xMessageFromCarla(int length, byte[] command) {

        String message;
        if (command[4] == 0) {
            message = new String(Arrays.copyOfRange(command, 15, length));
        } else {
            message = new String(Arrays.copyOfRange(command, 11, length));
        }
        try {
            // send response to client
            if (carlaConnection.getDataOutputStream() != null) {
                carlaConnection.getDataOutputStream().writeInt(11);
                carlaConnection.getDataOutputStream().write(new byte[] { 0x07, 0x2f, 0x00, 0x00, 0x00, 0x00, 0x00 });
            }
        } catch (Exception e) {
            log.error("error occurs during process received messages: " + e.getMessage());
        }
        return message.split(";");
    }
}
