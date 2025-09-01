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
// import org.eclipse.mosaic.fed.sumo.traci.writer.ListTraciWriter;
// import org.eclipse.mosaic.fed.sumo.traci.writer.StringTraciWriter;
import org.eclipse.mosaic.interactions.application.*;
import org.eclipse.mosaic.interactions.detector.DetectedObjectInteraction;
import org.eclipse.mosaic.interactions.detector.DetectorRegistration;
import org.eclipse.mosaic.interactions.application.CarlaTrafficLightRequest;
import org.eclipse.mosaic.interactions.application.CarlaActorRequest;
import org.eclipse.mosaic.interactions.application.CarlaTrafficLightResponse;
import org.eclipse.mosaic.interactions.application.CarlaActorResponse;
import org.eclipse.mosaic.interactions.application.SimulationStep;

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
import java.util.Map;
import java.util.HashMap;

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
        // Start the CARLA simulator
        startCarlaLocal();
        //initialize CarlaXmlRpcClient
        //set the connected server URL
        try{
            if (carlaXmlRpcClient== null) {
                URL xmlRpcServerUrl = new URL(carlaConfig.carlaCDASimAdapterUrl);
                carlaXmlRpcClient = new CarlaXmlRpcClient(xmlRpcServerUrl);
            }
            
        }
        catch (MalformedURLException m) 
        {
            throw new InternalFederateException("Carla Ambassador initialization failed due to CARLA CDA Sim Adapter" 
                + "connection! Check carla_config.json!", m);
        }
        
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
            if ( time == 0 ) {
                // Try to connect to CARLA CDA Sim Adapter on first timestep
                carlaXmlRpcClient.connect(60);
            }
            // if the simulation step received from CARLA, advance CARLA federate local
            // simulation time
            if (isSimulationStep) {
                List<DetectedObjectInteraction> detectedObjectInteractions = new ArrayList<>();
                // Get all detections from all currently registered detectors.
                for (DetectorRegistration registration: registeredDetectors ) {
                    DetectedObject[] detections = carlaXmlRpcClient.getDetectedObjects( registration.getInfrastructureId() , registration.getDetector().getSensorId());
                    for (DetectedObject detected: detections) {
                        DetectedObjectInteraction interaction = new DetectedObjectInteraction(time, detected);
                        // Convert nanosecond timestamp to millisecond timestamp
                        interaction.getDetectedObject().setTimestamp((int)(time/1e6));
                        detectedObjectInteractions.add(interaction);
                    }
                }
                // trigger all detection interactions
                for (DetectedObjectInteraction detectionInteraction: detectedObjectInteractions) {
                    this.rti.triggerInteraction(detectionInteraction);
                }

                // Emit CARLA state updates towards SUMO: actors and traffic lights
                try {
                    // Actors
                    java.util.Map<String, java.util.Map<String, Object>> actors = carlaXmlRpcClient.getAllActors();
                    for (java.util.Map.Entry<String, java.util.Map<String, Object>> entry : actors.entrySet()) {
                        String actorId = entry.getKey();
                        java.util.Map<String, Object> info = entry.getValue();

                        java.util.List<Double> loc = null;
                        java.util.List<Double> rot = null;
                        java.util.List<Double> vel = null;

                        Object t = info.get("transform");
                        if (t instanceof java.util.Map) {
                            Object l = ((java.util.Map<?,?>) t).get("location");
                            Object r = ((java.util.Map<?,?>) t).get("rotation");
                            if (l instanceof java.util.List) {
                                // assume [x,y,z]
                                loc = new java.util.ArrayList<>();
                                for (Object o : (java.util.List<?>) l) if (o instanceof Number) loc.add(((Number)o).doubleValue());
                            }
                            if (r instanceof java.util.List) {
                                // assume [pitch,yaw,roll]
                                rot = new java.util.ArrayList<>();
                                for (Object o : (java.util.List<?>) r) if (o instanceof Number) rot.add(((Number)o).doubleValue());
                            }
                        }
                        Object v = info.get("velocity");
                        if (v instanceof java.util.List) {
                            vel = new java.util.ArrayList<>();
                            for (Object o : (java.util.List<?>) v) if (o instanceof Number) vel.add(((Number)o).doubleValue());
                        }

                        this.rti.triggerInteraction(new org.eclipse.mosaic.interactions.application.CarlaActorResponse(time, actorId, loc, rot, vel, null));
                    }

                    // Traffic lights
                    java.util.List<java.util.Map<String, Object>> tlStates = carlaXmlRpcClient.getAllTrafficLightStates();
                    for (java.util.Map<String, Object> tl : tlStates) {
                        Object id = tl.get("id");
                        Object state = tl.get("state");
                        Object timer = tl.get("timer");
                        String idStr = id != null ? id.toString() : null;
                        String stateStr = state != null ? state.toString() : null;
                        Double timerVal = null;
                        if (timer instanceof Number) timerVal = ((Number) timer).doubleValue();
                        if (idStr != null && stateStr != null) {
                            this.rti.triggerInteraction(new org.eclipse.mosaic.interactions.application.CarlaTrafficLightResponse(time, idStr, stateStr, timerVal));
                        }
                    }
                } catch (Exception e) {
                    log.warn("Failed to poll and emit CARLA state updates: {}", e.getMessage());
                }
                nextTimeStep += carlaConfig.updateInterval * TIME.MILLI_SECOND;
                isSimulationStep = false;
                rti.requestAdvanceTime(nextTimeStep , 0, (byte) 2);
            }
            
        } 
        catch (IllegalValueException e) {
            log.error("Failed to process advance time grant due to : ", e);
        }
        catch (XmlRpcException e ) {
            throw new InternalFederateException("Failed to process advance time grant due to CARLA CDA Sim "
                        + "Adapter connection! Check carla_config.json!", e);
        }
        catch (InterruptedException e) {
            log.error("Failed to process advance time grant due to failed thread sleep!", e);
            Thread.currentThread().interrupt();
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
                Thread.currentThread().interrupt();
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
     * Trigger interactions based on commands received from CARLA simulator.
     * Now uses XML-RPC for simulation advancement instead of TraCI.
     *
     * @param length  command length
     * @param command command
     */
    public synchronized void triggerInteraction(int length, byte[] command) throws InternalFederateException {
        try {
            // Handle different command types using XML-RPC approach
            if (command[5] == CommandSimulationControl.COMMAND_SIMULATION_STEP) {
                // Use XML-RPC to advance simulation instead of TraCI-based SimulationStep
                boolean advanced = carlaXmlRpcClient.advanceSimulation();
                if (advanced) {
                    // Trigger internal simulation step coordination
                    triggerInternalSimulationStep();
                    log.debug("CARLA simulation advanced via XML-RPC at time: {}", this.nextTimeStep);
                } else {
                    log.warn("Failed to advance CARLA simulation via XML-RPC");
                }
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
                    // message: {}", message[1]);
                }
            } else if (command[5] == 0x85) {
                log.info("Received vehicle add command from CARLA " + Hex.encodeHex(command));
            } else {
                log.debug("Ignoring legacy TraCI request path in favor of XML-RPC interactions");
            }

        } catch (IllegalValueException e) {
            throw new InternalFederateException(e);
        }
    }

    /**
     * Internal method to trigger simulation step coordination.
     * This replaces the external SimulationStep interaction with internal logic.
     */
    private void triggerInternalSimulationStep() {
        // Set simulation step flag to trigger state updates and time advancement
        isSimulationStep = true;
        
        // Optionally, we can still trigger a SimulationStep interaction for other federates
        // that might need to know about simulation advancement
        try {
            rti.triggerInteraction(new SimulationStep(this.nextTimeStep));
        } catch (Exception e) {
            log.warn("Failed to trigger SimulationStep interaction: {}", e.getMessage());
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
        
        // Handle interactions using XML-RPC calls
        if (interaction.getTypeId().equals(CarlaV2xMessageReception.TYPE_ID)) {
            this.receiveInteraction((CarlaV2xMessageReception) interaction);
        }
        else if (interaction.getTypeId().equals(DetectorRegistration.TYPE_ID)) {
            this.receiveInteraction((DetectorRegistration) interaction);
        }
        else if (interaction.getTypeId().equals(org.eclipse.mosaic.interactions.application.CarlaActorRequest.TYPE_ID)) {
            this.receiveInteraction((org.eclipse.mosaic.interactions.application.CarlaActorRequest) interaction);
        }
        else if (interaction.getTypeId().equals(org.eclipse.mosaic.interactions.application.CarlaTrafficLightRequest.TYPE_ID)) {
            this.receiveInteraction((org.eclipse.mosaic.interactions.application.CarlaTrafficLightRequest) interaction);
        }
    }

    /**
     * Method to call XMLRPC method to create sensor on reception of DetectionRegistration interactions. 
     * @param interaction Interaction triggered by Ambassadors attempting to create sensors in CARLA.
     * @throws InterruptedException
     */
    private void receiveInteraction(DetectorRegistration interaction) {
        try {
            carlaXmlRpcClient.createSensor(interaction);
            registeredDetectors.add(interaction);
        }
        catch(XmlRpcException e) {
            log.error("Error occurred attempting to create sensor : {}\n{}", interaction.getDetector(), e);
        }

    }

    private void receiveInteraction(CarlaActorRequest interaction) {
        try {
            boolean ok = true;
            if (interaction.getAction() == CarlaActorRequest.Action.CREATE) {
                ok = carlaXmlRpcClient.spawnActor(
                    interaction.getActorType(), interaction.getActorId(),
                    interaction.getLocation(), interaction.getRotation(), interaction.getProperties());
            } else if (interaction.getAction() == CarlaActorRequest.Action.UPDATE) {
                if (interaction.getLocation() != null || interaction.getRotation() != null) {
                    ok &= carlaXmlRpcClient.updateActorTransform(interaction.getActorId(), interaction.getLocation(), interaction.getRotation());
                }
                if (interaction.getVelocity() != null) {
                    ok &= carlaXmlRpcClient.updateActorVelocity(interaction.getActorId(), interaction.getVelocity());
                }
                if (interaction.getProperties() != null) {
                    ok &= carlaXmlRpcClient.setActorStateProperties(interaction.getActorId(), interaction.getProperties());
                }
            } else if (interaction.getAction() == CarlaActorRequest.Action.DESTROY) {
                ok = carlaXmlRpcClient.destroyActor(interaction.getActorId());
            }
            if (!ok) {
                log.warn("CarlaActorRequest action {} failed for actor {}", interaction.getAction(), interaction.getActorId());
            }
        } catch (Exception e) {
            log.error("Error while processing CarlaActorRequest for {}: {}", interaction.getActorId(), e.getMessage());
        }
    }

    private void receiveInteraction(CarlaTrafficLightRequest interaction) {
        try {
            if (interaction.getAction() == CarlaTrafficLightRequest.Action.UPDATE) {
                boolean ok = carlaXmlRpcClient.setTrafficLightState(interaction.getTrafficLightId(), interaction.getState());
                if (!ok) {
                    log.warn("Failed to set traffic light {} state {}", interaction.getTrafficLightId(), interaction.getState());
                }
                if (interaction.getTimerSeconds() != null) {
                    boolean timerOk = carlaXmlRpcClient.setTrafficLightTimer(interaction.getTrafficLightId(), interaction.getTimerSeconds());
                    if (!timerOk) {
                        log.warn("Failed to set traffic light {} timer {}", interaction.getTrafficLightId(), interaction.getTimerSeconds());
                    }
                }
            }
        } catch (Exception e) {
            log.error("Error while processing CarlaTrafficLightRequest for {}: {}", interaction.getTrafficLightId(), e.getMessage());
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
     * Send received V2X message to CARLA simulator via XML-RPC
     */
    private void sendReceivedV2xMessageToCarla() {
        List<Map<String, Object>> v2xMessages = new ArrayList<>();
        
        while (!carlaV2xInteractionQueue.isEmpty()) {
            if (carlaV2xInteractionQueue.peek().getTime() > nextTimeStep)
                break;
                
            CarlaV2xMessageReception carlaV2xMessageReception = carlaV2xInteractionQueue.poll();
            if (carlaV2xMessageReception != null) {
                Map<String, Object> messageData = new HashMap<>();
                messageData.put("timestamp", carlaV2xMessageReception.getTime());
                messageData.put("receiverId", carlaV2xMessageReception.getReceiverID());
                messageData.put("message", carlaV2xMessageReception.getMessage());
                messageData.put("senderId", "MOSAIC_SUMO");
                
                v2xMessages.add(messageData);
            }
        }
        
        if (!v2xMessages.isEmpty()) {
            try {
                // Send V2X messages via XML-RPC instead of TraCI bridge
                for (Map<String, Object> messageData : v2xMessages) {
                    // Use XML-RPC client to send V2X message to CARLA
                    // This assumes CARLA XML-RPC server has a method to handle V2X messages
                    boolean sent = carlaXmlRpcClient.sendV2xMessage(
                        (String) messageData.get("receiverId"),
                        (String) messageData.get("message"),
                        (String) messageData.get("senderId"),
                        (Long) messageData.get("timestamp")
                    );
                    
                    if (sent) {
                        log.debug("V2X message sent to CARLA via XML-RPC: {}", messageData);
                    } else {
                        log.warn("Failed to send V2X message to CARLA via XML-RPC: {}", messageData);
                    }
                }
            } catch (Exception e) {
                log.error("Error sending V2X messages to CARLA via XML-RPC: {}", e.getMessage());
            }
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
            log.error("error occurs during process received messages: {}",  e.getMessage());
        }
        return message.split(";");
    }
}
