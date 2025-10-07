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
import org.eclipse.mosaic.fed.carla.carlaconnect.CarlaMultiXmlRpcManager;
import org.eclipse.mosaic.fed.carla.config.CarlaConfiguration;
import org.eclipse.mosaic.fed.sumo.traci.constants.CommandSimulationControl;
import org.eclipse.mosaic.fed.sumo.traci.writer.ListTraciWriter;
import org.eclipse.mosaic.fed.sumo.traci.writer.StringTraciWriter;
import org.eclipse.mosaic.interactions.application.*;
import org.eclipse.mosaic.interactions.traffic.VehicleUpdates;
import org.eclipse.mosaic.interactions.traffic.TrafficLightUpdates;
import org.eclipse.mosaic.interactions.traffic.TrafficLightStateChange;
import org.eclipse.mosaic.interactions.detector.DetectedObjectInteraction;
import org.eclipse.mosaic.interactions.detector.DetectorRegistration;
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
import java.util.Set;
import java.util.HashSet;
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
     * Multi-connection manager for multiple XML-RPC servers
     */
    private CarlaMultiXmlRpcManager multiXmlRpcManager = null;

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
     * Cache of current CARLA actor ids for quick existence checks during synchronization.
     */
    private final Set<String> currentActorIds = new HashSet<>();


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
            // If configured, bypass launcher script to avoid chmod attempts inside it
            if (Boolean.TRUE.equals(carlaConfig.useDirectBinary)) {
                if (isWindows) {
                    return carlaHome + File.separator + "CarlaUE4.exe";
                } else {
                    return carlaHome + File.separator + "CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping";
                }
            } else {
                if (isWindows) {
                    executable += ".exe";
                } else {
                    executable += ".sh";
                }
                return carlaHome + File.separator + executable;
            }
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
        
        // Initialize XML-RPC connections
        if (carlaConfig.carlaSensorLibRPCUrl != null || carlaConfig.carlaActorLibRPCUrl != null) {
            // Use multi-server manager for separate sensor and actor connections
            multiXmlRpcManager = new CarlaMultiXmlRpcManager();
            
            try {
                // Add sensor library server
                // convert string null to null
                if ("null".equalsIgnoreCase(carlaConfig.carlaSensorLibRPCUrl)) {
                    carlaConfig.carlaSensorLibRPCUrl = null;
                }
                if ("null".equalsIgnoreCase(carlaConfig.carlaActorLibRPCUrl)) {
                    carlaConfig.carlaActorLibRPCUrl = null;
                }
                if (carlaConfig.carlaSensorLibRPCUrl != null) {
                    log.info("Start adding Sensor_LIB server: {}", carlaConfig.carlaSensorLibRPCUrl);
                    multiXmlRpcManager.addClient(CarlaXmlRpcClient.ServerType.SENSOR_LIB, carlaConfig.carlaSensorLibRPCUrl);
                    log.info("Added SENSOR_LIB server: {}", carlaConfig.carlaSensorLibRPCUrl);
                }
                
                // Add actor library server
                if (carlaConfig.carlaActorLibRPCUrl != null) {
                    log.info("Start adding ACTOR_LIB server: {}", carlaConfig.carlaActorLibRPCUrl);
                    multiXmlRpcManager.addClient(CarlaXmlRpcClient.ServerType.ACTOR_LIB, carlaConfig.carlaActorLibRPCUrl);
                    log.info("Added ACTOR_LIB server: {}", carlaConfig.carlaActorLibRPCUrl);
                }
                
                if (multiXmlRpcManager.getClient(CarlaXmlRpcClient.ServerType.SENSOR_LIB) == null &&
                    multiXmlRpcManager.getClient(CarlaXmlRpcClient.ServerType.ACTOR_LIB) == null) {
                    throw new InternalFederateException("No XML-RPC servers configured for multi-server mode");
                }
                
            } catch (MalformedURLException m) {
                throw new InternalFederateException("Carla Ambassador initialization failed due to invalid XML-RPC server URLs! Check carla_config.json!");
            }
        } else {
            // Legacy single connection mode
            if (carlaXmlRpcClient == null) {
                // For CARLA Sensor Lib Connection
                if (carlaConfig.carlaSensorLibRPCUrl != null){
                    try {
                        URL xmlRpcServerUrl = new URL(carlaConfig.carlaSensorLibRPCUrl);
                        carlaXmlRpcClient = new CarlaXmlRpcClient(xmlRpcServerUrl, CarlaXmlRpcClient.ServerType.SENSOR_LIB);
                    } catch (MalformedURLException m) {
                        throw new InternalFederateException("Carla Ambassador initialization failed due to CARLA CDA Sim Adapter"
                            + "connection! Check carla_config.json!", m);
                    }
                }

                // For CARLA Actor Lib Connection
                if (carlaConfig.carlaActorLibRPCUrl != null){
                    try {
                        URL xmlRpcServerUrl = new URL(carlaConfig.carlaActorLibRPCUrl);
                        carlaXmlRpcClient = new CarlaXmlRpcClient(xmlRpcServerUrl, CarlaXmlRpcClient.ServerType.ACTOR_LIB);
                    } catch (MalformedURLException m) {
                        throw new InternalFederateException("Carla Ambassador initialization failed due to CARLA CDA Sim Adapter"
                            + "connection! Check carla_config.json!", m);
                    }
                }
            }
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
            if (time == 0) {
                // Try to connect to XML-RPC servers on first timestep
                if (multiXmlRpcManager != null) {
                    multiXmlRpcManager.connectAll(60);
                    // After connecting, configure server input frame and SUMO net offset if provided
                    try {
                        org.eclipse.mosaic.fed.carla.carlaconnect.CarlaXmlRpcClient actorClient = multiXmlRpcManager.getClient(CarlaXmlRpcClient.ServerType.ACTOR_LIB);
                        if (actorClient != null && actorClient.isConnected()) {
                            actorClient.setInputFrameMode("sumo");
                            // double[] netOffset = readSumoNetOffsetFromEnv();
                            // actorClient.setNetOffsetXY(netOffset[0], netOffset[1]);
                        }
                    } catch (Exception ignore) { }
                } else if (carlaXmlRpcClient != null) {
                    carlaXmlRpcClient.connect(60);
                    try {
                        carlaXmlRpcClient.setInputFrameMode("sumo");
                        // double[] netOffset = readSumoNetOffsetFromEnv();
                        // carlaXmlRpcClient.setNetOffsetXY(netOffset[0], netOffset[1]);
                    } catch (Exception ignore) { }
                }
            }
            // if the simulation step received from CARLA, advance CARLA federate local
            // simulation time
            if (isSimulationStep) {
                
                // Handle sensor operations
                boolean sensorConnected = false;
                if (multiXmlRpcManager != null) {
                    sensorConnected = multiXmlRpcManager.isConnected(CarlaXmlRpcClient.ServerType.SENSOR_LIB);
                } else if (carlaXmlRpcClient != null && carlaXmlRpcClient.getServerType() == CarlaXmlRpcClient.ServerType.SENSOR_LIB) {
                    sensorConnected = carlaXmlRpcClient.isConnected();
                }
                
                if (sensorConnected) {
                    List<DetectedObjectInteraction> detectedObjectInteractions = new ArrayList<>();
                    // Get all detections from all currently registered detectors.
                    for (DetectorRegistration registration: registeredDetectors ) {
                        DetectedObject[] detections;
                        if (multiXmlRpcManager != null) {
                            detections = multiXmlRpcManager.getClient(CarlaXmlRpcClient.ServerType.SENSOR_LIB).getDetectedObjects(registration.getInfrastructureId(), registration.getDetector().getSensorId());
                        } else {
                            detections = carlaXmlRpcClient.getDetectedObjects(registration.getInfrastructureId(), registration.getDetector().getSensorId());
                        }
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
                }
                // Handle actor operations
                boolean actorConnected = false;
                if (multiXmlRpcManager != null) {
                    actorConnected = multiXmlRpcManager.isConnected(CarlaXmlRpcClient.ServerType.ACTOR_LIB);
                } else if (carlaXmlRpcClient != null && carlaXmlRpcClient.getServerType() == CarlaXmlRpcClient.ServerType.ACTOR_LIB) {
                    actorConnected = carlaXmlRpcClient.isConnected();
                }
                
                if (actorConnected) {
                    // Publish CARLA state updates to SUMO using VehicleUpdates and TrafficLightUpdates
                    try {
                        CarlaXmlRpcClient actorClient = null;
                        if (multiXmlRpcManager != null) {
                            actorClient = multiXmlRpcManager.getClient(CarlaXmlRpcClient.ServerType.ACTOR_LIB);
                        } else {
                            actorClient = carlaXmlRpcClient;
                        }
                        
                        // Use Client's high-level change detection
                        java.util.Map<String, Object> actorChanges = actorClient.getActorChanges();
                        java.util.List<java.util.Map<String, Object>> addedActors = (java.util.List<java.util.Map<String, Object>>) actorChanges.get("added");
                        java.util.List<java.util.Map<String, Object>> updatedActors = (java.util.List<java.util.Map<String, Object>>) actorChanges.get("updated");
                        java.util.List<String> removedActors = (java.util.List<String>) actorChanges.get("removed");
                        
                        // Convert to VehicleData objects
                        java.util.List<org.eclipse.mosaic.lib.objects.vehicle.VehicleData> addedVehicles = new java.util.ArrayList<>();
                        java.util.List<org.eclipse.mosaic.lib.objects.vehicle.VehicleData> updatedVehicles = new java.util.ArrayList<>();
                        
                        // Process added actors
                        for (java.util.Map<String, Object> actorInfo : addedActors) {
                            // Extract actor ID from the actor info (assuming it's stored as a key in the original map)
                            // This is a simplified approach - in practice, you might need to store actor ID differently
                            String actorId = actorInfo.get("id") != null ? actorInfo.get("id").toString() : "unknown";
                            org.eclipse.mosaic.lib.objects.vehicle.VehicleData vehicleData = actorClient.createVehicleDataFromActor(actorId, actorInfo);
                            if (vehicleData != null) {
                                addedVehicles.add(vehicleData);
                            }
                        }
                        
                        // Process updated actors
                        for (java.util.Map<String, Object> actorInfo : updatedActors) {
                            String actorId = actorInfo.get("id") != null ? actorInfo.get("id").toString() : "unknown";
                            org.eclipse.mosaic.lib.objects.vehicle.VehicleData vehicleData = actorClient.createVehicleDataFromActor(actorId, actorInfo);
                            if (vehicleData != null) {
                                updatedVehicles.add(vehicleData);
                            }
                        }
                        
                        // Update current actor IDs cache
                        currentActorIds.clear();
                        java.util.Map<String, java.util.Map<String, Object>> allActors = actorClient.getAllActors();
                        currentActorIds.addAll(allActors.keySet());
                        
                        // Publish VehicleUpdates if there are changes
                        if (!addedVehicles.isEmpty() || !updatedVehicles.isEmpty() || !removedActors.isEmpty()) {
                            VehicleUpdates vehicleUpdates = new VehicleUpdates(time, addedVehicles, updatedVehicles, removedActors);
                            this.rti.triggerInteraction(vehicleUpdates);
                            log.debug("Published VehicleUpdates: added={}, updated={}, removed={}", 
                                addedVehicles.size(), updatedVehicles.size(), removedActors.size());
                        }

                        // Handle traffic lights using Client's change detection
                        java.util.Map<String, java.util.Map<String, Object>> trafficLightChanges = actorClient.getTrafficLightChanges();
                        
                        if (!trafficLightChanges.isEmpty()) {
                            java.util.Map<String, org.eclipse.mosaic.lib.objects.trafficlight.TrafficLightGroupInfo> updatedTrafficLights = new java.util.HashMap<>();
                            
                            for (java.util.Map.Entry<String, java.util.Map<String, Object>> entry : trafficLightChanges.entrySet()) {
                                String id = entry.getKey();
                                java.util.Map<String, Object> tlInfo = entry.getValue();
                                
                                String state = tlInfo.get("state") != null ? tlInfo.get("state").toString() : "Unknown";
                                Double timer = tlInfo.get("timer") instanceof Number ? ((Number) tlInfo.get("timer")).doubleValue() : null;
                                
                                // Create a simple TrafficLightGroupInfo with basic information
                                // Since we don't have full SUMO traffic light program details from CARLA,
                                // we'll create a minimal representation
                                java.util.List<org.eclipse.mosaic.lib.objects.trafficlight.TrafficLightState> states = new java.util.ArrayList<>();
                                // Add a basic state representation - TrafficLightState constructor takes (red, green, yellow) booleans
                                states.add(new org.eclipse.mosaic.lib.objects.trafficlight.TrafficLightState(true, false, false)); // Red state
                                
                                org.eclipse.mosaic.lib.objects.trafficlight.TrafficLightGroupInfo tlGroupInfo = 
                                    new org.eclipse.mosaic.lib.objects.trafficlight.TrafficLightGroupInfo(
                                        id, 
                                        "default", // program ID
                                        0, // phase index
                                        timer != null ? (long)(timer * 1e9) : 0, // convert seconds to nanoseconds
                                        states
                                    );
                                updatedTrafficLights.put(id, tlGroupInfo);
                            }
                            
                            TrafficLightUpdates trafficLightUpdates = new TrafficLightUpdates(time, updatedTrafficLights);
                            this.rti.triggerInteraction(trafficLightUpdates);
                            log.debug("Published TrafficLightUpdates: {} traffic lights updated", updatedTrafficLights.size());
                        }
                        
                    } catch (Exception e) {
                        log.warn("Failed to poll and emit CARLA state updates: {}", e.getMessage());
                    }
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

        // Disconnect from XML-RPC servers
        if (multiXmlRpcManager != null) {
            multiXmlRpcManager.disconnectAll();
        } else if (carlaXmlRpcClient != null) {
            carlaXmlRpcClient.disconnect();
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
                boolean advanced = false;
                if (multiXmlRpcManager != null) {
                    advanced = multiXmlRpcManager.getClient(CarlaXmlRpcClient.ServerType.ACTOR_LIB).advanceSimulation();
                } else if (carlaXmlRpcClient != null) {
                    advanced = carlaXmlRpcClient.advanceSimulation();
                }
                
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
     * Read SUMO netOffset from environment or scenario config if available.
     * Falls back to (0,0) if not provided.
     */
    private double[] readSumoNetOffsetFromEnv() {
        try {
            String xStr = System.getenv("SUMO_NET_OFFSET_X");
            String yStr = System.getenv("SUMO_NET_OFFSET_Y");
            if (xStr != null && yStr != null) {
                return new double[]{Double.parseDouble(xStr), Double.parseDouble(yStr)};
            }
        } catch (Exception ignore) { }
        // TODO: optionally parse scenario net.xml to read <location netOffset="x,y"> if paths are known
        return new double[]{0.0, 0.0};
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
        log.info("Processing interaction with type '{}' at time: {}", type, interactionTime);
        
        // Handle interactions using XML-RPC calls
        if (interaction.getTypeId().equals(CarlaV2xMessageReception.TYPE_ID)) {
            log.info("Processing CarlaV2xMessageReception interaction");
            this.receiveInteraction((CarlaV2xMessageReception) interaction);
        }
        else if (interaction.getTypeId().equals(DetectorRegistration.TYPE_ID)) {
            log.info("Processing DetectorRegistration interaction");
            this.receiveInteraction((DetectorRegistration) interaction);
        }
        else if (interaction.getTypeId().equals(VehicleUpdates.TYPE_ID)) {
            log.info("Processing VehicleUpdates interaction - this should trigger spawn_actor calls");
            this.receiveInteraction((VehicleUpdates) interaction);
        }
        else if (interaction.getTypeId().equals(TrafficLightStateChange.TYPE_ID)) {
            log.info("Processing TrafficLightStateChange interaction - this should forward traffic light commands to CARLA");
            this.receiveInteraction((TrafficLightStateChange) interaction);
        }
        else {
            log.debug("Ignoring interaction of type: {}", type);
        }
    }

    /**
     * Method to call XMLRPC method to create sensor on reception of DetectionRegistration interactions. 
     * @param interaction Interaction triggered by Ambassadors attempting to create sensors in CARLA.
     * @throws InterruptedException
     */
    private void receiveInteraction(DetectorRegistration interaction) {
        boolean sensorConnected = false;
        if (multiXmlRpcManager != null) {
            sensorConnected = multiXmlRpcManager.isConnected(CarlaXmlRpcClient.ServerType.SENSOR_LIB);
        } else if (carlaXmlRpcClient != null && carlaXmlRpcClient.getServerType() == CarlaXmlRpcClient.ServerType.SENSOR_LIB) {
            sensorConnected = carlaXmlRpcClient.isConnected();
        }
        
        if (sensorConnected) {
            try {
                if (multiXmlRpcManager != null) {
                    multiXmlRpcManager.getClient(CarlaXmlRpcClient.ServerType.SENSOR_LIB).createSensor(interaction);
                } else {
                    carlaXmlRpcClient.createSensor(interaction);
                }
                registeredDetectors.add(interaction);
            }
            catch(XmlRpcException e) {
                log.error("Error occurred attempting to create sensor : {}\n{}", interaction.getDetector(), e);
            }
        } else {
            log.warn("Sensor server not connected, cannot create sensor: {}", interaction.getDetector().getSensorId());
        }
    }


    /**
     * Synchronize CARLA with SUMO vehicle updates.
     * - Spawn missing CARLA actors for SUMO vehicles in added/updated lists
     * - Update transforms for existing ones
     * - Destroy CARLA actors for SUMO removed vehicles
     */
    private void receiveInteraction(VehicleUpdates interaction) {
        log.info("Received VehicleUpdates interaction at time {}: added={}, updated={}, removed={}", 
                interaction.getTime(), 
                interaction.getAdded() != null ? interaction.getAdded().size() : 0,
                interaction.getUpdated() != null ? interaction.getUpdated().size() : 0,
                interaction.getRemovedNames() != null ? interaction.getRemovedNames().size() : 0);
        
        boolean actorConnected = false;
        if (multiXmlRpcManager != null) {
            actorConnected = multiXmlRpcManager.isConnected(CarlaXmlRpcClient.ServerType.ACTOR_LIB);
            log.info("Multi-XML-RPC manager actor connection status: {}", actorConnected);
        } else if (carlaXmlRpcClient != null && carlaXmlRpcClient.getServerType() == CarlaXmlRpcClient.ServerType.ACTOR_LIB) {
            actorConnected = carlaXmlRpcClient.isConnected();
            log.info("Single XML-RPC client actor connection status: {}", actorConnected);
        } else {
            log.warn("No XML-RPC client configured for ACTOR_LIB");
        }

        if (!actorConnected) {
            log.warn("Actor server not connected; skip SUMO->CARLA sync. multiXmlRpcManager={}, carlaXmlRpcClient={}", 
                    multiXmlRpcManager != null, carlaXmlRpcClient != null);
            return;
        }

        try {
            // Log incoming sync request counts
            int numAdded = interaction.getAdded() != null ? interaction.getAdded().size() : 0;
            int numUpdated = interaction.getUpdated() != null ? interaction.getUpdated().size() : 0;
            int numRemoved = interaction.getRemovedNames() != null ? interaction.getRemovedNames().size() : 0;
            log.info("Starting SUMO->CARLA vehicle sync: added={}, updated={}, removed={}", numAdded, numUpdated, numRemoved);

            // Ensure we have up-to-date list of CARLA actors
            java.util.Map<String, java.util.Map<String, Object>> actors;
            if (multiXmlRpcManager != null) {
                actors = multiXmlRpcManager.getClient(CarlaXmlRpcClient.ServerType.ACTOR_LIB).getAllActors();
            } else {
                actors = carlaXmlRpcClient.getAllActors();
            }
            currentActorIds.clear();
            currentActorIds.addAll(actors.keySet());

            // Helper to spawn/update
            java.util.function.Consumer<org.eclipse.mosaic.lib.objects.vehicle.VehicleData> applyVehicle = vd -> {
                String id = vd.getName();
                // Build location [x,y,z] from projected position (x,y), z=0 by default
                java.util.List<Double> location = new java.util.ArrayList<>(3);
                double x = vd.getProjectedPosition() != null ? vd.getProjectedPosition().getX() : 0.0;
                double y = vd.getProjectedPosition() != null ? vd.getProjectedPosition().getY() : 0.0;
                location.add(x);
                location.add(y);
                location.add(0.0);
                // Rotation [pitch,yaw,roll] where yaw from heading if available
                java.util.List<Double> rotation = new java.util.ArrayList<>(3);
                rotation.add(0.0);
                rotation.add(vd.getHeading() != null ? vd.getHeading() : 0.0);
                rotation.add(0.0);

                boolean ok;
                if (!currentActorIds.contains(id)) {
                    // Spawn a basic vehicle actor if missing
                    log.info("Attempting to spawn CARLA actor for SUMO vehicle '{}' at ({}, {}) yaw {}", id, location.get(0), location.get(1), rotation.get(1));
                    String blueprint = carlaConfig != null && StringUtils.isNotBlank(carlaConfig.defaultVehicleBlueprint)
                            ? carlaConfig.defaultVehicleBlueprint
                            : "vehicle.tesla.model3";
                    // Attach SUMO vehicle extent to attributes if available (so server can correct front-bumper reference)
                    java.util.Map<String, Object> attributes = new java.util.HashMap<>();
                    try {
                        Object extra = vd.getAdditionalData();
                        // Prefer structured Size additional data
                        if (extra instanceof org.eclipse.mosaic.lib.objects.detector.Size) {
                            org.eclipse.mosaic.lib.objects.detector.Size sz = (org.eclipse.mosaic.lib.objects.detector.Size) extra;
                            double length = sz.getLength();
                            double width = sz.getWidth();
                            double height = sz.getHeight();
                            java.util.Map<String, Object> extent = new java.util.HashMap<>();
                            extent.put("x", length / 2.0);
                            extent.put("y", width / 2.0);
                            extent.put("z", height / 2.0);
                            attributes.put("extent", extent);
                            attributes.put("length", length);
                        } else if (extra instanceof java.util.Map) {
                            @SuppressWarnings("rawtypes")
                            java.util.Map m = (java.util.Map) extra;
                            Object l = m.get("length");
                            Object w = m.get("width");
                            Object h = m.get("height");
                            if (l instanceof Number || w instanceof Number || h instanceof Number) {
                                double length = l instanceof Number ? ((Number) l).doubleValue() : 0.0;
                                double width = w instanceof Number ? ((Number) w).doubleValue() : 0.0;
                                double height = h instanceof Number ? ((Number) h).doubleValue() : 0.0;
                                java.util.Map<String, Object> extent = new java.util.HashMap<>();
                                extent.put("x", length / 2.0);
                                extent.put("y", width / 2.0);
                                extent.put("z", height / 2.0);
                                attributes.put("extent", extent);
                                if (length > 0.0) {
                                    attributes.put("length", length);
                                }
                            }
                        }
                    } catch (Exception ignore) {
                        // Best-effort; attributes remain empty if no size info
                    }
                    // Apply a small Z-lift to reduce spawn collisions with ground
                    final double SPAWN_Z_LIFT = 2; // meters
                    if (location != null && location.size() >= 3) {
                        try {
                            double z = location.get(2) != null ? location.get(2) : 0.0;
                            location.set(2, z + SPAWN_Z_LIFT);
                        } catch (Exception ignore) { /* keep original if any issue */ }
                    }

                    if (multiXmlRpcManager != null) {
                        log.info("Using multi-XML-RPC manager to spawn actor (z+{} m)", SPAWN_Z_LIFT);
                        ok = multiXmlRpcManager.getClient(CarlaXmlRpcClient.ServerType.ACTOR_LIB).spawnActor(blueprint, id, location, rotation, attributes);
                    } else {
                        log.info("Using single XML-RPC client to spawn actor (z+{} m)", SPAWN_Z_LIFT);
                        ok = carlaXmlRpcClient.spawnActor(blueprint, id, location, rotation, attributes);
                    }
                    if (ok) {
                        log.info("Successfully spawned CARLA actor for SUMO vehicle '{}' at ({}, {}) yaw {}", id, location.get(0), location.get(1), rotation.get(1));
                        currentActorIds.add(id);
                    } else {
                        log.error("Failed to spawn CARLA actor for SUMO vehicle {} - XML-RPC call returned false", id);
                    }
                } else {
                    // Update transform
                    if (multiXmlRpcManager != null) {
                        ok = multiXmlRpcManager.getClient(CarlaXmlRpcClient.ServerType.ACTOR_LIB).updateActorTransform(id, location, rotation);
                    } else {
                        ok = carlaXmlRpcClient.updateActorTransform(id, location, rotation);
                    }
                    if (!ok) {
                        log.debug("Failed to update CARLA actor transform for {}", id);
                    }
                }
            };

            // Apply to added and updated vehicles
            for (org.eclipse.mosaic.lib.objects.vehicle.VehicleData v : interaction.getAdded()) {
                applyVehicle.accept(v);
            }
            for (org.eclipse.mosaic.lib.objects.vehicle.VehicleData v : interaction.getUpdated()) {
                applyVehicle.accept(v);
            }

            // Handle removals
            for (String removedId : interaction.getRemovedNames()) {
                if (currentActorIds.contains(removedId)) {
                    boolean ok;
                    if (multiXmlRpcManager != null) {
                        ok = multiXmlRpcManager.getClient(CarlaXmlRpcClient.ServerType.ACTOR_LIB).destroyActor(removedId);
                    } else {
                        ok = carlaXmlRpcClient.destroyActor(removedId);
                    }
                    if (ok) {
                        currentActorIds.remove(removedId);
                    } else {
                        log.debug("Failed to destroy CARLA actor {} for SUMO removal", removedId);
                    }
                }
            }
        } catch (Exception e) {
            log.warn("SUMO->CARLA vehicle synchronization failed: {}", e.getMessage());
        }
    }
    /**
     * Process the CARLA vehicles receiving V2X message interaction
     *
     * @param interaction CarlaV2xMessageReception interaction
     */
    private void receiveInteraction(CarlaV2xMessageReception interaction) {
        log.info("{} received V2x message: {}.", interaction.getReceiverID(), interaction.getMessage());

        carlaV2xInteractionQueue.add(interaction);
    }

    /**
     * Process traffic light state change commands and forward them to CARLA.
     * This enables other federates (like applications or SUMO) to control CARLA traffic lights.
     *
     * @param interaction TrafficLightStateChange interaction
     */
    private void receiveInteraction(TrafficLightStateChange interaction) {
        log.info("Received TrafficLightStateChange for traffic light group '{}' with parameter type: {}", 
                interaction.getTrafficLightGroupId(), interaction.getParameterType());
        
        boolean actorConnected = false;
        if (multiXmlRpcManager != null) {
            actorConnected = multiXmlRpcManager.isConnected(CarlaXmlRpcClient.ServerType.ACTOR_LIB);
        } else if (carlaXmlRpcClient != null && carlaXmlRpcClient.getServerType() == CarlaXmlRpcClient.ServerType.ACTOR_LIB) {
            actorConnected = carlaXmlRpcClient.isConnected();
        }
        
        if (!actorConnected) {
            log.warn("Actor server not connected; cannot forward traffic light state change to CARLA");
            return;
        }
        
        try {
            String trafficLightId = interaction.getTrafficLightGroupId();
            
            switch (interaction.getParameterType()) {
                case ChangePhase:
                    log.info("Changing traffic light '{}' to phase index: {}", trafficLightId, interaction.getPhaseIndex());
                    if (multiXmlRpcManager != null) {
                        multiXmlRpcManager.getClient(CarlaXmlRpcClient.ServerType.ACTOR_LIB).setTrafficLightState(trafficLightId, "phase_" + interaction.getPhaseIndex());
                    } else {
                        carlaXmlRpcClient.setTrafficLightState(trafficLightId, "phase_" + interaction.getPhaseIndex());
                    }
                    break;
                    
                case RemainingDuration:
                    double durationInSeconds = interaction.getPhaseRemainingDuration() / 1000.0; // ms -> s
                    log.info("Setting traffic light '{}' remaining duration to: {} seconds", trafficLightId, durationInSeconds);
                    if (multiXmlRpcManager != null) {
                        multiXmlRpcManager.getClient(CarlaXmlRpcClient.ServerType.ACTOR_LIB).setTrafficLightTimer(trafficLightId, durationInSeconds);
                    } else {
                        carlaXmlRpcClient.setTrafficLightTimer(trafficLightId, durationInSeconds);
                    }
                    break;
                    
                case ProgramId:
                    log.info("Changing traffic light '{}' to program: {}", trafficLightId, interaction.getProgramId());
                    if (multiXmlRpcManager != null) {
                        multiXmlRpcManager.getClient(CarlaXmlRpcClient.ServerType.ACTOR_LIB).setTrafficLightState(trafficLightId, interaction.getProgramId());
                    } else {
                        carlaXmlRpcClient.setTrafficLightState(trafficLightId, interaction.getProgramId());
                    }
                    break;
                    
                case ChangeProgramWithPhase:
                    log.info("Changing traffic light '{}' to program '{}' with phase: {}", 
                            trafficLightId, interaction.getProgramId(), interaction.getPhaseIndex());
                    if (multiXmlRpcManager != null) {
                        multiXmlRpcManager.getClient(CarlaXmlRpcClient.ServerType.ACTOR_LIB).setTrafficLightState(trafficLightId, interaction.getProgramId() + "_phase_" + interaction.getPhaseIndex());
                    } else {
                        carlaXmlRpcClient.setTrafficLightState(trafficLightId, interaction.getProgramId() + "_phase_" + interaction.getPhaseIndex());
                    }
                    break;
                    
                case ChangeToCustomState:
                    log.info("Setting traffic light '{}' to custom state", trafficLightId);
                    // For custom states, we'll use a generic "custom" state
                    if (multiXmlRpcManager != null) {
                        multiXmlRpcManager.getClient(CarlaXmlRpcClient.ServerType.ACTOR_LIB).setTrafficLightState(trafficLightId, "custom");
                    } else {
                        carlaXmlRpcClient.setTrafficLightState(trafficLightId, "custom");
                    }
                    break;
                    
                default:
                    log.warn("Unknown traffic light state change parameter type: {}", interaction.getParameterType());
                    break;
            }
        } catch (Exception e) {
            log.error("Failed to forward traffic light state change to CARLA: {}", e.getMessage());
        }
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
            log.error("error occurs during sending messages to bridge: {}", e.getMessage());
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
