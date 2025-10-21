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
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.DocumentBuilder;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import java.io.FileInputStream;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.io.FileNotFoundException;
import java.io.IOException;

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
     * Snapshot of CARLA actor ids from previous tick to detect externally spawned actors.
     */
    private final Set<String> lastActorIds = new HashSet<>();

    /**
     * SUMO net offset parsed from scenario .net.xml (x, y) in meters.
     * Default to Town04 values if parsing fails.
     */
    private double[] sumoNetOffsetXY = new double[]{503.02, 423.76};


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

        // Initialize SUMO net offset
        try {
            String sumoNetXmlPath = carlaConfig.sumoNetXmlPath;
            if (StringUtils.isBlank(sumoNetXmlPath)) {
                log.error("Couldn't find .net.xml file under the directory: {}", sumoNetXmlPath);
            } else {
                log.info("Using net.xml path from: {}", sumoNetXmlPath);
            }
            
            double[] parsed = readSumoNetOffsetFromNetXml(sumoNetXmlPath);
            if (parsed != null) {
                sumoNetOffsetXY = parsed;
                log.info("SUMO netOffset successfully parsed from {}: x={}, y={}", sumoNetXmlPath, sumoNetOffsetXY[0], sumoNetOffsetXY[1]);
            } else {
                // Fallback to env
                sumoNetOffsetXY = readSumoNetOffsetFromEnv();
                log.info("SUMO netOffset via env or default: x={}, y={}", sumoNetOffsetXY[0], sumoNetOffsetXY[1]);
            }
        } catch (Exception ex) {
            log.warn("Failed to parse SUMO netOffset; using defaults: {}", ex.getMessage());
            sumoNetOffsetXY = readSumoNetOffsetFromEnv();
        }
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
                } else if (carlaXmlRpcClient != null) {
                    carlaXmlRpcClient.connect(60);
                }
            }
            // if the simulation step received from CARLA, advance CARLA federate local
            // simulation time
        
                
                // Handle sensor operations
                log.info("Simulation step");
                boolean sensorConnected = false;
                if (multiXmlRpcManager != null) {
                    sensorConnected = multiXmlRpcManager.isConnected(CarlaXmlRpcClient.ServerType.SENSOR_LIB);
                } else if (carlaXmlRpcClient != null && carlaXmlRpcClient.getServerType() == CarlaXmlRpcClient.ServerType.SENSOR_LIB) {
                    sensorConnected = carlaXmlRpcClient.isConnected();
                }
                
                if (sensorConnected) {
                    // Get sensor client once to avoid repeated calls
                    CarlaXmlRpcClient sensorClient = null;
                    if (multiXmlRpcManager != null) {
                        sensorClient = multiXmlRpcManager.getClient(CarlaXmlRpcClient.ServerType.SENSOR_LIB);
                    } else {
                        sensorClient = carlaXmlRpcClient;
                    }
                    
                    List<DetectedObjectInteraction> detectedObjectInteractions = new ArrayList<>();
                    // Get all detections from all currently registered detectors.
                    for (DetectorRegistration registration: registeredDetectors ) {
                        DetectedObject[] detections = sensorClient.getDetectedObjects(registration.getInfrastructureId(), registration.getDetector().getSensorId());
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
                    log.info("actor connected");
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
                        
                        log.info("EXTERNAL VEHICLE DETECTION: Detected changes - Added: {}, Updated: {}, Removed: {}", 
                                addedActors.size(), updatedActors.size(), removedActors.size());
                        
                        // Log detailed information about added actors
                        for (java.util.Map<String, Object> actorInfo : addedActors) {
                            String actorId = actorInfo.get("id") != null ? actorInfo.get("id").toString() : "unknown";
                            log.info("EXTERNAL VEHICLE ADDED: Actor ID={}, Info={}", actorId, actorInfo);
                        }
                        
                        // Log detailed information about updated actors
                        for (java.util.Map<String, Object> actorInfo : updatedActors) {
                            String actorId = actorInfo.get("id") != null ? actorInfo.get("id").toString() : "unknown";
                            log.info("EXTERNAL VEHICLE UPDATED: Actor ID={}, Info={}", actorId, actorInfo);
                        }
                        
                        // Log detailed information about removed actors
                        for (String removedId : removedActors) {
                            log.info("EXTERNAL VEHICLE REMOVED: Actor ID={}", removedId);
                        }
                        
                        // Convert to VehicleData objects
                        java.util.List<org.eclipse.mosaic.lib.objects.vehicle.VehicleData> addedVehicleData = new java.util.ArrayList<>();
                        java.util.List<org.eclipse.mosaic.lib.objects.vehicle.VehicleData> updatedVehicleData = new java.util.ArrayList<>();
                        
                        // Convert added actors
                        for (java.util.Map<String, Object> actorInfo : addedActors) {
                            String actorId = (String) actorInfo.get("id");
                            if (actorId != null) {
                                org.eclipse.mosaic.lib.objects.vehicle.VehicleData vehicleData = convertCarlaActorToVehicleData(actorId, actorInfo);
                                if (vehicleData != null) {
                                    addedVehicleData.add(vehicleData);
                                }
                            }
                        }
                        
                        // Convert updated actors
                        for (java.util.Map<String, Object> actorInfo : updatedActors) {
                            String actorId = (String) actorInfo.get("id");
                            if (actorId != null) {
                                org.eclipse.mosaic.lib.objects.vehicle.VehicleData vehicleData = convertCarlaActorToVehicleData(actorId, actorInfo);
                                if (vehicleData != null) {
                                    updatedVehicleData.add(vehicleData);
                                }
                            }
                        }

                        // Update current actor IDs cache for next iteration
                        // Note: Removed the redundant full scan as getActorChanges() already handles this efficiently
                        java.util.Set<String> previousIds = new java.util.HashSet<>(currentActorIds);
                        currentActorIds.clear();
                        
                        // Add current actors from the change detection
                        for (java.util.Map<String, Object> actorInfo : addedActors) {
                            String actorId = (String) actorInfo.get("id");
                            if (actorId != null) {
                                currentActorIds.add(actorId);
                            }
                        }
                        for (java.util.Map<String, Object> actorInfo : updatedActors) {
                            String actorId = (String) actorInfo.get("id");
                            if (actorId != null) {
                                currentActorIds.add(actorId);
                            }
                        }
                        
                        // Remove actors that were removed
                        for (String removedId : removedActors) {
                            currentActorIds.remove(removedId);
                        }
                        
                        // Publish VehicleUpdates with converted VehicleData if there are changes
                        if (!addedVehicleData.isEmpty() || !updatedVehicleData.isEmpty() || !removedActors.isEmpty()) {
                            VehicleUpdates vehicleUpdates = new VehicleUpdates(time, addedVehicleData, updatedVehicleData, removedActors);
                            this.rti.triggerInteraction(vehicleUpdates);
                            log.info("CARLA->SUMO SYNC: Published VehicleUpdates to SUMO - added={}, updated={}, removed={}", 
                                addedVehicleData.size(), updatedVehicleData.size(), removedActors.size());

                        }

                        // Update last known actor id snapshot after publishing
                        lastActorIds.clear();
                        lastActorIds.addAll(currentActorIds);

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
                rti.requestAdvanceTime(nextTimeStep , 0, (byte) 2);
        
            
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

        // Disconnect from XML-RPC servers and cleanup resources
        if (multiXmlRpcManager != null) {
            multiXmlRpcManager.disconnectAll();
            // Note: multiXmlRpcManager cleanup would need to be implemented if needed
        } else if (carlaXmlRpcClient != null) {
            carlaXmlRpcClient.cleanup();
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

    // /**
    //  * Trigger interactions based on commands received from CARLA simulator.
    //  * Now uses XML-RPC for simulation advancement instead of TraCI.
    //  *
    //  * @param length  command length
    //  * @param command command
    //  */
    // public synchronized void triggerInteraction(int length, byte[] command) throws InternalFederateException {
    //     try {
    //         // Handle different command types using XML-RPC approach
    //         if (command[5] == CommandSimulationControl.COMMAND_SIMULATION_STEP) {
    //             // Use XML-RPC to advance simulation instead of TraCI-based SimulationStep
    //             boolean advanced = false;
    //             if (multiXmlRpcManager != null) {
    //                 advanced = multiXmlRpcManager.getClient(CarlaXmlRpcClient.ServerType.ACTOR_LIB).advanceSimulation();
    //             } else if (carlaXmlRpcClient != null) {
    //                 advanced = carlaXmlRpcClient.advanceSimulation();
    //             }
                
    //             if (advanced) {
    //                 // Trigger internal simulation step coordination
    //                 triggerInternalSimulationStep();
    //                 log.debug("CARLA simulation advanced via XML-RPC at time: {}", this.nextTimeStep);
    //             } else {
    //                 log.warn("Failed to advance CARLA simulation via XML-RPC");
    //             }
    //         } else if (command[5] == 0x0d) {
    //             // send received V2X message to CARLA simulator
    //             sendReceivedV2xMessageToCarla();
    //             // log.debug("Carla ambassador sends V2X messages to bridge client.");
    //         } else if (command[5] == 0x2f) {
    //             // receive message from CARLA simulator
    //             String[] message = processReceivedV2xMessageFromCarla(length, command);
    //             if (message != null) {
    //                 rti.triggerInteraction(new ExternalMessage(this.nextTimeStep, message[1], message[0]));
    //                 // log.debug("received message from CARLA simulator: message is sent by {};
    //                 // message: {}", message[0],
    //                 // message: {}", message[1]);
    //             }
    //         } else if (command[5] == 0x85) {
    //             log.info("Received vehicle add command from CARLA " + Hex.encodeHex(command));
    //         } else {
    //             log.debug("Ignoring legacy TraCI request path in favor of XML-RPC interactions");
    //         }

    //     } catch (IllegalValueException e) {
    //         throw new InternalFederateException(e);
    //     }
    // }

    /**
     * Internal method to trigger simulation step coordination.
     * This replaces the external SimulationStep interaction with internal logic.
     */
    private void triggerInternalSimulationStep() {
        // Set simulation step flag to trigger state updates and time advancement
        
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
        // Default to Town04 netOffset if not found in environment
        return new double[]{503.02, 423.76};
    }

    /**
     * Parse netOffset from a SUMO .net.xml file. Returns null if not found.
     */
    private double[] readSumoNetOffsetFromNetXml(String path) {
        if (StringUtils.isBlank(path)) {
            return null;
        }
        File f = new File(path);
        if (!f.exists() || !f.isFile()) {
            return null;
        }
        try (FileInputStream fis = new FileInputStream(f)) {
            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            dbFactory.setFeature("http://apache.org/xml/features/disallow-doctype-decl", true);
            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
            Document doc = dBuilder.parse(fis);
            doc.getDocumentElement().normalize();
            
            // Look for <location> element with netOffset attribute
            Element location = (Element) doc.getElementsByTagName("location").item(0);
            if (location != null && location.hasAttribute("netOffset")) {
                String val = location.getAttribute("netOffset");
                log.info("Found netOffset attribute: {}", val);
                String[] parts = val.split(",");
                if (parts.length >= 2) {
                    double x = Double.parseDouble(parts[0].trim());
                    double y = Double.parseDouble(parts[1].trim());
                    log.info("Parsed netOffset: x={}, y={}", x, y);
                    return new double[]{x, y};
                }
            } else {
                log.warn("No location element with netOffset found in {}", path);
            }
        } catch (Exception e) {
            log.warn("Failed to parse netOffset from {}: {}", path, e.getMessage());
        }
        return null;
    }

    /**
     * Convert SUMO projected position and heading to CARLA frame, applying netOffset and handedness.
     * This implementation follows the Python bridge_helper.py get_carla_transform logic.
     * Optionally adjust by extentX to convert front-bumper reference to vehicle center.
     * 
     * @param xSumo SUMO X coordinate
     * @param ySumo SUMO Y coordinate
     * @param headingDeg SUMO heading angle in degrees
     * @param extentX Vehicle extent in X direction (half length) for front-bumper to center conversion
     * @return Transform object with CARLA coordinates and heading
     */
    private Transform carlaTransformFromSumo(double xSumo, double ySumo, Double headingDeg, Double extentX) {
        // Start with SUMO coordinates
        double sumoX = xSumo;
        double sumoY = ySumo;
        double sumoZ = 0.0;
        
        // From front-center-bumper to center (sumo reference system)
        // Following Python logic: yaw = -1 * in_rotation.yaw + 90
        if (extentX != null && extentX > 0.0 && headingDeg != null) {
            // Fixed: Use consistent yaw calculation
            double yaw = -1 * headingDeg + 90; // Python: yaw = -1 * in_rotation.yaw + 90
            double yawRad = Math.toRadians(yaw);
            // Python: out_location = (in_location.x - math.cos(math.radians(yaw)) * extent.x,
            //                         in_location.y - math.sin(math.radians(yaw)) * extent.x,
            //                         in_location.z - math.sin(math.radians(pitch)) * extent.x)
            sumoX -= Math.cos(yawRad) * extentX;
            sumoY -= Math.sin(yawRad) * extentX;
            // Note: Python also considers pitch for Z, but we assume pitch=0 for simplicity
        }
        
        // Applying offset sumo-carla net
        // Python: out_location = (out_location[0] - offset[0], out_location[1] - offset[1], out_location[2])
        double xWithOffset = sumoX - sumoNetOffsetXY[0];
        double yWithOffset = sumoY - sumoNetOffsetXY[1];
        double zWithOffset = sumoZ;
        
        // Transform to carla reference system (left-handed)
        // Python: carla.Location(out_location[0], -out_location[1], out_location[2])
        double carlaX = xWithOffset;
        double carlaY = -yWithOffset; // Flip Y for left-handed system
        double carlaZ = zWithOffset;
        
        // Convert SUMO heading to CARLA yaw
        // Fixed: Ensure consistent angle conversion
        double carlaYawDeg = headingDeg != null ? (headingDeg - 90.0) : 0.0;
        // Normalize yaw to [-180, 180] range for CARLA
        while (carlaYawDeg > 180.0) carlaYawDeg -= 360.0;
        while (carlaYawDeg < -180.0) carlaYawDeg += 360.0;
        
        double pitchDeg = 0.0;
        double rollDeg = 0.0;
        
        return new Transform(carlaX, carlaY, carlaZ, pitchDeg, carlaYawDeg, rollDeg);
    }

    /**
     * Convert CARLA actor information to VehicleData for SUMO synchronization.
     * This method extracts position, velocity, and other vehicle properties from CARLA actor data
     * and converts them to the SUMO coordinate system.
     * 
     * @param actorId CARLA actor ID
     * @param actorInfo CARLA actor information map
     * @return VehicleData object or null if conversion fails
     */
    private org.eclipse.mosaic.lib.objects.vehicle.VehicleData convertCarlaActorToVehicleData(String actorId, java.util.Map<String, Object> actorInfo) {
        try {
            // Extract transform information
            Object transformObj = actorInfo.get("transform");
            if (!(transformObj instanceof java.util.Map)) {
                log.warn("No transform information found for CARLA actor '{}'", actorId);
                return null;
            }
            
            @SuppressWarnings("unchecked")
            java.util.Map<String, Object> transform = (java.util.Map<String, Object>) transformObj;
            
            // Extract location
            Object locationObj = transform.get("location");
            if (!(locationObj instanceof java.util.List)) {
                log.warn("No location information found for CARLA actor '{}'", actorId);
                return null;
            }
            
            @SuppressWarnings("unchecked")
            java.util.List<Object> locationList = (java.util.List<Object>) locationObj;
            if (locationList.size() < 3) {
                log.warn("Insufficient location data for CARLA actor '{}'", actorId);
                return null;
            }
            
            double xCarla = ((Number) locationList.get(0)).doubleValue();
            double yCarla = ((Number) locationList.get(1)).doubleValue();
            double zCarla = ((Number) locationList.get(2)).doubleValue();
            
            // Extract rotation
            Object rotationObj = transform.get("rotation");
            double yawDeg = 0.0;
            if (rotationObj instanceof java.util.List) {
                @SuppressWarnings("unchecked")
                java.util.List<Object> rotationList = (java.util.List<Object>) rotationObj;
                if (rotationList.size() >= 2) {
                    yawDeg = ((Number) rotationList.get(1)).doubleValue(); // yaw is typically the second element
                }
            }
            
            // Derive extentX (half length) when available to compensate front-bumper vs center reference
            Double extentX = null;
            try {
                Object extentObj = actorInfo.get("extent");
                if (extentObj instanceof java.util.Map) {
                    @SuppressWarnings("rawtypes")
                    java.util.Map m = (java.util.Map) extentObj;
                    Object ex = m.get("x");
                    if (ex instanceof Number) {
                        extentX = ((Number) ex).doubleValue();
                    }
                }
                // Try nested bounding box: bounding_box.extent.x
                if (extentX == null) {
                    Object bbObj = actorInfo.get("bounding_box");
                    if (bbObj instanceof java.util.Map) {
                        @SuppressWarnings("rawtypes")
                        java.util.Map bb = (java.util.Map) bbObj;
                        Object bbExt = bb.get("extent");
                        if (bbExt instanceof java.util.Map) {
                            @SuppressWarnings("rawtypes")
                            java.util.Map m = (java.util.Map) bbExt;
                            Object ex = m.get("x");
                            if (ex instanceof Number) {
                                extentX = ((Number) ex).doubleValue();
                            }
                        }
                    }
                }
                // Sometimes raw length is provided (full length)
                if (extentX == null) {
                    Object lengthObj = actorInfo.get("length");
                    if (lengthObj instanceof Number) {
                        extentX = ((Number) lengthObj).doubleValue() / 2.0;
                    }
                }
                // Attributes bag may carry extent/length
                if (extentX == null) {
                    Object attrsObj = actorInfo.get("attributes");
                    if (attrsObj instanceof java.util.Map) {
                        @SuppressWarnings("rawtypes")
                        java.util.Map attrs = (java.util.Map) attrsObj;
                        Object ext = attrs.get("extent");
                        if (ext instanceof java.util.Map) {
                            @SuppressWarnings("rawtypes")
                            java.util.Map m = (java.util.Map) ext;
                            Object ex = m.get("x");
                            if (ex instanceof Number) {
                                extentX = ((Number) ex).doubleValue();
                            }
                        }
                        if (extentX == null) {
                            Object l = attrs.get("length");
                            if (l instanceof Number) {
                                extentX = ((Number) l).doubleValue() / 2.0;
                            }
                        }
                    }
                }
            } catch (Exception ignore) { }

            // Convert CARLA coordinates to SUMO coordinates
            Transform sumoTransform = sumoTransformFromCarla(xCarla, yCarla, zCarla, yawDeg, extentX);
            
            // Extract velocity information
            double speed = 0.0;
            Object velocityObj = actorInfo.get("velocity");
            if (velocityObj instanceof java.util.Map) {
                @SuppressWarnings("unchecked")
                java.util.Map<String, Object> velocityMap = (java.util.Map<String, Object>) velocityObj;
                Object linearVelObj = velocityMap.get("linear");
                if (linearVelObj instanceof java.util.List) {
                    @SuppressWarnings("unchecked")
                    java.util.List<Object> linearVelList = (java.util.List<Object>) linearVelObj;
                    if (linearVelList.size() >= 3) {
                        double vx = ((Number) linearVelList.get(0)).doubleValue();
                        double vy = ((Number) linearVelList.get(1)).doubleValue();
                        speed = Math.sqrt(vx * vx + vy * vy); // Calculate speed magnitude
                    }
                }
            }
            
            // Create CartesianPoint for projected position
            org.eclipse.mosaic.lib.geo.CartesianPoint projectedPosition = 
                org.eclipse.mosaic.lib.geo.CartesianPoint.xy(sumoTransform.x, sumoTransform.y);
            
            // Create VehicleData via local factory to avoid direct Builder usage at call site
            return createVehicleData(this.nextTimeStep, actorId, projectedPosition, speed, sumoTransform.yaw, "external_carla_route");
                
        } catch (Exception e) {
            log.warn("Failed to convert CARLA actor '{}' to VehicleData: {}", actorId, e.getMessage());
            return null;
        }
    }

    /**
     * Convert CARLA position and heading to SUMO frame, applying netOffset and handedness.
     * This implementation follows the Python bridge_helper.py get_sumo_transform logic.
     * Used for external CARLA vehicle synchronization to SUMO.
     * 
     * @param xCarla CARLA X coordinate
     * @param yCarla CARLA Y coordinate  
     * @param zCarla CARLA Z coordinate
     * @param yawDeg CARLA yaw angle in degrees
     * @param extentX Vehicle extent in X direction (half length) for front-bumper to center conversion
     * @return Transform object with SUMO coordinates and heading
     */
    private Transform sumoTransformFromCarla(double xCarla, double yCarla, double zCarla, Double yawDeg, Double extentX) {
        // Start with CARLA coordinates
        double carlaX = xCarla;
        double carlaY = yCarla;
        double carlaZ = zCarla;
        
        // From center to front-center-bumper (carla reference system)
        // Fixed: Use consistent yaw calculation with carlaTransformFromSumo
        if (extentX != null && extentX > 0.0 && yawDeg != null) {
            // Use the same yaw calculation as in carlaTransformFromSumo for consistency
            double yaw = -1 * yawDeg + 90; // Consistent with carlaTransformFromSumo
            double yawRad = Math.toRadians(yaw);

            carlaX += Math.cos(yawRad) * extentX;
            carlaY -= Math.sin(yawRad) * extentX;
            // Note: Python also considers pitch for Z, but we assume pitch=0 for simplicity
        }
        
        // Applying offset carla-sumo net
        // Python: out_location = (out_location[0] + offset[0], out_location[1] - offset[1], out_location[2])
        double xWithOffset = carlaX + sumoNetOffsetXY[0];
        double yWithOffset = carlaY - sumoNetOffsetXY[1];
        double zWithOffset = carlaZ;
        

        double sumoX = xWithOffset;
        double sumoY = -yWithOffset; // Flip Y for right-handed system
        double sumoZ = zWithOffset;
        
        double sumoHeadingDeg = yawDeg != null ? (yawDeg + 90.0) : 0.0;
        
        // Normalize heading to [0, 360) range
        while (sumoHeadingDeg < 0) {
            sumoHeadingDeg += 360.0;
        }
        while (sumoHeadingDeg >= 360.0) {
            sumoHeadingDeg -= 360.0;
        }
        
        return new Transform(sumoX, sumoY, sumoZ, 0.0, sumoHeadingDeg, 0.0);
    }


    /** Simple struct for passing transforms */
    private static class Transform {
        final double x; final double y; final double z; final double pitch; final double yaw; final double roll;
        Transform(double x, double y, double z, double pitch, double yaw, double roll) {
            this.x = x; this.y = y; this.z = z; this.pitch = pitch; this.yaw = yaw; this.roll = roll;
        }
        List<Double> toLocationList() {
            List<Double> l = new ArrayList<>(3);
            l.add(x); l.add(y); l.add(z);
            return l;
        }
        List<Double> toRotationList() {
            List<Double> r = new ArrayList<>(3);
            r.add(pitch); r.add(yaw); r.add(roll);
            return r;
        }
    }

    /**
     * Local helper to create VehicleData without exposing Builder at call sites.
     */
    private org.eclipse.mosaic.lib.objects.vehicle.VehicleData createVehicleData(
            long timestampNs,
            String vehicleId,
            org.eclipse.mosaic.lib.geo.CartesianPoint projectedPosition,
            double speed,
            Double headingDeg,
            String routeId) {
        return new org.eclipse.mosaic.lib.objects.vehicle.VehicleData.Builder(timestampNs, vehicleId)
                .position(null, projectedPosition)
                .movement(speed, 0.0, 0.0)
                .orientation(org.eclipse.mosaic.lib.enums.DriveDirection.UNAVAILABLE, headingDeg, 0.0)
                .route(routeId)
                .create();
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
            // Get actor client once to avoid repeated calls
            CarlaXmlRpcClient actorClient = null;
            if (multiXmlRpcManager != null) {
                actorClient = multiXmlRpcManager.getClient(CarlaXmlRpcClient.ServerType.ACTOR_LIB);
            } else {
                actorClient = carlaXmlRpcClient;
            }
            
            // Log incoming sync request counts
            int numAdded = interaction.getAdded() != null ? interaction.getAdded().size() : 0;
            int numUpdated = interaction.getUpdated() != null ? interaction.getUpdated().size() : 0;
            int numRemoved = interaction.getRemovedNames() != null ? interaction.getRemovedNames().size() : 0;
            log.info("Starting SUMO->CARLA vehicle sync: added={}, updated={}, removed={}", numAdded, numUpdated, numRemoved);

            // Ensure we have up-to-date list of CARLA actors
            java.util.Map<String, java.util.Map<String, Object>> actors = actorClient.getAllActors();
            currentActorIds.clear();
            currentActorIds.addAll(actors.keySet());

            // Create a local copy for lambda use
            final java.util.Set<String> localCurrentActorIds = new java.util.HashSet<>(currentActorIds);
            
            // Track actors that need to be spawned
            final java.util.List<org.eclipse.mosaic.lib.objects.vehicle.VehicleData> actorsToSpawn = new java.util.ArrayList<>();
            final java.util.List<org.eclipse.mosaic.lib.objects.vehicle.VehicleData> actorsToUpdate = new java.util.ArrayList<>();

            // Helper to categorize vehicles
            java.util.function.Consumer<org.eclipse.mosaic.lib.objects.vehicle.VehicleData> categorizeVehicle = vd -> {
                final String id = vd.getName();
                final double xSumo = vd.getProjectedPosition() != null ? vd.getProjectedPosition().getX() : 0.0;
                final double ySumo = vd.getProjectedPosition() != null ? vd.getProjectedPosition().getY() : 0.0;
                final Double heading = vd.getHeading() != null ? vd.getHeading() : 0.0;
                final double speed = vd.getSpeed(); // Get speed from VehicleData
                
                // Determine extentX (half length) to convert front-bumper reference to vehicle center if available
                Double extentX = null;
                try {
                    Object extra = vd.getAdditionalData();
                    if (extra instanceof org.eclipse.mosaic.lib.objects.detector.Size) {
                        org.eclipse.mosaic.lib.objects.detector.Size sz = (org.eclipse.mosaic.lib.objects.detector.Size) extra;
                        extentX = sz.getLength() / 2.0;
                    } else if (extra instanceof java.util.Map) {
                        @SuppressWarnings("rawtypes")
                        java.util.Map m = (java.util.Map) extra;
                        Object l = m.get("length");
                        if (l instanceof Number) {
                            extentX = ((Number) l).doubleValue() / 2.0;
                        }
                    }
                } catch (Exception ignore) { }
                
                final Transform tf = carlaTransformFromSumo(xSumo, ySumo, heading, extentX);
                final java.util.List<Double> location = tf.toLocationList();
                final java.util.List<Double> rotation = tf.toRotationList();
                
                if (!localCurrentActorIds.contains(id)) {
                    // Add to spawn list
                    actorsToSpawn.add(vd);
                } else {
                    // Add to update list
                    actorsToUpdate.add(vd);
                }
            };

            // Apply to added and updated vehicles
            for (org.eclipse.mosaic.lib.objects.vehicle.VehicleData v : interaction.getAdded()) {
                categorizeVehicle.accept(v);
            }
            for (org.eclipse.mosaic.lib.objects.vehicle.VehicleData v : interaction.getUpdated()) {
                categorizeVehicle.accept(v);
            }

            // Process actors to spawn
            final java.util.Set<String> newlySpawnedActors = new java.util.HashSet<>();
            for (org.eclipse.mosaic.lib.objects.vehicle.VehicleData vd : actorsToSpawn) {
                final String id = vd.getName();
                final double xSumo = vd.getProjectedPosition() != null ? vd.getProjectedPosition().getX() : 0.0;
                final double ySumo = vd.getProjectedPosition() != null ? vd.getProjectedPosition().getY() : 0.0;
                final Double heading = vd.getHeading() != null ? vd.getHeading() : 0.0;
                final double speed = vd.getSpeed();
                
                // Determine extentX (half length) to convert front-bumper reference to vehicle center if available
                Double extentX = null;
                try {
                    Object extra = vd.getAdditionalData();
                    if (extra instanceof org.eclipse.mosaic.lib.objects.detector.Size) {
                        org.eclipse.mosaic.lib.objects.detector.Size sz = (org.eclipse.mosaic.lib.objects.detector.Size) extra;
                        extentX = sz.getLength() / 2.0;
                    } else if (extra instanceof java.util.Map) {
                        @SuppressWarnings("rawtypes")
                        java.util.Map m = (java.util.Map) extra;
                        Object l = m.get("length");
                        if (l instanceof Number) {
                            extentX = ((Number) l).doubleValue() / 2.0;
                        }
                    }
                } catch (Exception ignore) { }
                
                final Transform tf = carlaTransformFromSumo(xSumo, ySumo, heading, extentX);
                final java.util.List<Double> location = tf.toLocationList();
                final java.util.List<Double> rotation = tf.toRotationList();
                
                // Spawn a basic vehicle actor if missing
                log.info("Attempting to spawn CARLA actor for SUMO vehicle '{}' at ({}, {}) yaw {} speed {}", 
                        id, location.get(0), location.get(1), rotation.get(1), speed);
                final String blueprint = carlaConfig != null && StringUtils.isNotBlank(carlaConfig.defaultVehicleBlueprint)
                        ? carlaConfig.defaultVehicleBlueprint
                        : "vehicle.tesla.model3";
                
                // Attach SUMO vehicle extent to attributes if available (so server can correct front-bumper reference)
                final java.util.Map<String, Object> attributes = new java.util.HashMap<>();
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
                
                // Apply a small Z-lift to reduce spawn collisions with ground (client/server do no conversion)
                final double SPAWN_Z_LIFT = 2; // meters
                final java.util.List<Double> finalLocation = new java.util.ArrayList<>(location);
                if (finalLocation != null && finalLocation.size() >= 3) {
                    try {
                        double z = finalLocation.get(2) != null ? finalLocation.get(2) : 0.0;
                        finalLocation.set(2, z + SPAWN_Z_LIFT);
                    } catch (Exception ignore) { /* keep original if any issue */ }
                }

                log.info("Spawning actor (z+{} m)", SPAWN_Z_LIFT);
                final boolean ok = actorClient.spawnActor(blueprint, id, finalLocation, rotation, attributes);
                if (ok) {
                    log.info("Successfully spawned CARLA actor for SUMO vehicle '{}' at ({}, {}) yaw {} speed {}", 
                            id, finalLocation.get(0), finalLocation.get(1), rotation.get(1), speed);
                    newlySpawnedActors.add(id);
                } else {
                    log.error("Failed to spawn CARLA actor for SUMO vehicle {} - XML-RPC call returned false", id);
                }
            }

            // Process actors to update
            for (org.eclipse.mosaic.lib.objects.vehicle.VehicleData vd : actorsToUpdate) {
                final String id = vd.getName();
                final double xSumo = vd.getProjectedPosition() != null ? vd.getProjectedPosition().getX() : 0.0;
                final double ySumo = vd.getProjectedPosition() != null ? vd.getProjectedPosition().getY() : 0.0;
                final Double heading = vd.getHeading() != null ? vd.getHeading() : 0.0;
                final double speed = vd.getSpeed();
                
                // Determine extentX (half length) to convert front-bumper reference to vehicle center if available
                Double extentX = null;
                try {
                    Object extra = vd.getAdditionalData();
                    if (extra instanceof org.eclipse.mosaic.lib.objects.detector.Size) {
                        org.eclipse.mosaic.lib.objects.detector.Size sz = (org.eclipse.mosaic.lib.objects.detector.Size) extra;
                        extentX = sz.getLength() / 2.0;
                    } else if (extra instanceof java.util.Map) {
                        @SuppressWarnings("rawtypes")
                        java.util.Map m = (java.util.Map) extra;
                        Object l = m.get("length");
                        if (l instanceof Number) {
                            extentX = ((Number) l).doubleValue() / 2.0;
                        }
                    }
                } catch (Exception ignore) { }
                
                final Transform tf = carlaTransformFromSumo(xSumo, ySumo, heading, extentX);
                final java.util.List<Double> location = tf.toLocationList();
                final java.util.List<Double> rotation = tf.toRotationList();
                
                // Update transform and velocity for existing actors
                final boolean transformOk = actorClient.updateActorTransform(id, location, rotation);
                
                if (!transformOk) {
                    log.debug("Failed to update CARLA actor transform for {}", id);
                }
                
                if (transformOk) {
                    log.debug("Successfully updated CARLA actor '{}' transform (speed: {} m/s)", id, speed);
                }
            }

            // Update currentActorIds with newly spawned actors
            currentActorIds.addAll(newlySpawnedActors);

            // Handle removals
            for (String removedId : interaction.getRemovedNames()) {
                if (currentActorIds.contains(removedId)) {
                    boolean ok = actorClient.destroyActor(removedId);
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
            // Get actor client once to avoid repeated calls
            CarlaXmlRpcClient actorClient = null;
            if (multiXmlRpcManager != null) {
                actorClient = multiXmlRpcManager.getClient(CarlaXmlRpcClient.ServerType.ACTOR_LIB);
            } else {
                actorClient = carlaXmlRpcClient;
            }
            
            String trafficLightId = interaction.getTrafficLightGroupId();
            
            switch (interaction.getParameterType()) {
                case ChangePhase:
                    log.info("Changing traffic light '{}' to phase index: {}", trafficLightId, interaction.getPhaseIndex());
                    actorClient.setTrafficLightState(trafficLightId, "phase_" + interaction.getPhaseIndex());
                    break;
                    
                case RemainingDuration:
                    double durationInSeconds = interaction.getPhaseRemainingDuration() / 1000.0; // ms -> s
                    log.info("Setting traffic light '{}' remaining duration to: {} seconds", trafficLightId, durationInSeconds);
                    actorClient.setTrafficLightTimer(trafficLightId, durationInSeconds);
                    break;
                    
                case ProgramId:
                    log.info("Changing traffic light '{}' to program: {}", trafficLightId, interaction.getProgramId());
                    actorClient.setTrafficLightState(trafficLightId, interaction.getProgramId());
                    break;
                    
                case ChangeProgramWithPhase:
                    log.info("Changing traffic light '{}' to program '{}' with phase: {}", 
                            trafficLightId, interaction.getProgramId(), interaction.getPhaseIndex());
                    actorClient.setTrafficLightState(trafficLightId, interaction.getProgramId() + "_phase_" + interaction.getPhaseIndex());
                    break;
                    
                case ChangeToCustomState:
                    log.info("Setting traffic light '{}' to custom state", trafficLightId);
                    // For custom states, we'll use a generic "custom" state
                    actorClient.setTrafficLightState(trafficLightId, "custom");
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
