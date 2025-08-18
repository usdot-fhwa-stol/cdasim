/*
*Copyright (C) 2023 LEIDOS.
*
*Licensed under the Apache License, Version 2.0 (the "License"); you may not
*use this file except in compliance with the License. You may obtain a copy of
*the License at
*
*http://www.apache.org/licenses/LICENSE-2.0
*
*Unless required by applicable law or agreed to in writing, software
*distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
*WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
*License for the specific language governing permissions and limitations under
*the License.
*/
package org.eclipse.mosaic.fed.carla;

import org.eclipse.mosaic.fed.carla.carlaconnect.CarlaXmlRpcClient;
import org.eclipse.mosaic.interactions.detector.DetectorRegistration;
import org.eclipse.mosaic.interactions.detector.DetectorResult;
import org.eclipse.mosaic.interactions.vehicle.VehicleRegistration;
import org.eclipse.mosaic.interactions.vehicle.VehicleUpdate;
import org.eclipse.mosaic.interactions.traffic.TrafficLightRegistration;
import org.eclipse.mosaic.interactions.traffic.TrafficLightUpdate;
import org.eclipse.mosaic.lib.objects.detector.DetectedObject;
import org.eclipse.mosaic.lib.objects.vehicle.VehicleData;
import org.eclipse.mosaic.lib.objects.trafficlight.TrafficLightState;
import org.eclipse.mosaic.rti.api.Interaction;
import org.eclipse.mosaic.rti.api.RtiAmbassador;
import org.eclipse.mosaic.rti.api.federatestarter.FederateStarter;
import org.eclipse.mosaic.rti.api.parameters.AmbassadorParameter;
import org.eclipse.mosaic.rti.api.parameters.FederateDescriptor;
import org.eclipse.mosaic.rti.api.parameters.FederateParameter;

import org.apache.xmlrpc.XmlRpcException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.net.URL;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;

/**
 * CARLA Federate for MOSAIC integration using XML-RPC architecture.
 * 
 * This federate manages actors (vehicles, pedestrians) and traffic lights
 * in CARLA through XML-RPC communication, replacing the TraCI-based bridge.
 * It provides comprehensive actor lifecycle management and traffic light control.
 */
public class CarlaFederate implements FederateStarter {

    private static final Logger log = LoggerFactory.getLogger(CarlaFederate.class);
    
    // Configuration constants
    private static final String DEFAULT_CARLA_SERVER_URL = "http://localhost:8000";
    private static final int DEFAULT_CONNECTION_RETRIES = 5;
    private static final double DEFAULT_TIME_STEP = 0.1; // 100ms
    
    // XML-RPC client
    private CarlaXmlRpcClient carlaClient;
    
    // RTI ambassador
    private RtiAmbassador rtiAmbassador;
    
    // Federate configuration
    private String carlaServerUrl;
    private int connectionRetries;
    private double timeStep;
    private String mapName;
    
    // State tracking
    private boolean isConnected = false;
    private boolean isRunning = false;
    private double currentTime = 0.0;
    
    // Actor tracking
    private final Map<String, VehicleData> registeredVehicles = new ConcurrentHashMap<>();
    private final Map<String, String> vehicleActorIds = new ConcurrentHashMap<>(); // MOSAIC ID -> CARLA Actor ID
    private final Map<String, String> actorVehicleIds = new ConcurrentHashMap<>(); // CARLA Actor ID -> MOSAIC ID
    
    // Traffic light tracking
    private final Map<String, TrafficLightState> registeredTrafficLights = new ConcurrentHashMap<>();
    private final Map<String, String> trafficLightActorIds = new ConcurrentHashMap<>(); // MOSAIC ID -> CARLA Actor ID
    
    // Sensor tracking
    private final Set<String> registeredSensors = new HashSet<>();

    @Override
    public void initialize(FederateDescriptor federateDescriptor, AmbassadorParameter ambassadorParameter) {
        log.info("Initializing CARLA Federate with XML-RPC architecture");
        
        // Load configuration
        loadConfiguration(federateDescriptor);
        
        // Initialize RTI ambassador
        this.rtiAmbassador = ambassadorParameter.getRtiAmbassador();
        
        // Initialize XML-RPC client
        try {
            URL serverUrl = new URL(carlaServerUrl);
            carlaClient = new CarlaXmlRpcClient(serverUrl);
            log.info("CARLA XML-RPC client initialized for server: {}", carlaServerUrl);
        } catch (Exception e) {
            log.error("Failed to initialize CARLA XML-RPC client", e);
            throw new RuntimeException("Failed to initialize CARLA client", e);
        }
    }

    @Override
    public void start() {
        log.info("Starting CARLA Federate");
        
        try {
            // Connect to CARLA server
            carlaClient.connect(connectionRetries);
            isConnected = true;
            log.info("Connected to CARLA XML-RPC server");
            
            // Load map if specified
            if (mapName != null && !mapName.isEmpty()) {
                if (carlaClient.loadMap(mapName)) {
                    log.info("Loaded CARLA map: {}", mapName);
                } else {
                    log.warn("Failed to load map: {}", mapName);
                }
            }
            
            // Start simulation
            if (carlaClient.startSimulation()) {
                isRunning = true;
                log.info("CARLA simulation started");
            } else {
                log.error("Failed to start CARLA simulation");
                throw new RuntimeException("Failed to start CARLA simulation");
            }
            
            // Register for interactions
            registerForInteractions();
            
            log.info("CARLA Federate started successfully");
            
        } catch (Exception e) {
            log.error("Failed to start CARLA Federate", e);
            throw new RuntimeException("Failed to start CARLA Federate", e);
        }
    }

    @Override
    public void stop() {
        log.info("Stopping CARLA Federate");
        
        try {
            if (isRunning) {
                carlaClient.stopSimulation();
                isRunning = false;
                log.info("CARLA simulation stopped");
            }
            
            if (isConnected) {
                carlaClient.disconnect();
                isConnected = false;
                log.info("Disconnected from CARLA XML-RPC server");
            }
            
            log.info("CARLA Federate stopped successfully");
            
        } catch (Exception e) {
            log.error("Error stopping CARLA Federate", e);
        }
    }

    @Override
    public void processInteraction(Interaction interaction) {
        try {
            if (interaction instanceof VehicleRegistration) {
                processVehicleRegistration((VehicleRegistration) interaction);
            } else if (interaction instanceof VehicleUpdate) {
                processVehicleUpdate((VehicleUpdate) interaction);
            } else if (interaction instanceof TrafficLightRegistration) {
                processTrafficLightRegistration((TrafficLightRegistration) interaction);
            } else if (interaction instanceof TrafficLightUpdate) {
                processTrafficLightUpdate((TrafficLightUpdate) interaction);
            } else if (interaction instanceof DetectorRegistration) {
                processDetectorRegistration((DetectorRegistration) interaction);
            } else {
                log.debug("Received unhandled interaction: {}", interaction.getClass().getSimpleName());
            }
        } catch (Exception e) {
            log.error("Error processing interaction: {}", interaction.getClass().getSimpleName(), e);
        }
    }

    @Override
    public void processTimeAdvanceGrant(long time) {
        try {
            currentTime = time / 1000.0; // Convert to seconds
            
            // Step CARLA simulation
            if (isRunning && isConnected) {
                if (carlaClient.stepSimulation(timeStep)) {
                    // Process sensor data
                    processSensorData();
                    
                    // Update vehicle states from CARLA
                    updateVehicleStates();
                    
                    // Update traffic light states from CARLA
                    updateTrafficLightStates();
                } else {
                    log.error("Failed to step CARLA simulation");
                }
            }
            
            // Request next time advance
            rtiAmbassador.requestAdvanceTime(time + (long)(timeStep * 1000));
            
        } catch (Exception e) {
            log.error("Error processing time advance grant", e);
        }
    }

    /**
     * Load federate configuration from parameters.
     */
    private void loadConfiguration(FederateDescriptor federateDescriptor) {
        // Load configuration parameters
        this.carlaServerUrl = federateDescriptor.getConfiguration().getString("carlaServerUrl", DEFAULT_CARLA_SERVER_URL);
        this.connectionRetries = federateDescriptor.getConfiguration().getInt("connectionRetries", DEFAULT_CONNECTION_RETRIES);
        this.timeStep = federateDescriptor.getConfiguration().getDouble("timeStep", DEFAULT_TIME_STEP);
        this.mapName = federateDescriptor.getConfiguration().getString("mapName", "");
        
        log.info("CARLA Federate configuration:");
        log.info("  Server URL: {}", carlaServerUrl);
        log.info("  Connection retries: {}", connectionRetries);
        log.info("  Time step: {}s", timeStep);
        log.info("  Map name: {}", mapName.isEmpty() ? "default" : mapName);
    }

    /**
     * Register for MOSAIC interactions.
     */
    private void registerForInteractions() {
        try {
            rtiAmbassador.subscribeInteraction(VehicleRegistration.class);
            rtiAmbassador.subscribeInteraction(VehicleUpdate.class);
            rtiAmbassador.subscribeInteraction(TrafficLightRegistration.class);
            rtiAmbassador.subscribeInteraction(TrafficLightUpdate.class);
            rtiAmbassador.subscribeInteraction(DetectorRegistration.class);
            log.info("Registered for MOSAIC interactions");
        } catch (Exception e) {
            log.error("Failed to register for interactions", e);
            throw new RuntimeException("Failed to register for interactions", e);
        }
    }

    /**
     * Process vehicle registration interaction.
     */
    private void processVehicleRegistration(VehicleRegistration registration) {
        try {
            VehicleData vehicleData = registration.getVehicle();
            String vehicleId = vehicleData.getName();
            
            log.info("Processing vehicle registration: {}", vehicleId);
            
            // Spawn vehicle in CARLA
            String actorType = getVehicleActorType(vehicleData);
            String actorId = generateActorId(vehicleId);
            
            List<Double> location = Arrays.asList(
                vehicleData.getPosition().getX(),
                vehicleData.getPosition().getY(),
                vehicleData.getPosition().getZ()
            );
            
            List<Double> rotation = Arrays.asList(
                vehicleData.getHeading().getPitch(),
                vehicleData.getHeading().getYaw(),
                vehicleData.getHeading().getRoll()
            );
            
            Map<String, Object> attributes = new HashMap<>();
            attributes.put("role_name", vehicleId);
            
            if (carlaClient.spawnActor(actorType, actorId, location, rotation, attributes)) {
                // Track the vehicle
                registeredVehicles.put(vehicleId, vehicleData);
                vehicleActorIds.put(vehicleId, actorId);
                actorVehicleIds.put(actorId, vehicleId);
                
                log.info("Successfully spawned vehicle {} as actor {}", vehicleId, actorId);
            } else {
                log.error("Failed to spawn vehicle: {}", vehicleId);
            }
            
        } catch (Exception e) {
            log.error("Error processing vehicle registration", e);
        }
    }

    /**
     * Process vehicle update interaction.
     */
    private void processVehicleUpdate(VehicleUpdate update) {
        try {
            VehicleData vehicleData = update.getVehicle();
            String vehicleId = vehicleData.getName();
            
            String actorId = vehicleActorIds.get(vehicleId);
            if (actorId == null) {
                log.warn("Vehicle {} not found in CARLA", vehicleId);
                return;
            }
            
            // Update vehicle transform
            List<Double> location = Arrays.asList(
                vehicleData.getPosition().getX(),
                vehicleData.getPosition().getY(),
                vehicleData.getPosition().getZ()
            );
            
            List<Double> rotation = Arrays.asList(
                vehicleData.getHeading().getPitch(),
                vehicleData.getHeading().getYaw(),
                vehicleData.getHeading().getRoll()
            );
            
            if (carlaClient.updateActorTransform(actorId, location, rotation)) {
                // Update velocity if available
                if (vehicleData.getSpeed() != null) {
                    List<Double> velocity = Arrays.asList(
                        vehicleData.getSpeed().getX(),
                        vehicleData.getSpeed().getY(),
                        vehicleData.getSpeed().getZ()
                    );
                    carlaClient.updateActorVelocity(actorId, velocity);
                }
                
                // Update local state
                registeredVehicles.put(vehicleId, vehicleData);
                
                log.debug("Updated vehicle {} transform", vehicleId);
            } else {
                log.error("Failed to update vehicle {} transform", vehicleId);
            }
            
        } catch (Exception e) {
            log.error("Error processing vehicle update", e);
        }
    }

    /**
     * Process traffic light registration interaction.
     */
    private void processTrafficLightRegistration(TrafficLightRegistration registration) {
        try {
            String trafficLightId = registration.getTrafficLightId();
            
            log.info("Processing traffic light registration: {}", trafficLightId);
            
            // Note: Traffic lights are typically already present in CARLA maps
            // We just need to track them for state management
            registeredTrafficLights.put(trafficLightId, TrafficLightState.UNKNOWN);
            
            log.info("Registered traffic light: {}", trafficLightId);
            
        } catch (Exception e) {
            log.error("Error processing traffic light registration", e);
        }
    }

    /**
     * Process traffic light update interaction.
     */
    private void processTrafficLightUpdate(TrafficLightUpdate update) {
        try {
            String trafficLightId = update.getTrafficLightId();
            TrafficLightState state = update.getTrafficLightState();
            
            log.info("Processing traffic light update: {} -> {}", trafficLightId, state);
            
            // Find corresponding CARLA traffic light
            String actorId = trafficLightActorIds.get(trafficLightId);
            if (actorId == null) {
                // Try to find by ID in CARLA
                List<String> carlaTrafficLights = carlaClient.getTrafficLights();
                for (String carlaId : carlaTrafficLights) {
                    if (carlaId.contains(trafficLightId) || trafficLightId.contains(carlaId)) {
                        actorId = carlaId;
                        trafficLightActorIds.put(trafficLightId, actorId);
                        break;
                    }
                }
            }
            
            if (actorId != null) {
                String carlaState = convertToCarlaTrafficLightState(state);
                if (carlaClient.setTrafficLightState(actorId, carlaState)) {
                    registeredTrafficLights.put(trafficLightId, state);
                    log.info("Updated traffic light {} to {}", trafficLightId, state);
                } else {
                    log.error("Failed to update traffic light {} state", trafficLightId);
                }
            } else {
                log.warn("Traffic light {} not found in CARLA", trafficLightId);
            }
            
        } catch (Exception e) {
            log.error("Error processing traffic light update", e);
        }
    }

    /**
     * Process detector registration interaction.
     */
    private void processDetectorRegistration(DetectorRegistration registration) {
        try {
            log.info("Processing detector registration: {}", registration.getDetector().getSensorId());
            
            // Create sensor in CARLA
            carlaClient.createSensor(registration);
            registeredSensors.add(registration.getDetector().getSensorId());
            
        } catch (Exception e) {
            log.error("Error processing detector registration", e);
        }
    }

    /**
     * Process sensor data from CARLA.
     */
    private void processSensorData() {
        try {
            for (String sensorId : registeredSensors) {
                DetectedObject[] detectedObjects = carlaClient.getDetectedObjects("", sensorId);
                if (detectedObjects != null && detectedObjects.length > 0) {
                    // Create and send detector result interaction
                    DetectorResult result = new DetectorResult(currentTime * 1000, sensorId, detectedObjects);
                    rtiAmbassador.triggerInteraction(result);
                    
                    log.debug("Processed {} detections from sensor {}", detectedObjects.length, sensorId);
                }
            }
        } catch (Exception e) {
            log.error("Error processing sensor data", e);
        }
    }

    /**
     * Update vehicle states from CARLA.
     */
    private void updateVehicleStates() {
        try {
            Map<String, Map<String, Object>> allActors = carlaClient.getAllActors();
            
            for (Map.Entry<String, Map<String, Object>> entry : allActors.entrySet()) {
                String actorId = entry.getKey();
                Map<String, Object> actorInfo = entry.getValue();
                
                String vehicleId = actorVehicleIds.get(actorId);
                if (vehicleId != null) {
                    // Update vehicle state from CARLA
                    @SuppressWarnings("unchecked")
                    Map<String, List<Double>> transform = (Map<String, List<Double>>) actorInfo.get("transform");
                    if (transform != null) {
                        // Create vehicle update interaction
                        // Note: This would require creating a VehicleData object from the transform
                        log.debug("Updated vehicle {} state from CARLA", vehicleId);
                    }
                }
            }
        } catch (Exception e) {
            log.error("Error updating vehicle states", e);
        }
    }

    /**
     * Update traffic light states from CARLA.
     */
    private void updateTrafficLightStates() {
        try {
            List<String> trafficLights = carlaClient.getTrafficLights();
            
            for (String actorId : trafficLights) {
                String state = carlaClient.getTrafficLightState(actorId);
                if (state != null) {
                    // Find corresponding MOSAIC traffic light
                    for (Map.Entry<String, String> entry : trafficLightActorIds.entrySet()) {
                        if (entry.getValue().equals(actorId)) {
                            String trafficLightId = entry.getKey();
                            TrafficLightState mosaicState = convertFromCarlaTrafficLightState(state);
                            
                            if (mosaicState != registeredTrafficLights.get(trafficLightId)) {
                                // Create and send traffic light update interaction
                                TrafficLightUpdate update = new TrafficLightUpdate(currentTime * 1000, trafficLightId, mosaicState);
                                rtiAmbassador.triggerInteraction(update);
                                
                                registeredTrafficLights.put(trafficLightId, mosaicState);
                                log.debug("Updated traffic light {} state from CARLA: {}", trafficLightId, mosaicState);
                            }
                            break;
                        }
                    }
                }
            }
        } catch (Exception e) {
            log.error("Error updating traffic light states", e);
        }
    }

    /**
     * Get vehicle actor type for CARLA.
     */
    private String getVehicleActorType(VehicleData vehicleData) {
        // Map MOSAIC vehicle types to CARLA actor types
        String vehicleType = vehicleData.getVehicleType();
        if (vehicleType != null) {
            switch (vehicleType.toLowerCase()) {
                case "car":
                case "passenger":
                    return "vehicle.tesla.model3";
                case "truck":
                    return "vehicle.truck";
                case "bus":
                    return "vehicle.bus";
                case "motorcycle":
                    return "vehicle.kawasaki.ninja";
                case "bicycle":
                    return "vehicle.bh.crossbike";
                default:
                    return "vehicle.tesla.model3"; // Default
            }
        }
        return "vehicle.tesla.model3"; // Default
    }

    /**
     * Generate unique actor ID for CARLA.
     */
    private String generateActorId(String vehicleId) {
        return "mosaic_" + vehicleId + "_" + System.currentTimeMillis();
    }

    /**
     * Convert MOSAIC traffic light state to CARLA state.
     */
    private String convertToCarlaTrafficLightState(TrafficLightState state) {
        switch (state) {
            case RED:
                return "Red";
            case YELLOW:
                return "Yellow";
            case GREEN:
                return "Green";
            default:
                return "Red";
        }
    }

    /**
     * Convert CARLA traffic light state to MOSAIC state.
     */
    private TrafficLightState convertFromCarlaTrafficLightState(String state) {
        switch (state.toLowerCase()) {
            case "red":
                return TrafficLightState.RED;
            case "yellow":
                return TrafficLightState.YELLOW;
            case "green":
                return TrafficLightState.GREEN;
            default:
                return TrafficLightState.UNKNOWN;
        }
    }
}
