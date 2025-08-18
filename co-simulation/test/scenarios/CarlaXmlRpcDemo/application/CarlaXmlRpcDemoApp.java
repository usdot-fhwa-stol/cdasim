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
package org.eclipse.mosaic.test.carla;

import org.eclipse.mosaic.interactions.vehicle.VehicleRegistration;
import org.eclipse.mosaic.interactions.vehicle.VehicleUpdate;
import org.eclipse.mosaic.interactions.traffic.TrafficLightRegistration;
import org.eclipse.mosaic.interactions.traffic.TrafficLightUpdate;
import org.eclipse.mosaic.interactions.detector.DetectorRegistration;
import org.eclipse.mosaic.interactions.detector.DetectorResult;
import org.eclipse.mosaic.lib.objects.vehicle.VehicleData;
import org.eclipse.mosaic.lib.objects.trafficlight.TrafficLightState;
import org.eclipse.mosaic.lib.objects.detector.DetectedObject;
import org.eclipse.mosaic.lib.geo.CartesianPoint;
import org.eclipse.mosaic.lib.geo.GeoPoint;
import org.eclipse.mosaic.lib.objects.road.IRoadPosition;
import org.eclipse.mosaic.lib.objects.road.SimpleRoadPosition;
import org.eclipse.mosaic.lib.objects.vehicle.VehicleType;
import org.eclipse.mosaic.lib.objects.detector.Detector;
import org.eclipse.mosaic.lib.objects.detector.DetectorType;
import org.eclipse.mosaic.rti.api.Interaction;
import org.eclipse.mosaic.rti.api.RtiAmbassador;
import org.eclipse.mosaic.rti.api.federatestarter.FederateStarter;
import org.eclipse.mosaic.rti.api.parameters.AmbassadorParameter;
import org.eclipse.mosaic.rti.api.parameters.FederateDescriptor;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;

/**
 * Demo application for CARLA XML-RPC architecture validation.
 * 
 * This application demonstrates the complete actor lifecycle management
 * and traffic light control using the new XML-RPC architecture.
 */
public class CarlaXmlRpcDemoApp implements FederateStarter {

    private static final Logger log = LoggerFactory.getLogger(CarlaXmlRpcDemoApp.class);
    
    private RtiAmbassador rtiAmbassador;
    private long currentTime = 0;
    private boolean demoStarted = false;
    
    // Demo configuration
    private int vehicleCount = 5;
    private int trafficLightCount = 2;
    private int sensorCount = 3;
    
    // Demo state
    private final Set<String> registeredVehicles = new HashSet<>();
    private final Set<String> registeredTrafficLights = new HashSet<>();
    private final Set<String> registeredSensors = new HashSet<>();
    private final Map<String, VehicleData> vehicleStates = new HashMap<>();

    @Override
    public void initialize(FederateDescriptor federateDescriptor, AmbassadorParameter ambassadorParameter) {
        log.info("Initializing CARLA XML-RPC Demo Application");
        
        this.rtiAmbassador = ambassadorParameter.getRtiAmbassador();
        
        // Load configuration
        this.vehicleCount = federateDescriptor.getConfiguration().getInt("vehicleCount", 5);
        this.trafficLightCount = federateDescriptor.getConfiguration().getInt("trafficLightCount", 2);
        this.sensorCount = federateDescriptor.getConfiguration().getInt("sensorCount", 3);
        
        log.info("Demo configuration: {} vehicles, {} traffic lights, {} sensors", 
                vehicleCount, trafficLightCount, sensorCount);
    }

    @Override
    public void start() {
        log.info("Starting CARLA XML-RPC Demo Application");
        
        try {
            // Register for interactions
            rtiAmbassador.subscribeInteraction(VehicleUpdate.class);
            rtiAmbassador.subscribeInteraction(TrafficLightUpdate.class);
            rtiAmbassador.subscribeInteraction(DetectorResult.class);
            
            log.info("Demo application started successfully");
            
        } catch (Exception e) {
            log.error("Failed to start demo application", e);
            throw new RuntimeException("Failed to start demo application", e);
        }
    }

    @Override
    public void stop() {
        log.info("Stopping CARLA XML-RPC Demo Application");
        
        // Clean up demo state
        registeredVehicles.clear();
        registeredTrafficLights.clear();
        registeredSensors.clear();
        vehicleStates.clear();
        
        log.info("Demo application stopped successfully");
    }

    @Override
    public void processInteraction(Interaction interaction) {
        try {
            if (interaction instanceof VehicleUpdate) {
                processVehicleUpdate((VehicleUpdate) interaction);
            } else if (interaction instanceof TrafficLightUpdate) {
                processTrafficLightUpdate((TrafficLightUpdate) interaction);
            } else if (interaction instanceof DetectorResult) {
                processDetectorResult((DetectorResult) interaction);
            }
        } catch (Exception e) {
            log.error("Error processing interaction: {}", interaction.getClass().getSimpleName(), e);
        }
    }

    @Override
    public void processTimeAdvanceGrant(long time) {
        try {
            currentTime = time;
            
            // Start demo after 1 second
            if (!demoStarted && time >= 1000) {
                startDemo();
                demoStarted = true;
            }
            
            // Run demo logic
            if (demoStarted) {
                runDemoLogic();
            }
            
            // Request next time advance
            rtiAmbassador.requestAdvanceTime(time + 100);
            
        } catch (Exception e) {
            log.error("Error processing time advance grant", e);
        }
    }

    /**
     * Start the demo by creating vehicles, traffic lights, and sensors.
     */
    private void startDemo() {
        log.info("Starting demo at time {}", currentTime);
        
        try {
            // Create vehicles
            for (int i = 0; i < vehicleCount; i++) {
                createDemoVehicle("demo_vehicle_" + i, i);
            }
            
            // Create traffic lights
            for (int i = 0; i < trafficLightCount; i++) {
                createDemoTrafficLight("demo_trafficlight_" + i);
            }
            
            // Create sensors
            for (int i = 0; i < sensorCount; i++) {
                createDemoSensor("demo_sensor_" + i, i);
            }
            
            log.info("Demo initialization completed");
            
        } catch (Exception e) {
            log.error("Error starting demo", e);
        }
    }

    /**
     * Run the main demo logic.
     */
    private void runDemoLogic() {
        try {
            // Update vehicle positions
            updateVehiclePositions();
            
            // Cycle traffic lights
            cycleTrafficLights();
            
        } catch (Exception e) {
            log.error("Error running demo logic", e);
        }
    }

    /**
     * Create a demo vehicle.
     */
    private void createDemoVehicle(String vehicleId, int index) {
        try {
            // Create vehicle data
            CartesianPoint position = new CartesianPoint(100 + index * 50, 200, 0);
            GeoPoint geoPosition = new GeoPoint(0, 0); // Placeholder
            IRoadPosition roadPosition = new SimpleRoadPosition("road_1", 0, 0);
            
            VehicleType vehicleType = new VehicleType("car", 4.5, 2.0, 1.5);
            
            VehicleData vehicleData = new VehicleData(
                vehicleId,
                vehicleType,
                position,
                geoPosition,
                roadPosition,
                0.0, // heading
                0.0, // speed
                currentTime
            );
            
            // Register vehicle
            VehicleRegistration registration = new VehicleRegistration(currentTime, vehicleData);
            rtiAmbassador.triggerInteraction(registration);
            
            registeredVehicles.add(vehicleId);
            vehicleStates.put(vehicleId, vehicleData);
            
            log.info("Created demo vehicle: {}", vehicleId);
            
        } catch (Exception e) {
            log.error("Error creating demo vehicle: {}", vehicleId, e);
        }
    }

    /**
     * Create a demo traffic light.
     */
    private void createDemoTrafficLight(String trafficLightId) {
        try {
            // Register traffic light
            TrafficLightRegistration registration = new TrafficLightRegistration(currentTime, trafficLightId);
            rtiAmbassador.triggerInteraction(registration);
            
            registeredTrafficLights.add(trafficLightId);
            
            log.info("Created demo traffic light: {}", trafficLightId);
            
        } catch (Exception e) {
            log.error("Error creating demo traffic light: {}", trafficLightId, e);
        }
    }

    /**
     * Create a demo sensor.
     */
    private void createDemoSensor(String sensorId, int index) {
        try {
            // Create detector
            CartesianPoint position = new CartesianPoint(150 + index * 30, 250, 2.0);
            Detector detector = new Detector(
                sensorId,
                DetectorType.CAMERA,
                position,
                0.0, // pitch
                0.0, // roll
                0.0  // yaw
            );
            
            // Register detector
            DetectorRegistration registration = new DetectorRegistration(currentTime, "demo_infrastructure", detector);
            rtiAmbassador.triggerInteraction(registration);
            
            registeredSensors.add(sensorId);
            
            log.info("Created demo sensor: {}", sensorId);
            
        } catch (Exception e) {
            log.error("Error creating demo sensor: {}", sensorId, e);
        }
    }

    /**
     * Update vehicle positions for demo.
     */
    private void updateVehiclePositions() {
        try {
            for (String vehicleId : registeredVehicles) {
                VehicleData currentData = vehicleStates.get(vehicleId);
                if (currentData != null) {
                    // Update position (simple movement)
                    CartesianPoint newPosition = new CartesianPoint(
                        currentData.getPosition().getX() + 1.0,
                        currentData.getPosition().getY(),
                        currentData.getPosition().getZ()
                    );
                    
                    VehicleData updatedData = new VehicleData(
                        currentData.getName(),
                        currentData.getVehicleType(),
                        newPosition,
                        currentData.getGeoPosition(),
                        currentData.getRoadPosition(),
                        currentData.getHeading(),
                        currentData.getSpeed() + 0.1,
                        currentTime
                    );
                    
                    // Send vehicle update
                    VehicleUpdate update = new VehicleUpdate(currentTime, updatedData);
                    rtiAmbassador.triggerInteraction(update);
                    
                    vehicleStates.put(vehicleId, updatedData);
                    
                    log.debug("Updated vehicle {} position", vehicleId);
                }
            }
        } catch (Exception e) {
            log.error("Error updating vehicle positions", e);
        }
    }

    /**
     * Cycle traffic lights for demo.
     */
    private void cycleTrafficLights() {
        try {
            int lightIndex = 0;
            for (String trafficLightId : registeredTrafficLights) {
                // Cycle through states based on time
                TrafficLightState state;
                long cycleTime = (currentTime / 5000) % 3; // 5 second cycles
                
                switch ((int) cycleTime) {
                    case 0:
                        state = TrafficLightState.RED;
                        break;
                    case 1:
                        state = TrafficLightState.YELLOW;
                        break;
                    case 2:
                        state = TrafficLightState.GREEN;
                        break;
                    default:
                        state = TrafficLightState.RED;
                }
                
                // Send traffic light update
                TrafficLightUpdate update = new TrafficLightUpdate(currentTime, trafficLightId, state);
                rtiAmbassador.triggerInteraction(update);
                
                log.debug("Updated traffic light {} to {}", trafficLightId, state);
                lightIndex++;
            }
        } catch (Exception e) {
            log.error("Error cycling traffic lights", e);
        }
    }

    /**
     * Process vehicle update from CARLA.
     */
    private void processVehicleUpdate(VehicleUpdate update) {
        try {
            VehicleData vehicleData = update.getVehicle();
            String vehicleId = vehicleData.getName();
            
            log.info("Received vehicle update for {}: position=({}, {}, {}), speed={}", 
                    vehicleId,
                    vehicleData.getPosition().getX(),
                    vehicleData.getPosition().getY(),
                    vehicleData.getPosition().getZ(),
                    vehicleData.getSpeed());
            
        } catch (Exception e) {
            log.error("Error processing vehicle update", e);
        }
    }

    /**
     * Process traffic light update from CARLA.
     */
    private void processTrafficLightUpdate(TrafficLightUpdate update) {
        try {
            String trafficLightId = update.getTrafficLightId();
            TrafficLightState state = update.getTrafficLightState();
            
            log.info("Received traffic light update for {}: state={}", trafficLightId, state);
            
        } catch (Exception e) {
            log.error("Error processing traffic light update", e);
        }
    }

    /**
     * Process detector result from CARLA.
     */
    private void processDetectorResult(DetectorResult result) {
        try {
            String sensorId = result.getSensorId();
            DetectedObject[] detectedObjects = result.getDetectedObjects();
            
            log.info("Received detector result from {}: {} objects detected", 
                    sensorId, detectedObjects.length);
            
            // Log detected objects
            for (DetectedObject obj : detectedObjects) {
                log.debug("Detected object: type={}, position=({}, {}, {})", 
                        obj.getType(),
                        obj.getPosition().getX(),
                        obj.getPosition().getY(),
                        obj.getPosition().getZ());
            }
            
        } catch (Exception e) {
            log.error("Error processing detector result", e);
        }
    }
}
