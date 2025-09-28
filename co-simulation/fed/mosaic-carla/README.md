# MOSAIC CARLA Federate - XML-RPC Client Implementation

## Overview

The MOSAIC CARLA Federate includes a comprehensive XML-RPC client that interacts with the CARLA XML-RPC Server, bridging CARLA to the MOSAIC co-simulation framework. This implementation provides robust error handling, retry logic, and comprehensive actor management for seamless integration.

## Features

### Core Functionality
- **Connection Management**: Robust connection handling with retry logic and timeout configuration
- **Actor Discovery**: Dynamic discovery of active actors in CARLA using filter patterns
- **Granular Data Access**: Individual getter methods for actor properties (transform, velocity, acceleration, etc.)
- **State Synchronization**: Comprehensive actor state management between MOSAIC and CARLA
- **Traffic Light Control**: Full traffic light state management and control
- **Sensor Management**: Creation and management of various sensor types
- **Map Management**: Dynamic map loading and management

### Error Handling & Reliability
- **Retry Logic**: Configurable retry attempts for all XML-RPC calls
- **Connection Monitoring**: Continuous connection state monitoring
- **Graceful Degradation**: Robust error handling with detailed logging
- **Timeout Management**: Configurable timeouts for different operation types

## Architecture

### Tick-Loop Integration (MOSAIC's Perspective)

The federate implements the following tick-loop integration pattern:

1. **Discover Actors (As Needed)**: Call `get_active_actor_ids()` to get relevant actor IDs
2. **Update CARLA Actors (Optional)**: Synchronize actor states from other simulators (e.g., SUMO)
3. **Step CARLA**: Call `advance_simulation()` to progress CARLA's simulation state
4. **Retrieve Needed CARLA State**: Call specific getter methods for required data
5. **Distribute Data**: Provide retrieved data to the MOSAIC RTI

### XML-RPC Methods Implemented

#### Connection & Simulation Control
- `connect()` - Establish connection to CARLA server
- `disconnect()` - Disconnect from CARLA server
- `is_connected()` - Check connection status
- `advance_simulation()` - Step CARLA simulation by one tick
- `get_simulation_time()` - Get current simulation time

#### Actor Discovery & Management
- `get_active_actor_ids(filter_pattern)` - Discover active actors with filter
- `get_actor_basic_info(actor_id)` - Get basic actor information
- `get_actor_transform(actor_id)` - Get actor position and rotation
- `get_actor_velocity(actor_id)` - Get actor velocity
- `get_actor_acceleration(actor_id)` - Get actor acceleration
- `get_actor_angular_velocity(actor_id)` - Get actor angular velocity
- `get_actor_bounding_box(actor_id)` - Get actor bounding box
- `get_vehicle_light_state(actor_id)` - Get vehicle light state
- `set_actor_state_properties(actor_id, properties)` - Set comprehensive actor state

#### Actor Lifecycle
- `spawn_actor(type, id, location, rotation, attributes)` - Spawn new actor
- `destroy_actor(actor_id)` - Destroy actor
- `update_actor_transform(actor_id, location, rotation)` - Update actor transform
- `update_actor_velocity(actor_id, velocity)` - Update actor velocity
- `get_all_actors()` - Get all actors with basic information

#### Traffic Light Management
- `get_traffic_light_state(traffic_light_id)` - Get individual traffic light state
- `get_all_traffic_light_states()` - Get all traffic light states
- `set_traffic_light_state(traffic_light_id, state)` - Set traffic light state
- `set_traffic_light_timer(traffic_light_id, time)` - Set traffic light timer

#### Sensor Management
- `create_sensor(type, id, location, rotation, attributes)` - Create sensor
- `destroy_sensor(sensor_id)` - Destroy sensor
- `get_sensor_data(sensor_id)` - Get sensor data
- `get_detected_objects(infrastructure_id, sensor_id)` - Get detected objects

#### Map Management
- `get_map_name()` - Get current map name
- `get_available_maps()` - Get list of available maps
- `load_map(map_name)` - Load specific map

## Configuration

### Basic Configuration
```json
{
  "carlaServerUrl": "http://localhost:8090",
  "connectionRetries": 5,
  "timeStep": 0.1,
  "mapName": "Town01"
}
```

### Advanced Configuration
```json
{
  "carlaServerUrl": "http://localhost:8090",
  "connectionRetries": 5,
  "timeStep": 0.1,
  "mapName": "Town01",
  "logLevel": "INFO",
  "enableActorManagement": true,
  "enableTrafficLightControl": true,
  "enableSensorManagement": true,
  "actorTypes": {
    "car": "vehicle.tesla.model3",
    "truck": "vehicle.truck",
    "bus": "vehicle.bus",
    "motorcycle": "vehicle.kawasaki.ninja",
    "bicycle": "vehicle.bh.crossbike",
    "pedestrian": "walker.pedestrian.0001"
  },
  "sensorTypes": {
    "camera": "sensor.camera.rgb",
    "lidar": "sensor.lidar.ray_cast",
    "radar": "sensor.other.radar",
    "gps": "sensor.other.gnss",
    "imu": "sensor.other.imu"
  }
}
```

## Usage Examples

### Basic Usage
```java
// Initialize client
URL serverUrl = new URL("http://localhost:8090");
CarlaXmlRpcClient client = new CarlaXmlRpcClient(serverUrl);

// Connect to server
client.connect(5);

// Advance simulation
client.advanceSimulation();

// Get active actors
List<Integer> actorIds = client.getActiveActorIds("vehicle.*");

// Get actor data
for (Integer actorId : actorIds) {
    Map<String, Object> transform = client.getActorTransform(actorId);
    Map<String, Object> velocity = client.getActorVelocity(actorId);
    // Process data...
}

// Disconnect
client.disconnect();
```

### Actor Management
```java
// Spawn actor
List<Double> location = Arrays.asList(100.0, 200.0, 0.0);
List<Double> rotation = Arrays.asList(0.0, 90.0, 0.0);
Map<String, Object> attributes = new HashMap<>();
attributes.put("role_name", "test_vehicle");

boolean success = client.spawnActor("vehicle.tesla.model3", "test_actor_1", 
                                   location, rotation, attributes);

// Update actor state
Map<String, Object> properties = new HashMap<>();
Map<String, Object> transform = new HashMap<>();
Map<String, Object> location = new HashMap<>();
location.put("x", 150.0);
location.put("y", 250.0);
location.put("z", 0.0);
transform.put("location", location);
properties.put("transform", transform);

client.setActorStateProperties("test_actor_1", properties);
```

### Traffic Light Control
```java
// Get all traffic light states
List<Map<String, Object>> states = client.getAllTrafficLightStates();

// Set traffic light state
client.setTrafficLightState("traffic_light_1", "Red");

// Set traffic light timer
client.setTrafficLightTimer("traffic_light_1", 30.0);
```

### Sensor Management
```java
// Create sensor
List<Double> location = Arrays.asList(0.0, 0.0, 2.0);
List<Double> rotation = Arrays.asList(0.0, 0.0, 0.0);
Map<String, Object> attributes = new HashMap<>();
attributes.put("fov", "90");

client.createSensor("sensor.camera.rgb", "camera_1", location, rotation, attributes);

// Get sensor data
Map<String, Object> sensorData = client.getSensorData("camera_1");
```

## Error Handling

The client implements comprehensive error handling:

- **Retry Logic**: All XML-RPC calls include configurable retry attempts
- **Connection Monitoring**: Continuous monitoring of connection state
- **Detailed Logging**: Extensive logging for debugging and monitoring
- **Graceful Degradation**: Robust handling of network issues and server errors

### Error Recovery
```java
try {
    client.advanceSimulation();
} catch (XmlRpcException e) {
    log.error("Failed to advance simulation: {}", e.getMessage());
    // Handle error - could retry, fallback, or notify user
}
```

## Performance Considerations

- **Connection Pooling**: Efficient connection management
- **Batch Operations**: Support for batch actor operations
- **Selective Data Retrieval**: Only retrieve required actor properties
- **Caching**: Local caching of frequently accessed data

## Integration with MOSAIC

The federate integrates seamlessly with the MOSAIC co-simulation framework:

- **RTI Integration**: Full integration with MOSAIC RTI
- **Interaction Handling**: Processes MOSAIC interactions (VehicleRegistration, VehicleUpdate, etc.)
- **Time Management**: Synchronized time advancement with other federates
- **Data Distribution**: Distributes CARLA data to other federates via RTI

## Troubleshooting

### Common Issues

1. **Connection Failures**
   - Verify CARLA server is running on correct port
   - Check network connectivity
   - Increase connection timeout if needed

2. **Actor Not Found**
   - Verify actor ID exists in CARLA
   - Check actor filter patterns
   - Ensure actor is still active

3. **Performance Issues**
   - Reduce frequency of actor discovery calls
   - Use selective data retrieval
   - Optimize batch operations

### Debugging

Enable debug logging:
```json
{
  "logLevel": "DEBUG"
}
```

Monitor XML-RPC calls and responses in the logs for detailed debugging information.

## Dependencies

- Apache XML-RPC Client
- SLF4J for logging
- Gson for JSON processing
- MOSAIC RTI API

## License

50.36 [ERROR] Failed to execute goal org.apache.maven.plugins:maven-compiler-plugin:3.8.1:compile (default-compile) on project mosaic-sumo: Compilation failure: Compilation failure: 
50.36 [ERROR] /home/carma/src/co-simulation/fed/mosaic-sumo/src/main/java/org/eclipse/mosaic/fed/sumo/ambassador/SumoAmbassador.java:[32,51] cannot find symbol
50.36 [ERROR]   symbol:   class CarlaActorResponse
50.36 [ERROR]   location: package org.eclipse.mosaic.interactions.application
50.36 [ERROR] /home/carma/src/co-simulation/fed/mosaic-sumo/src/main/java/org/eclipse/mosaic/fed/sumo/ambassador/SumoAmbassador.java:[33,51] cannot find symbol
50.36 [ERROR]   symbol:   class CarlaTrafficLightResponse
50.36 [ERROR]   location: package org.eclipse.mosaic.interactions.application
50.36 [ERROR] /home/carma/src/co-simulation/fed/mosaic-sumo/src/main/java/org/eclipse/mosaic/fed/sumo/ambassador/AbstractSumoAmbassador.java:[69,51] cannot find symbol
50.36 [ERROR]   symbol:   class CarlaActorRequest
50.36 [ERROR]   location: package org.eclipse.mosaic.interactions.application
50.36 [ERROR] /home/carma/src/co-simulation/fed/mosaic-sumo/src/main/java/org/eclipse/mosaic/fed/sumo/ambassador/SumoAmbassador.java:[574,37] cannot find symbol
50.36 [ERROR]   symbol:   class CarlaActorResponse
50.36 [ERROR]   location: class org.eclipse.mosaic.fed.sumo.ambassador.SumoAmbassador
50.36 [ERROR] /home/carma/src/co-simulation/fed/mosaic-sumo/src/main/java/org/eclipse/mosaic/fed/sumo/ambassador/SumoAmbassador.java:[622,37] cannot find symbol
50.36 [ERROR]   symbol:   class CarlaTrafficLightResponse
50.36 [ERROR]   location: class org.eclipse.mosaic.fed.sumo.ambassador.SumoAmbassador
50.36 [ERROR] /home/carma/src/co-simulation/fed/mosaic-sumo/src/main/java/org/eclipse/mosaic/fed/sumo/ambassador/SumoAmbassador.java:[622,18] receiveInteraction(CarlaTrafficLightResponse) in org.eclipse.mosaic.fed.sumo.ambassador.SumoAmbassador cannot implement receiveInteraction(org.eclipse.mosaic.rti.api.Interaction) in org.eclipse.mosaic.rti.api.FederateAmbassador
50.36 [ERROR]   attempting to assign weaker access privileges; was public
50.36 [ERROR] /home/carma/src/co-simulation/fed/mosaic-sumo/src/main/java/org/eclipse/mosaic/fed/sumo/ambassador/SumoAmbassador.java:[129,17] reference to receiveInteraction is ambiguous
50.36 [ERROR]   both method receiveInteraction(CarlaActorResponse) in org.eclipse.mosaic.fed.sumo.ambassador.SumoAmbassador and method receiveInteraction(CarlaTrafficLightResponse) in org.eclipse.mosaic.fed.sumo.ambassador.SumoAmbassador match
50.36 [ERROR] /home/carma/src/co-simulation/fed/mosaic-sumo/src/main/java/org/eclipse/mosaic/fed/sumo/ambassador/SumoAmbassador.java:[131,17] reference to receiveInteraction is ambiguous
50.36 [ERROR]   both method receiveInteraction(CarlaActorResponse) in org.eclipse.mosaic.fed.sumo.ambassador.SumoAmbassador and method receiveInteraction(CarlaTrafficLightResponse) in org.eclipse.mosaic.fed.sumo.ambassador.SumoAmbassador match
50.36 [ERROR] /home/carma/src/co-simulation/fed/mosaic-sumo/src/main/java/org/eclipse/mosaic/fed/sumo/ambassador/SumoAmbassador.java:[133,17] reference to receiveInteraction is ambiguous
50.36 [ERROR]   both method receiveInteraction(CarlaActorResponse) in org.eclipse.mosaic.fed.sumo.ambassador.SumoAmbassador and method receiveInteraction(CarlaTrafficLightResponse) in org.eclipse.mosaic.fed.sumo.ambassador.SumoAmbassador match
50.36 [ERROR] /home/carma/src/co-simulation/fed/mosaic-sumo/src/main/java/org/eclipse/mosaic/fed/sumo/ambassador/SumoAmbassador.java:[134,51] cannot find symbol
50.36 [ERROR]   symbol:   variable CarlaActorResponse
50.36 [ERROR]   location: class org.eclipse.mosaic.fed.sumo.ambassador.SumoAmbassador
50.36 [ERROR] /home/carma/src/co-simulation/fed/mosaic-sumo/src/main/java/org/eclipse/mosaic/fed/sumo/ambassador/SumoAmbassador.java:[135,38] cannot find symbol
50.36 [ERROR]   symbol:   class CarlaActorResponse
50.36 [ERROR]   location: class org.eclipse.mosaic.fed.sumo.ambassador.SumoAmbassador
50.36 [ERROR] /home/carma/src/co-simulation/fed/mosaic-sumo/src/main/java/org/eclipse/mosaic/fed/sumo/ambassador/SumoAmbassador.java:[136,51] cannot find symbol
50.36 [ERROR]   symbol:   variable CarlaTrafficLightResponse
50.36 [ERROR]   location: class org.eclipse.mosaic.fed.sumo.ambassador.SumoAmbassador
50.36 [ERROR] /home/carma/src/co-simulation/fed/mosaic-sumo/src/main/java/org/eclipse/mosaic/fed/sumo/ambassador/SumoAmbassador.java:[137,38] cannot find symbol
50.36 [ERROR]   symbol:   class CarlaTrafficLightResponse
50.36 [ERROR]   location: class org.eclipse.mosaic.fed.sumo.ambassador.SumoAmbassador
50.36 [ERROR] /home/carma/src/co-simulation/fed/mosaic-sumo/src/main/java/org/eclipse/mosaic/fed/sumo/ambassador/SumoAmbassador.java:[139,17] reference to receiveInteraction is ambiguous
50.36 [ERROR]   both method receiveInteraction(CarlaActorResponse) in org.eclipse.mosaic.fed.sumo.ambassador.SumoAmbassador and method receiveInteraction(CarlaTrafficLightResponse) in org.eclipse.mosaic.fed.sumo.ambassador.SumoAmbassador match
50.36 [ERROR] /home/carma/src/co-simulation/fed/mosaic-sumo/src/main/java/org/eclipse/mosaic/fed/sumo/ambassador/SumoAmbassador.java:[141,17] reference to receiveInteraction is ambiguous
50.36 [ERROR]   both method receiveInteraction(CarlaActorResponse) in org.eclipse.mosaic.fed.sumo.ambassador.SumoAmbassador and method receiveInteraction(CarlaTrafficLightResponse) in org.eclipse.mosaic.fed.sumo.ambassador.SumoAmbassador match
50.36 [ERROR] /home/carma/src/co-simulation/fed/mosaic-sumo/src/main/java/org/eclipse/mosaic/fed/sumo/ambassador/SumoAmbassador.java:[143,17] reference to receiveInteraction is ambiguous
50.36 [ERROR]   both method receiveInteraction(CarlaActorResponse) in org.eclipse.mosaic.fed.sumo.ambassador.SumoAmbassador and method receiveInteraction(CarlaTrafficLightResponse) in org.eclipse.mosaic.fed.sumo.ambassador.SumoAmbassador match
50.36 [ERROR] /home/carma/src/co-simulation/fed/mosaic-sumo/src/main/java/org/eclipse/mosaic/fed/sumo/ambassador/SumoAmbassador.java:[168,13] reference to receiveInteraction is ambiguous
50.36 [ERROR]   both method receiveInteraction(CarlaActorResponse) in org.eclipse.mosaic.fed.sumo.ambassador.SumoAmbassador and method receiveInteraction(CarlaTrafficLightResponse) in org.eclipse.mosaic.fed.sumo.ambassador.SumoAmbassador match
50.36 [ERROR] /home/carma/src/co-simulation/fed/mosaic-sumo/src/main/java/org/eclipse/mosaic/fed/sumo/ambassador/SumoAmbassador.java:[574,18] receiveInteraction(CarlaActorResponse) in org.eclipse.mosaic.fed.sumo.ambassador.SumoAmbassador cannot override receiveInteraction(org.eclipse.mosaic.rti.api.Interaction) in org.eclipse.mosaic.rti.api.AbstractFederateAmbassador
50.36 [ERROR]   overridden method is final
50.36 [ERROR] /home/carma/src/co-simulation/fed/mosaic-sumo/src/main/java/org/eclipse/mosaic/fed/sumo/ambassador/SumoGuiAmbassador.java:[31,8] receiveInteraction(CarlaTrafficLightResponse) in org.eclipse.mosaic.fed.sumo.ambassador.SumoAmbassador cannot implement receiveInteraction(org.eclipse.mosaic.rti.api.Interaction) in org.eclipse.mosaic.rti.api.FederateAmbassador
50.36 [ERROR]   attempting to assign weaker access privileges; was public
50.36 [ERROR] -> [Help 1]
50.36 [ERROR] 
50.36 [ERROR] To see the full stack trace of the errors, re-run Maven with the -e switch.
50.36 [ERROR] Re-run Maven using the -X switch to enable full debug logging.
50.36 [ERROR] 
50.36 [ERROR] For more information about the errors and possible solutions, please read the following articles:
50.36 [ERROR] [Help 1] http://cwiki.apache.org/confluence/display/MAVEN/MojoFailureException
50.36 [ERROR] 
50.36 [ERROR] After correcting the problems, you can resume the build with the command
50.36 [ERROR]   mvn <args> -rf :mosaic-sumo
```bash
docker run --rm -it --gpus all --net=host -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 --user=carma usdotfhwastol/cdasim:latest /bin/bash
cd bridge
python carla_v0.9_xmlrpc_server.py
./mosaic.sh -s Town04
```