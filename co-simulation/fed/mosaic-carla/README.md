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



## Log
### Carla.log
```bash
2025-09-23 20:57:47,565 INFO  CarlaAmbassador:144 - carlaConfig.updateInterval: 100
2025-09-23 20:57:47,566 INFO  CarlaAmbassador:188 - use carla path from configuration file: /opt/carla/
2025-09-23 20:57:47,630 TRACE AbstractFederateAmbassador:170 - setRtiAmbassador(RtiAmbassador rti)
2025-09-23 20:57:48,115 TRACE AbstractFederateAmbassador:272 - initialize(long startTime, long endTime); startTime: 0, endTime: 600000000000
2025-09-23 20:57:48,115 INFO  CarlaAmbassador:380 - Start Federate local
2025-09-23 20:57:48,115 INFO  CarlaAmbassador:381 - Directory: ./tmp/carla
2025-09-23 20:57:48,116 INFO  CarlaAmbassador:311 - Use connection bridge path from configuration file: /opt/carma-simulation/scenarios/Town04/carla; bridge.sh
2025-09-23 20:57:48,117 INFO  CarlaAmbassador:360 - Client connected
2025-09-23 20:57:48,118 INFO  CarlaAmbassador:251 - Start adding ACTOR_LIB server: http://127.0.0.1:8090/RPC2
2025-09-23 20:57:48,120 ERROR ProcessLoggingThread:72 - Process carla : chmod: changing permissions of '/opt/carla/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping': Operation not permitted
2025-09-23 20:57:48,128 INFO  CarlaXmlRpcClient:146 - CARLA XML-RPC client initialized for ACTOR_LIB server: http://127.0.0.1:8090/RPC2
2025-09-23 20:57:48,128 INFO  CarlaMultiXmlRpcManager:54 - Added ACTOR_LIB client for URL: http://127.0.0.1:8090/RPC2
2025-09-23 20:57:48,128 INFO  CarlaAmbassador:253 - Added ACTOR_LIB server: http://127.0.0.1:8090/RPC2
2025-09-23 20:57:55,564 INFO  CarlaConnection:75 - Carla Connected
2025-09-23 20:57:55,564 INFO  CarlaConnection:81 - Begin Co-Simulation
2025-09-23 20:57:55,564 DEBUG CarlaAmbassador:671 - Ignoring legacy TraCI request path in favor of XML-RPC interactions
2025-09-23 20:57:57,088 INFO  CarlaMultiXmlRpcManager:69 - Connecting to ACTOR_LIB server...
2025-09-23 20:57:57,088 INFO  CarlaXmlRpcClient:175 - Attempting to connect to CARLA XML-RPC server (attempt 1/60)
2025-09-23 20:57:57,088 DEBUG CarlaXmlRpcClient:866 - Executing XML-RPC call connect (request #1)
2025-09-23 20:57:57,239 DEBUG CarlaXmlRpcClient:870 - XML-RPC call connect completed successfully (request #1)
2025-09-23 20:57:57,239 INFO  CarlaXmlRpcClient:182 - Successfully connected to CARLA XML-RPC server
2025-09-23 20:57:57,239 INFO  CarlaMultiXmlRpcManager:72 - Successfully connected to ACTOR_LIB server
2025-09-23 21:07:57,325 INFO  CarlaAmbassador:556 - Closing CARLA connection.
2025-09-23 21:07:57,325 INFO  CarlaConnection:115 - carla socket closing
2025-09-23 21:07:57,325 INFO  CarlaConnection:118 - carla connection server socket closing
2025-09-23 21:07:57,325 DEBUG CarlaXmlRpcClient:866 - Executing XML-RPC call disconnect (request #2)
2025-09-23 21:07:57,325 ERROR CarlaConnection:95 - error occurs during data streaming: Socket closed
2025-09-23 21:07:57,327 DEBUG CarlaXmlRpcClient:870 - XML-RPC call disconnect completed successfully (request #2)
2025-09-23 21:07:57,327 INFO  CarlaXmlRpcClient:218 - Successfully disconnected from CARLA XML-RPC server
2025-09-23 21:07:57,327 INFO  CarlaMultiXmlRpcManager:92 - Disconnected from ACTOR_LIB server
2025-09-23 21:08:07,327 INFO  CarlaAmbassador:588 - Finished simulation
------
```
### MOSAIC.log
```bash
2025-09-23 20:57:47,520 INFO  MosaicSimulation:224 - Running Eclipse MOSAIC 22.1-SNAPSHOT on Java JRE v11.0.19 (Ubuntu)
2025-09-23 20:57:47,627 INFO  LocalFederationManagement:87 - Start federation with id 'Town04'
2025-09-23 20:57:47,627 INFO  LocalFederationManagement:92 - Add ambassador/federate with id 'application'
2025-09-23 20:57:47,627 INFO  LocalFederationManagement:92 - Add ambassador/federate with id 'carla'
2025-09-23 20:57:47,628 INFO  LocalFederationManagement:186 - Deploying federate 'carla' locally in ./tmp/carla
2025-09-23 20:57:47,629 INFO  LocalFederationManagement:237 - Starting federate 'carla' locally in ./tmp/carla
2025-09-23 20:57:47,630 INFO  LocalFederationManagement:92 - Add ambassador/federate with id 'carma'
2025-09-23 20:57:47,630 INFO  LocalFederationManagement:92 - Add ambassador/federate with id 'infrastructure'
2025-09-23 20:57:47,630 INFO  LocalFederationManagement:92 - Add ambassador/federate with id 'carma-cloud'
2025-09-23 20:57:47,630 INFO  LocalFederationManagement:92 - Add ambassador/federate with id 'mapping'
2025-09-23 20:57:47,630 INFO  LocalFederationManagement:92 - Add ambassador/federate with id 'ns3'
2025-09-23 20:57:47,630 INFO  LocalFederationManagement:186 - Deploying federate 'ns3' locally in ./tmp/ns3
2025-09-23 20:57:47,848 INFO  LocalFederationManagement:237 - Starting federate 'ns3' locally in ./tmp/ns3
2025-09-23 20:57:47,917 INFO  LocalFederationManagement:92 - Add ambassador/federate with id 'sumo'
2025-09-23 20:57:47,918 INFO  LocalFederationManagement:186 - Deploying federate 'sumo' locally in ./tmp/sumo
2025-09-23 20:57:47,920 INFO  LocalFederationManagement:237 - Starting federate 'sumo' locally in ./tmp/sumo
2025-09-23 20:57:47,922 WARN  SpawningFramework:245 - You didn't define any spawners in your mapping config, which means that there will be no vehicles in your simulation. Keep this in mind when troubleshooting.
2025-09-23 20:57:48,114 INFO  CarmaV2xMessageReceiver:78 - CarmaV2xMessageReceiver started listening on UDP port: 1517.
2025-09-23 20:57:48,115 INFO  CarmaV2xMessageReceiver:78 - CarmaV2xMessageReceiver started listening on UDP port: 1516.
2025-09-23 21:08:07,391 INFO  MosaicStarter:311 - Simulation started: 2025-09-23 20:57:47
2025-09-23 21:08:07,391 INFO  MosaicStarter:312 - Simulation ended: 2025-09-23 21:08:07
2025-09-23 21:08:07,391 INFO  MosaicStarter:313 - Finishing simulation (duration: 00h 10m 19.471s)
```

```bash
docker run --rm -it --gpus all --net=host -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 --user=carma usdotfhwastol/cdasim:latest /bin/bash
cd bridge
python carla_v0.9_xmlrpc_server.py
./mosaic.sh -s Town04
```