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
  "mapName": "Town01",
  "autoLoadMap": true
}
```

### Advanced Configuration
```json
{
  "carlaServerUrl": "http://localhost:8090",
  "connectionRetries": 5,
  "timeStep": 0.1,
  "mapName": "Town04",
  "autoLoadMap": true,
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

### Map Management
```java
// Get current map name
String currentMap = client.getMapName();
System.out.println("Current map: " + currentMap);

// Get available maps
List<String> availableMaps = client.getAvailableMaps();
System.out.println("Available maps: " + availableMaps);

// Load a specific map
boolean success = client.loadMap("Town04");
if (success) {
    System.out.println("Successfully loaded Town04 map");
} else {
    System.out.println("Failed to load Town04 map");
}
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
2025-10-23 19:21:27,975 DEBUG CarlaXmlRpcClient:675 - getAllActorsExcludingSumo: 3 total actors, 1 after filtering SUMO vehicles
2025-10-23 19:21:27,975 DEBUG CarlaAmbassador:1239 - Current CARLA actors (excluding SUMO): 1
2025-10-23 19:21:27,975 DEBUG CarlaAmbassador:1305 - SUMO vehicle 'veh_0' already exists in mapping with CARLA ID '207', skipping spawn
2025-10-23 19:21:27,975 DEBUG CarlaAmbassador:1305 - SUMO vehicle 'veh_1' already exists in mapping with CARLA ID '208', skipping spawn
2025-10-23 19:21:27,975 DEBUG CarlaAmbassador:1459 - Successfully updated CARLA actor '207' (SUMO: 'veh_0') transform (speed: 0.0 m/s)
2025-10-23 19:21:27,976 DEBUG CarlaAmbassador:1459 - Successfully updated CARLA actor '208' (SUMO: 'veh_1') transform (speed: 0.0 m/s)
2025-10-23 19:21:27,977 DEBUG CarlaXmlRpcClient:671 - Excluding SUMO-managed actor '207' from getAllActors result
2025-10-23 19:21:27,977 DEBUG CarlaXmlRpcClient:671 - Excluding SUMO-managed actor '208' from getAllActors result
2025-10-23 19:21:27,977 DEBUG CarlaXmlRpcClient:675 - getAllActorsExcludingSumo: 3 total actors, 1 after filtering SUMO vehicles
2025-10-23 19:21:27,977 DEBUG CarlaXmlRpcClient:1080 - getActorChanges: Retrieved 1 current actors (excluding SUMO-managed)
2025-10-23 19:21:27,977 DEBUG CarlaXmlRpcClient:1120 - getActorChanges: Found added=0, updated=1, removed=0
2025-10-23 19:21:27,977 INFO  CarlaXmlRpcClient:1122 - Actor changes: added=0, updated=1, removed=0
2025-10-23 19:21:27,977 INFO  CarlaAmbassador:474 - EXTERNAL VEHICLE DETECTION: Detected changes - Added: 0, Updated: 1, Removed: 0
2025-10-23 19:21:27,977 INFO  CarlaAmbassador:476 - SUMO->CARLA MAPPING: Currently tracking 2 SUMO vehicles
2025-10-23 19:21:27,977 INFO  CarlaAmbassador:487 - EXTERNAL VEHICLE UPDATED: Actor ID=209, Info={type=vehicle.tesla.model3, transform={rotation=[Ljava.lang.Object;@66420549, location=[Ljava.lang.Object;@15dc339f}, id=209}
2025-10-23 19:21:27,977 DEBUG CarlaAmbassador:848 - Converting CARLA actor '209' with data: {type=vehicle.tesla.model3, transform={rotation=[Ljava.lang.Object;@66420549, location=[Ljava.lang.Object;@15dc339f}, id=209}
2025-10-23 19:21:27,977 DEBUG CarlaAmbassador:1029 - Converting CARLA position to SUMO: carlaX=316.15753173828125, carlaY=-172.0, yawDeg=-2.1362301777116954E-4, extentX=null
2025-10-23 19:21:27,977 DEBUG CarlaAmbassador:1051 - Applied netOffset: offsetX=503.02, offsetY=423.76, xWithOffset=819.1775317382812, yWithOffset=-595.76
2025-10-23 19:21:27,977 DEBUG CarlaAmbassador:1059 - Final SUMO coordinates: sumoX=819.1775317382812, sumoY=595.76, sumoZ=0.042643431574106216
2025-10-23 19:21:27,977 INFO  CarlaAmbassador:560 - CARLA->SUMO SYNC: Published VehicleUpdates to SUMO - added=0, updated=1, removed=0
2025-10-23 19:21:27,979 INFO  CarlaAmbassador:612 - Next time step: 33900000000
2025-10-23 19:21:27,979 INFO  CarlaAmbassador:1134 - Processing interaction with type 'VehicleUpdates' at time: 33800000000
2025-10-23 19:21:27,979 INFO  CarlaAmbassador:1146 - Processing VehicleUpdates interaction - this should trigger spawn_actor calls
2025-10-23 19:21:27,979 INFO  CarlaAmbassador:1196 - Received VehicleUpdates interaction at time 33800000000: added=0, updated=1, removed=0
2025-10-23 19:21:27,979 INFO  CarlaAmbassador:1205 - Multi-XML-RPC manager actor connection status: true
2025-10-23 19:21:27,979 INFO  CarlaAmbassador:1233 - Starting SUMO->CARLA vehicle sync: added=0, updated=1, removed=0
2025-10-23 19:21:27,980 DEBUG CarlaXmlRpcClient:671 - Excluding SUMO-managed actor '207' from getAllActors result
2025-10-23 19:21:27,980 DEBUG CarlaXmlRpcClient:671 - Excluding SUMO-managed actor '208' from getAllActors result
2025-10-23 19:21:27,980 DEBUG CarlaXmlRpcClient:675 - getAllActorsExcludingSumo: 3 total actors, 1 after filtering SUMO vehicles
2025-10-23 19:21:27,980 DEBUG CarlaAmbassador:1239 - Current CARLA actors (excluding SUMO): 1
2025-10-23 19:21:27,980 WARN  CarlaAmbassador:1462 - No CARLA ID found for SUMO vehicle '209' during update
2025-10-23 19:21:28,073 INFO  CarlaAmbassador:1134 - Processing interaction with type 'VehicleUpdates' at time: 33900000000
2025-10-23 19:21:28,073 INFO  CarlaAmbassador:1146 - Processing VehicleUpdates interaction - this should trigger spawn_actor calls
2025-10-23 19:21:28,073 INFO  CarlaAmbassador:1196 - Received VehicleUpdates interaction at time 33900000000: added=0, updated=2, removed=0
2025-10-23 19:21:28,074 INFO  CarlaAmbassador:1205 - Multi-XML-RPC manager actor connection status: true
2025-10-23 19:21:28,074 INFO  CarlaAmbassador:1233 - Starting SUMO->CARLA vehicle sync: added=0, updated=2, removed=0
2025-10-23 19:21:28,075 DEBUG CarlaXmlRpcClient:671 - Excluding SUMO-managed actor '207' from getAllActors result
2025-10-23 19:21:28,075 DEBUG CarlaXmlRpcClient:671 - Excluding SUMO-managed actor '208' from getAllActors result
2025-10-23 19:21:28,075 DEBUG CarlaXmlRpcClient:675 - getAllActorsExcludingSumo: 3 total actors, 1 after filtering SUMO vehicles
2025-10-23 19:21:28,075 DEBUG CarlaAmbassador:1239 - Current CARLA actors (excluding SUMO): 1
2025-10-23 19:21:28,075 DEBUG CarlaAmbassador:1305 - SUMO vehicle 'veh_0' already exists in mapping with CARLA ID '207', skipping spawn
2025-10-23 19:21:28,075 DEBUG CarlaAmbassador:1305 - SUMO vehicle 'veh_1' already exists in mapping with CARLA ID '208', skipping spawn
2025-10-23 19:21:28,075 DEBUG CarlaAmbassador:1459 - Successfully updated CARLA actor '207' (SUMO: 'veh_0') transform (speed: 0.0 m/s)
2025-10-23 19:21:28,076 DEBUG CarlaAmbassador:1459 - Successfully updated CARLA actor '208' (SUMO: 'veh_1') transform (speed: 0.0 m/s)
2025-10-23 19:21:28,077 DEBUG CarlaXmlRpcClient:671 - Excluding SUMO-managed actor '207' from getAllActors result
2025-10-23 19:21:28,077 DEBUG CarlaXmlRpcClient:671 - Excluding SUMO-managed actor '208' from getAllActors result
2025-10-23 19:21:28,077 DEBUG CarlaXmlRpcClient:675 - getAllActorsExcludingSumo: 3 total actors, 1 after filtering SUMO vehicles
2025-10-23 19:21:28,077 DEBUG CarlaXmlRpcClient:1080 - getActorChanges: Retrieved 1 current actors (excluding SUMO-managed)
2025-10-23 19:21:28,077 DEBUG CarlaXmlRpcClient:1120 - getActorChanges: Found added=0, updated=1, removed=0
2025-10-23 19:21:28,077 INFO  CarlaXmlRpcClient:1122 - Actor changes: added=0, updated=1, removed=0
2025-10-23 19:21:28,077 INFO  CarlaAmbassador:474 - EXTERNAL VEHICLE DETECTION: Detected changes - Added: 0, Updated: 1, Removed: 0
2025-10-23 19:21:28,077 INFO  CarlaAmbassador:476 - SUMO->CARLA MAPPING: Currently tracking 2 SUMO vehicles
2025-10-23 19:21:28,077 INFO  CarlaAmbassador:487 - EXTERNAL VEHICLE UPDATED: Actor ID=209, Info={type=vehicle.tesla.model3, transform={rotation=[Ljava.lang.Object;@6cd56321, location=[Ljava.lang.Object;@34acbc60}, id=209}
2025-10-23 19:21:28,077 DEBUG CarlaAmbassador:848 - Converting CARLA actor '209' with data: {type=vehicle.tesla.model3, transform={rotation=[Ljava.lang.Object;@6cd56321, location=[Ljava.lang.Object;@34acbc60}, id=209}
2025-10-23 19:21:28,077 DEBUG CarlaAmbassador:1029 - Converting CARLA position to SUMO: carlaX=316.4820251464844, carlaY=-172.0, yawDeg=-2.1362303232308477E-4, extentX=null
2025-10-23 19:21:28,077 DEBUG CarlaAmbassador:1051 - Applied netOffset: offsetX=503.02, offsetY=423.76, xWithOffset=819.5020251464844, yWithOffset=-595.76
2025-10-23 19:21:28,077 DEBUG CarlaAmbassador:1059 - Final SUMO coordinates: sumoX=819.5020251464844, sumoY=595.76, sumoZ=0.03718280792236328
2025-10-23 19:21:28,077 INFO  CarlaAmbassador:560 - CARLA->SUMO SYNC: Published VehicleUpdates to SUMO - added=0, updated=1, removed=0
2025-10-23 19:21:28,078 INFO  CarlaAmbassador:612 - Next time step: 34000000000
2025-10-23 19:21:28,079 INFO  CarlaAmbassador:1134 - Processing interaction with type 'VehicleUpdates' at time: 33900000000
2025-10-23 19:21:28,079 INFO  CarlaAmbassador:1146 - Processing VehicleUpdates interaction - this should trigger spawn_actor calls
2025-10-23 19:21:28,079 INFO  CarlaAmbassador:1196 - Received VehicleUpdates interaction at time 33900000000: added=0, updated=1, removed=0
2025-10-23 19:21:28,079 INFO  CarlaAmbassador:1205 - Multi-XML-RPC manager actor connection status: true
2025-10-23 19:21:28,079 INFO  CarlaAmbassador:1233 - Starting SUMO->CARLA vehicle sync: added=0, updated=1, removed=0
2025-10-23 19:21:28,080 DEBUG CarlaXmlRpcClient:671 - Excluding SUMO-managed actor '207' from getAllActors result
2025-10-23 19:21:28,080 DEBUG CarlaXmlRpcClient:671 - Excluding SUMO-managed actor '208' from getAllActors result
2025-10-23 19:21:28,080 DEBUG CarlaXmlRpcClient:675 - getAllActorsExcludingSumo: 3 total actors, 1 after filtering SUMO vehicles
2025-10-23 19:21:28,080 DEBUG CarlaAmbassador:1239 - Current CARLA actors (excluding SUMO): 1
2025-10-23 19:21:28,080 WARN  CarlaAmbassador:1462 - No CARLA ID found for SUMO vehicle '209' during update
2025-10-23 19:21:28,173 INFO  CarlaAmbassador:1134 - Processing interaction with type 'VehicleUpdates' at time: 34000000000
2025-10-23 19:21:28,173 INFO  CarlaAmbassador:1146 - Processing VehicleUpdates interaction - this should trigger spawn_actor calls
2025-10-23 19:21:28,173 INFO  CarlaAmbassador:1196 - Received VehicleUpdates interaction at time 34000000000: added=0, updated=2, removed=0
2025-10-23 19:21:28,173 INFO  CarlaAmbassador:1205 - Multi-XML-RPC manager actor connection status: true
2025-10-23 19:21:28,173 INFO  CarlaAmbassador:1233 - Starting SUMO->CARLA vehicle sync: added=0, updated=2, removed=0
2025-10-23 19:21:28,174 DEBUG CarlaXmlRpcClient:671 - Excluding SUMO-managed actor '207' from getAllActors result
2025-10-23 19:21:28,174 DEBUG CarlaXmlRpcClient:671 - Excluding SUMO-managed actor '208' from getAllActors result
2025-10-23 19:21:28,174 DEBUG CarlaXmlRpcClient:675 - getAllActorsExcludingSumo: 3 total actors, 1 after filtering SUMO vehicles
2025-10-23 19:21:28,174 DEBUG CarlaAmbassador:1239 - Current CARLA actors (excluding SUMO): 1
2025-10-23 19:21:28,174 DEBUG CarlaAmbassador:1305 - SUMO vehicle 'veh_0' already exists in mapping with CARLA ID '207', skipping spawn
2025-10-23 19:21:28,174 DEBUG CarlaAmbassador:1305 - SUMO vehicle 'veh_1' already exists in mapping with CARLA ID '208', skipping spawn
2025-10-23 19:21:28,175 DEBUG CarlaAmbassador:1459 - Successfully updated CARLA actor '207' (SUMO: 'veh_0') transform (speed: 0.0 m/s)
2025-10-23 19:21:28,175 DEBUG CarlaAmbassador:1459 - Successfully updated CARLA actor '208' (SUMO: 'veh_1') transform (speed: 0.0 m/s)
2025-10-23 19:21:28,176 DEBUG CarlaXmlRpcClient:671 - Excluding SUMO-managed actor '207' from getAllActors result
2025-10-23 19:21:28,176 DEBUG CarlaXmlRpcClient:671 - Excluding SUMO-managed actor '208' from getAllActors result
2025-10-23 19:21:28,176 DEBUG CarlaXmlRpcClient:675 - getAllActorsExcludingSumo: 3 total actors, 1 after filtering SUMO vehicles
2025-10-23 19:21:28,176 DEBUG CarlaXmlRpcClient:1080 - getActorChanges: Retrieved 1 current actors (excluding SUMO-managed)
2025-10-23 19:21:28,176 DEBUG CarlaXmlRpcClient:1120 - getActorChanges: Found added=0, updated=1, removed=0
2025-10-23 19:21:28,176 INFO  CarlaXmlRpcClient:1122 - Actor changes: added=0, updated=1, removed=0
2025-10-23 19:21:28,176 INFO  CarlaAmbassador:474 - EXTERNAL VEHICLE DETECTION: Detected changes - Added: 0, Updated: 1, Removed: 0
2025-10-23 19:21:28,176 INFO  CarlaAmbassador:476 - SUMO->CARLA MAPPING: Currently tracking 2 SUMO vehicles
2025-10-23 19:21:28,176 INFO  CarlaAmbassador:487 - EXTERNAL VEHICLE UPDATED: Actor ID=209, Info={type=vehicle.tesla.model3, transform={rotation=[Ljava.lang.Object;@42b28ff1, location=[Ljava.lang.Object;@36061cf3}, id=209}
2025-10-23 19:21:28,176 DEBUG CarlaAmbassador:848 - Converting CARLA actor '209' with data: {type=vehicle.tesla.model3, transform={rotation=[Ljava.lang.Object;@42b28ff1, location=[Ljava.lang.Object;@36061cf3}, id=209}
2025-10-23 19:21:28,176 DEBUG CarlaAmbassador:1029 - Converting CARLA position to SUMO: carlaX=316.83428955078125, carlaY=-171.99996948242188, yawDeg=3.005343023687601E-4, extentX=null
2025-10-23 19:21:28,176 DEBUG CarlaAmbassador:1051 - Applied netOffset: offsetX=503.02, offsetY=423.76, xWithOffset=819.8542895507812, yWithOffset=-595.7599694824219
2025-10-23 19:21:28,176 DEBUG CarlaAmbassador:1059 - Final SUMO coordinates: sumoX=819.8542895507812, sumoY=595.7599694824219, sumoZ=0.03233981877565384


Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 598.99.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 598.75.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 598.50.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 598.27.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 598.02.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 597.80.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 597.54.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 597.32.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 597.07.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 596.82.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 596.60.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 596.35.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 596.12.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 595.88.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 595.64.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 595.41.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 595.18.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 594.94.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 594.71.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 594.47.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 594.23.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 593.98.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 593.74.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 593.51.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 593.51.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 593.51.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 593.51.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 593.51.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 593.51.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 593.51.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 593.51.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 593.51.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 593.51.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 593.51.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 593.51.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 593.51.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 593.51.
Error: Answered with error to command 0xc4: Could not map vehicle '209', distance to road is 593.51.

2025-10-23 19:21:25,581 TRACE AbstractSumoAmbassador:1259 - Simulate traffic until 31500000000
2025-10-23 19:21:25,581 TRACE TraciVehicleFacade:506 - Trying to move vehicle 209 to position CartesianPoint{x=308.41,y=172.00,z=0.00} with angle 90.00122618896421 and mode KEEP_ROUTE
2025-10-23 19:21:25,581 WARN  AbstractSumoAmbassador:1380 - Could not set position of vehicle 209
org.eclipse.mosaic.rti.api.InternalFederateException: Could not move vehicle 209
	at org.eclipse.mosaic.fed.sumo.traci.facades.TraciVehicleFacade.moveToXY(TraciVehicleFacade.java:509)
	at org.eclipse.mosaic.fed.sumo.ambassador.AbstractSumoAmbassador.setExternalVehiclesToLatestPositions(AbstractSumoAmbassador.java:1376)
	at org.eclipse.mosaic.fed.sumo.ambassador.AbstractSumoAmbassador.processTimeAdvanceGrant(AbstractSumoAmbassador.java:1270)
	at org.eclipse.mosaic.rti.api.AbstractFederateAmbassador.advanceTime(AbstractFederateAmbassador.java:111)
	at org.eclipse.mosaic.rti.time.SequentialTimeManagement.runSimulation(SequentialTimeManagement.java:122)
	at org.eclipse.mosaic.starter.MosaicSimulation.runSimulation(MosaicSimulation.java:191)
	at org.eclipse.mosaic.starter.MosaicStarter.execute(MosaicStarter.java:135)
	at org.eclipse.mosaic.starter.MosaicStarter.main(MosaicStarter.java:77)
Caused by: org.eclipse.mosaic.fed.sumo.traci.TraciCommandException: TraCI Command failed: Could not map vehicle '209', distance to road is 626.53.
	at org.eclipse.mosaic.fed.sumo.traci.AbstractTraciCommand.readResults(AbstractTraciCommand.java:274)
	at org.eclipse.mosaic.fed.sumo.traci.AbstractTraciCommand.execute(AbstractTraciCommand.java:108)
	at org.eclipse.mosaic.fed.sumo.traci.commands.VehicleSetMoveToXY.execute(VehicleSetMoveToXY.java:86)
	at org.eclipse.mosaic.fed.sumo.traci.facades.TraciVehicleFacade.moveToXY(TraciVehicleFacade.java:507)
	... 7 common frames omitted
```bash
docker run --rm -it --gpus all --net=host -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 --user=carma usdotfhwastol/cdasim:latest /bin/bash
cd bridge
python carla_v0.9_xmlrpc_server.py --debug
python manual_control.py
./mosaic.sh -s Town04
```