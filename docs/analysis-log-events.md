# CDAS analysis log events

Logs consumed by the CDASim data-analysis tools use this stable format:

```text
CDAS_EVENT event=<event_name> key=value key=value
```

Event names and field names use lower-case `snake_case`. Scalar field values do
not contain whitespace. Collection values, such as a CARLA location, retain
their normal Java representation and are parsed only by the event that owns
them. Logger timestamps, levels, and class names may appear before
`CDAS_EVENT`; analyzers must not depend on Java source line numbers.

The prefix makes the events easy to inspect manually:

```bash
rg 'CDAS_EVENT' Carla.log
rg 'CDAS_EVENT event=carla_external_vehicle_added' Carla.log
rg 'CDAS_EVENT event=v2x_message_(inserted|received)' CommunicationDetails.log
```

## Event catalog

| Event | Important fields |
| --- | --- |
| `carla_xmlrpc_connected` | `connected` |
| `carla_actor_connection_status` | `mode`, `connected` |
| `carla_spawn_request` | `actor_type`, `actor_id`, `location`, `rotation`, `attributes` |
| `carla_spawn_result` | `actor_id`, `carla_id`, `result_type`, `accepted` |
| `carla_actor_detected` | `actor_id` |
| `carla_external_vehicle_added` | `actor_id`, `info` |
| `carla_sumo_vehicle_spawned` | `vehicle_id`, `carla_id`, `x`, `y`, `yaw`, `speed` |
| `carla_vehicle_assignment_published` | `actor_id`, `vehicle_type` |
| `carla_vehicle_updates_published` | `added`, `updated`, `removed` |
| `carla_vehicle_updates_received` | `time_ns`, `added`, `updated`, `removed` |
| `carla_next_timestep` | `time_ns` |
| `carla_traffic_light_updates_received` | `time_ns` |
| `carla_vehicle_sync_started` | `added`, `updated`, `removed` |
| `carla_vehicle_sync_skipped` | `reason`, `multi_client`, `single_client` |
| `carla_spawn_skipped` | `vehicle_id`, `carla_id`, `reason` |
| `carla_actor_updated` | `carla_id`, `vehicle_id`, `speed_mps` |
| `sumo_connection_established` | `host`, `port` |
| `sumo_connection_retry` | `host`, `port`, `attempts_remaining` |
| `sumo_api_version` | `api_version`, `sumo_version` |
| `sumo_interaction_received` | `interaction_type`, `time_ns` |
| `sumo_simulation_time` | `time_ms`, `wall_time_ms`, `next_time_ns`, `ambassador_id` |
| `sumo_external_vehicle_ignored` | `vehicle_id`, `sender`, `reason` |
| `common_instance_received` | `instance_id` |
| `common_instance_registered` | `instance_id` |
| `common_registration_duplicate` | `instance_id` |
| `carma_instance_received` | `instance_id` |
| `carma_instance_registered` | `instance_id` |
| `common_time_sync_sent` | `instance_id`, `target`, `port`, `time_ns` |
| `v2x_message_inserted` | `message_id`, `sender`, `external_id`, `channel`, `time_ns` |
| `v2x_message_received` | `message_id`, `receiver`, `time_ns` |
| `v2x_reception_processing` | `receiver`, `message_id`, optional `sender` |
| `v2x_reception_forwarded` | `receiver`, `message_id`, `size_bytes` |
| `v2x_reception_ignored` | `receiver`, `message_id`, `sender`, `reason` |
| `v2x_receiver_started` | `port` |
| `federation_started` | `federation_id` |
| `federate_initializing` | `federate_id` |
| `federate_added` | `federate_id` |
| `mapping_no_spawners` | `configured` |
| `application_vehicle_ignored` | `action`, `reason` |

The Python analyzer accepts both these events and the previous prose messages
so historical scenario output remains analyzable.

`sumo_simulation_time` is currently emitted together with the legacy
`Simulation Time:` message because the CARMA analytics tooling still consumes
that older line.
