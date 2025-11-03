# CDASim Scenario Runner

The **CDASim Scenario Runner** automates execution of **multiple
simulation test cases** defined in `parameters.yaml`.\
It generates `sim_start.sh` / `sim_stop.sh`, launches scenarios, waits
for their runtime, and cleans everything --- all inside a single
isolated `tmp/` directory.


------------------------------------------------------------------------

## Prerequisites

``` bash
# Python packages
python3 -m pip install pyyaml jinja2

# Docker + Docker Compose
sudo apt-get install docker.io docker-compose
```

------------------------------------------------------------------------

## Project Structure

    ├── scenario_runner.py
    ├── scenario_generator.py
    └── config/
        └── parameters/
            └── parameters.yaml
        └── templates/
            ├── sim_start_template.sh.j2
            └── sim_stop_template.sh.j2

------------------------------------------------------------------------

## parameters.yaml -- Define Your Test Cases

``` yaml
test_cases:
  - label: intersection_basic
    runtime_seconds: 90
    env_settings:
      cdasim:
        PROJECT_NAME: cdasim
        RUNTIME_IMAGE_ORG: usdotfhwastol
        RUNTIME_IMAGE_TAG: carma-system-4.10.0
        CONFIG_IMAGE_FULL: usdotfhwastol/cdasim-config:carma-system-4.10.0
        settings:
          MAP: Town04

      vehicles:
        - PROJECT_NAME: vehicle1
          RUNTIME_IMAGE_ORG: usdotfhwastol
          RUNTIME_IMAGE_TAG: carma-system-4.11.0-23-g06a1723-development
          CONFIG_IMAGE_FULL: usdotfhwastol/carma-config:carma-system-4.11.0-23-g06a1723-development
          settings:
            ROUTE: north_route
            START_TIME: 12
            SPAWN_POSITION: {x: 10.0, y: 20.0, z: 0.0, yaw: 90.0}

      streets:
        - PROJECT_NAME: street1
          RUNTIME_IMAGE_ORG: usdotfhwastol
          RUNTIME_IMAGE_TAG: carma-system-4.10.0
          CONFIG_IMAGE_FULL: usdotfhwastol/street-config:carma-system-4.10.0
          settings:
            EVC: {enable: true, snmp_port: 5001}

  - label: highway_merge
    runtime_seconds: 180
    env_settings:
      cdasim:
        PROJECT_NAME: cdasim
        RUNTIME_IMAGE_ORG: usdotfhwastol
        RUNTIME_IMAGE_TAG: carma-system-4.10.0
        CONFIG_IMAGE_FULL: usdotfhwastol/cdasim-config:carma-system-4.10.0
        settings:
          MAP: Highway

      vehicles:
        - PROJECT_NAME: ego_vehicle
          RUNTIME_IMAGE_ORG: usdotfhwastol
          RUNTIME_IMAGE_TAG: carma-system-4.11.0-23-g06a1723-development
          CONFIG_IMAGE_FULL: usdotfhwastol/carma-config:carma-system-4.11.0-23-g06a1723-development
          settings:
            ROUTE: merge_lane
            START_TIME: 5
            SPAWN_POSITION: {x: 0.0, y: 0.0, z: 0.0, yaw: 0.0}

      streets:
        - PROJECT_NAME: ramp_sensor
          RUNTIME_IMAGE_ORG: usdotfhwastol
          RUNTIME_IMAGE_TAG: carma-system-4.10.0
          CONFIG_IMAGE_FULL: usdotfhwastol/street-config:carma-system-4.10.0
          settings:
            SENSORS:
              enable: true
              sensorId: "ramp_lidar"
              type: "SemanticLidarSensor"
            SPAWN_POSITION: {x: -20.0, y: 10.0, z: 0.0, yaw: 45.0}
```

------------------------------------------------------------------------

## Usage

### Run All Test Cases

``` bash
python3 scenario_runner.py
```

------------------------------------------------------------------------

## What Happens

1.  Creates `tmp/`
2.  Generates required scripts + configs
3.  Runs start script
4.  Waits for runtime
5.  Runs stop script
6.  Cleans `tmp/`

------------------------------------------------------------------------

## Example Output

    === Scenario 1: intersection_basic ===
    Generated tmp/parameter.yaml
    Generated tmp/sim_start.sh
    Generated tmp/sim_stop.sh
    Launching: tmp/sim_start.sh
    Running for 90 seconds...
    Stopping: tmp/sim_stop.sh
    Cleaned tmp

------------------------------------------------------------------------


