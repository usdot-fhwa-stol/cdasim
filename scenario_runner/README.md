# CDASim Automated Scenario Runner

The **CDASim Automated Scenario Runner** automates the end-to-end execution of multi-container simulation scenarios for CDASim, CARMA, and MOSAIC environments.  
It dynamically prepares Docker environments, launches simulations, and collects outputs — all defined through a single configuration file.

---

## Overview

This tool allows you to define and execute multiple simulation **test cases** in one run using `parameters.yaml`.  
Each test case defines its own runtime configuration, environment images, and data output structure.

The system automatically:
- Loads all defined scenarios
- Generates environment files and scripts dynamically
- Extracts `docker-compose.yml` files from configuration images
- Launches and stops simulation containers
- Collects simulation outputs (logs, rosbags, etc.)

---

## Architecture

### Runner components

| Component | Role |
|------------|------|
| **ScenarioRunner** | Main orchestrator. Loads scenarios from YAML, runs them sequentially, and manages the generator and collector. |
| **ScenarioTopologyAllocator** | Loads `network_topology_template.json` and assigns XIL networks, service hostnames, and required network-scoped DNS aliases. |
| **ScenarioGenerator** | Builds `.env` files, extracts `docker-compose.yml` from configuration images, and creates shell scripts for start/stop. |
| **DataCollector** | Collects simulation output data and organizes it by test case. |
| **sim_start.sh / sim_stop.sh** | Generated shell scripts used to bring containers up and down. |
| **parameters.yaml** | Central configuration file defining test cases, deployment entries, images, and data output rules. |

### Network topology template

`config/network_topology_template.json` describes only the network relationships
between Compose projects. Its top-level sections are:

| Section | Purpose |
|---------|---------|
| **`networks`** | Names the scenario-wide Docker networks, such as the simulation and cloud networks. |
| **`shared_services`** | Defines stable network hostnames for services shared across the scenario, such as CDASim and CARMA Cloud. |
| **`instance_topology_templates`** | Defines reusable network and endpoint patterns for repeatable Platform, Messenger, and Street instances. The allocator applies the appropriate template for each configured instance and substitutes its instance index where required. |

---

## Execution Flow

1. **Load Configurations**
   - `ScenarioRunner` reads `parameters.yaml` and loads all defined test cases.
   - Each test case includes runtime duration, environment settings, and output configuration.

2. **Generate Scenario Environment**
   - `ScenarioTopologyAllocator` loads `network_topology_template.json` and adds internal networks, Docker service hostnames, and only the instance-specific DNS aliases required to distinguish repeated endpoints. These values are not configured in `parameters.yaml`.
   - `ScenarioTopologyAllocator` applies the appropriate network template to each configured instance.
   - For each test case, `ScenarioRunner` writes a temporary `parameter.yaml`.
   - This is passed to `ScenarioGenerator`, which dynamically generates:
     - `.env` files for each Compose project (`cdasim`, CARMA Cloud, vehicles, streets)
     - Extracted `docker-compose.yml` files from the specified config images
     - Two shell scripts: `sim_start.sh` and `sim_stop.sh`

3. **Run Simulation**
   - The system executes `bash sim_start.sh` to bring up all simulation containers and networks.
   - Containers run concurrently for the duration defined by `runtime_seconds`.
   - Once the time elapses, or if a timeout occurs, `bash sim_stop.sh` is executed to gracefully stop the environment.

4. **Data Collection**
   - `DataCollector` gathers logs from `/opt/carma-simulation/logs` (MOSAIC) and `/opt/carma/logs` (CARMA/ROS).
   - (Optional) Also gathers v2xhub, carma messenger and carma cloud logs
   - These outputs are copied to the scenario’s result directory defined in the YAML (`data_output.output_directory`).

5. **Cleanup**
   - After every test, temporary files and directories are deleted.
   - The runner moves on to the next test case automatically.

---

## Key YAML Fields

Scenario Runner supports ROS 2 deployments only. Each item under
`env_settings` describes a Compose deployment entry. The following fields
configure those entries:

| Field | Description |
|--------|--------------|
| **`PROJECT_NAME`** | A unique Docker Compose project name for this deployment entry. |
| **`COMPONENT`** | Vehicle entries only: selects the `platform` or `messenger` instance topology template. The default is `platform`. Do not set this field on Street entries. |
| **`RUNTIME_IMAGE_ORG`** | The Docker organization or namespace that owns the runtime images (e.g., `usdotfhwastol`). |
| **`RUNTIME_IMAGE_TAG`** | The version tag for the runtime image. Defines which CARMA or CDASim build version to execute. |
| **`CONFIG_IMAGE_FULL`** | The full image name (including tag) of the configuration image containing the deployment's embedded `docker-compose.yml`. |
| **`COMPOSE_FILE`** | A repository-local base Compose file used instead of `CONFIG_IMAGE_FULL`. |
| **`settings`** | Deployment-specific runtime parameters such as route, map, sensors, and spawn positions. |

### How these image fields interact

- **`RUNTIME_IMAGE_ORG`** and **`RUNTIME_IMAGE_TAG`** are written into the generated `.env` file.  
  Docker Compose uses these environment variables to build full image paths dynamically during runtime, for example:  
  ```
  ${RUNTIME_IMAGE_ORG}/cdasim:${RUNTIME_IMAGE_TAG}
  ```

- **`CONFIG_IMAGE_FULL`** specifies a configuration image that contains the `docker-compose.yml` file.  
  The Scenario Generator temporarily runs this image, extracts the Compose file from inside it, and uses it to define the simulation stack.

When an extracted legacy Compose file uses instance suffixes such as
`platform_ros2_1`, Scenario Generator removes the suffix and updates its direct
`depends_on`, `network_mode`, and `volumes_from` service references before
applying the local overrides. This compatibility step is limited to extracted
Compose structure; topology endpoint names remain canonical.

Together, they decouple **what image to run** from **how it is configured**, providing flexible version control and easier upgrades.

---

## Usage

### Prerequisites
```bash
sudo apt install docker.io docker-compose python3-pip
pip install pyyaml jinja2
```

### Run all scenarios
```bash
python3 scenario_runner.py
```

### Generate files without running the scenario
```bash
python3 scenario_runner.py --generate-only
```

This generates the environment files, runtime Compose overrides, and matching
`sim_start.sh` and `sim_stop.sh` files. It does not execute either script, wait
for `runtime_seconds`, collect runtime data, or remove `tmp/` afterward.

### Output example
```
=== Scenario 1: intersection_basic ===
Generated tmp/.env files
Extracted docker-compose.yml
Launching sim_start.sh
Running for 90 seconds...
Stopping sim_stop.sh
Collecting logs...
Cleaned tmp/
Scenario 1 complete.
```

---

## Output Directory Structure

```
project_root/
├── config/
│   ├── network_topology_template.json
│   ├── templates/
│   │   ├── sim_start_template.sh.j2
│   │   └── sim_stop_template.sh.j2
│   └── parameters/
│       └── parameters.yaml
├── scenario_runner.py
├── scenario_generator.py
├── data_collector.py
├── tmp/                # temporary environment files
└── results/
    ├── town10_two_vehicle_xil/
    │   ├── mosaic_logs/
    │   └── rosbags/
    └── another_scenario_label/
```

---

## Key Features

- Dynamic `docker-compose.yml` extraction from configuration images  
- Environment-driven runtime substitution (`RUNTIME_IMAGE_ORG`, `RUNTIME_IMAGE_TAG`)  
- Automated start/stop and cleanup cycles  
- Integrated log collection per scenario  
- Reproducible multi-case testing workflow

---

## Compose Override Support

Each deployment entry can use a configuration image through `CONFIG_IMAGE_FULL` or a
repository Compose file through `COMPOSE_FILE`. Files listed in
`COMPOSE_OVERRIDES` are applied after the base Compose file, followed by the
Scenario Runner managed override.

The `carma_cloud` deployment entry follows the same repository-local model as CARMA
Street. Clone `carma-cloud` beside `cdasim`, point `COMPOSE_FILE` to its base
`docker-compose.yml`, and retain the Runner-owned
`config/compose/carma-cloud.cdasim.yml` internal override. To use a CARMA Cloud
configuration image later, replace `COMPOSE_FILE` with `CONFIG_IMAGE_FULL` and
`CONFIG_COMPOSE_PATH`; the same override can still be applied.

Scenario Runner creates the shared XIL networks, starts instance-specific
configuration containers for BusyBox configuration images, and then launches
all Compose projects. The configuration containers remain available so CARMA
services can read files such as `VehicleConfigParams.yaml` through
`volumes_from`. Scenario Runner recursively copies the complete configuration
tree into each config volume, including configuration subdirectories.
