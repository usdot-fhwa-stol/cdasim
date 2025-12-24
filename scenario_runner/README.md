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

### Components

| Component | Role |
|------------|------|
| **ScenarioRunner** | Main orchestrator. Loads scenarios from YAML, runs them sequentially, and manages the generator and collector. |
| **ScenarioGenerator** | Builds `.env` files, extracts `docker-compose.yml` from configuration images, and creates shell scripts for start/stop. |
| **DataCollector** | Collects simulation output data and organizes it by test case. |
| **sim_start.sh / sim_stop.sh** | Generated shell scripts used to bring containers up and down. |
| **parameters.yaml** | Central configuration file defining test cases, components, images, and data output rules. |

---

## Execution Flow

1. **Load Configurations**
   - `ScenarioRunner` reads `parameters.yaml` and loads all defined test cases.
   - Each test case includes runtime duration, environment settings, and output configuration.

2. **Generate Scenario Environment**
   - For each test case, `ScenarioRunner` writes a temporary `parameter.yaml`.
   - This is passed to `ScenarioGenerator`, which dynamically generates:
     - `.env` files for each component (`cdasim`, vehicles, streets)
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

Each test case in `parameters.yaml` defines simulation settings under `env_settings`, with the following key fields for every component (cdasim, vehicle, street):

| Field | Description |
|--------|--------------|
| **`PROJECT_NAME`** | A unique name for this simulation component (used as the Docker project name). |
| **`RUNTIME_IMAGE_ORG`** | The Docker organization or namespace that owns the runtime images (e.g., `usdotfhwastol`). |
| **`RUNTIME_IMAGE_TAG`** | The version tag for the runtime image. Defines which CARMA or CDASim build version to execute. |
| **`CONFIG_IMAGE_FULL`** | The full image name (including tag) of the configuration image that contains the embedded `docker-compose.yml` used to define how the component runs. |
| **`settings`** | Component-specific runtime parameters such as route, map, sensors, and spawn positions. |

### How these image fields interact

- **`RUNTIME_IMAGE_ORG`** and **`RUNTIME_IMAGE_TAG`** are written into the generated `.env` file.  
  Docker Compose uses these environment variables to build full image paths dynamically during runtime, for example:  
  ```
  ${RUNTIME_IMAGE_ORG}/cdasim:${RUNTIME_IMAGE_TAG}
  ```

- **`CONFIG_IMAGE_FULL`** specifies a configuration image that contains the `docker-compose.yml` file.  
  The Scenario Generator temporarily runs this image, extracts the compose file from inside it, and uses it to define the simulation stack.

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
    ├── test_1/
    │   ├── mosaic_logs/
    │   └── rosbags/
    └── test_2/
```

---

## Key Features

- Dynamic `docker-compose.yml` extraction from configuration images  
- Environment-driven runtime substitution (`RUNTIME_IMAGE_ORG`, `RUNTIME_IMAGE_TAG`)  
- Automated start/stop and cleanup cycles  
- Integrated log collection per scenario  
- Reproducible multi-case testing workflow

---
