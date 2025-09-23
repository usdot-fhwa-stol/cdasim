# CDASim Scenario Generator

The CDASim Scenario Generator automates Docker Compose deployments for CDASim, CARMA Streets, and vehicle services in simulation runs. The scenario_generator.py script reads a config.yaml file to create a sim_launch.sh bash script and .env files, streamlining setup.
Installation
# Prerequisites
* Python 3.x with required packages:
```sh
python3 -m pip install pyyaml jinja2
```

* Docker and Docker Compose installed.

* Pre-existing docker-compose.yml files in:
  * **`cdasim_config_dir`** (e.g., **`/path/to/cdasim-config`**)
  * **`street_config_dir`** (e.g., **`/path/to/carma-street`**)
  * **`carma_config_dir`** (e.g., **`/path/to/carma-config/basic_sim_vehicle`**)



# Setup Files

**1.** Place the following files in the same directory:
* **`generate_scenario.py`**
* **`sim_launch_template.sh.j2`**
* **`config.yaml`**



**2.** Configure **`config.yaml`**

* Edit **`config.yaml`** to specify directory paths and environment settings. Example:
```yaml
cdasim_config_dir: /path/to/cdasim-config
street_config_dir: /path/to/carma-street
carma_config_dir: /path/to/carma-config/basic_sim_vehicle
env_settings:
  cdasim:
    DOCKER_ORG: usdotfhwastol
    DOCKER_TAG: carma-system-4.10.0
    PROJECT_NAME: cdasim
  vehicles:
    - PROJECT_NAME: vehicle1
      CARMA_CONFIG_PATH: /path/to/carma-config
      DOCKER_ORG: usdotfhwastol
      DOCKER_TAG: carma-system-4.10.0
    - PROJECT_NAME: vehicle2
      CARMA_CONFIG_PATH: /path/to/carma-config
      DOCKER_ORG: usdotfhwastol
      DOCKER_TAG: carma-system-4.10.0
  streets:
    - PROJECT_NAME: street1
      DOCKER_ORG: usdotfhwastol
      DOCKER_TAG: carma-system-4.10.0
    - PROJECT_NAME: street2
      DOCKER_ORG: usdotfhwastol
      DOCKER_TAG: carma-system-4.10.0
```

* Replace placeholder paths (e.g., /path/to/carma-config) with actual paths.

# Usage

**1.** Generate FilesRun the Python script to create the bash script and .env files:
```sh
python generate_scenario.py
```
* **`sim_launch.sh`** in the current directory
* **`.env.cdasim`** in **`cdasim_config_dir`**
* **`.env.vehicle_1`**, **`.env.vehicle_2`**, etc., in **`carma_config_dir`**
* **`.env.street_1`**, **`.env.street_2`**, etc., in **`street_config_dir`**


**2.** Run the Bash Script

Execute the generated script:
```sh
./sim_launch.sh
```
