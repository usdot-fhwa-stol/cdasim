# CARLA ScenarioRunner scenarios

This directory contains custom [ScenarioRunner][scenario_runner_docs_link]
scenario configurations to facilitate integration testing. This still a work in
progress, so the scenarios serve more as example references.

[scenario_runner_docs_link]: https://carla-scenariorunner.readthedocs.io/en/latest/

## Getting started

ScenarioRunner's code resides in the `srunner` Python package, and the
`scenario_runner.py` script is responsible for launching scenarios. If we want
to use ScenarioRunner for custom CARMA scenarios, we have to overcome two
technical hurdles:

1. the `srunner` package is unavailable in [PyPi][pypi_link];
2. we have to embed our scenario definitions and configurations within
   `srunner`; and
3. ScenarioRunner uses on the [CARLA Python API][carla_python_api_link]
   internally, which we have to install separately.

You have three options for getting ScenarioRunner working with the custom
CARMA scenarios, each described below.

[pypi_link]: https://pypi.org
[carla_python_api_link]: https://carla.readthedocs.io/en/latest/python_api/

### Manual installation

Check out the official documentation on how to install ScenarioRunner, the
CARLA Python client library. Their instructions also show how to add custom
scenario definitions (in the `scenarios/` directory) and configurations (in the
`examples/` directory) to the installation.

### Convenience script

You can use the convenience script (`install_carma_scenario_runner`) provided
in this directory to download and install a specific ScenarioRunner version to
a desired location. The script will also replace the provided scenarios with
the ones specific to CARMA.

The example below shows how you can install ScenarioRunner version 0.9.10 to
the `/opt/scenario_runner` directory:

```shell
$ ./install_carma_scenario_runner --prefix /opt 0.9.10
```

> [!IMPORTANT]\
> This script does not install the CARLA Python client library, so you will
> still have to do that separately. It also does not modify the `PYTHONPATH`
> environment variable, so double check that Python can find the `srunner`
> package after you install it.

### Docker (recommended)

You can build a Docker image that packages ScenarioRunner, the CARMA scenarios,
and the CARLA Python API. The `Dockerfile` included in this directory handles
all the required installation steps.

After building the image, you can run ScenarioRunner as shown in the below
example:

```shell
$ docker run --rm -it usdotfhwastol/carma-scenario-runner:0.9.10 \
    --scenario FollowLeadingVehicle_1 --reloadWorld
```

The container automatically invokes the `scenario_runner.py` script, so you
need only to specify the desired scenario.
