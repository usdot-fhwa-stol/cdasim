#  Copyright (C) 2025 LEIDOS.
#
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
#
#  http://www.apache.org/licenses/LICENSE-2.0

from copy import deepcopy
import json
from pathlib import Path
from typing import Any, Dict, Mapping, Optional


TOPOLOGY_CONFIG_PATH = (
    Path(__file__).resolve().parent
    / "config"
    / "network_topology_template.json"
)
DEFAULT_DATA_OUTPUT = {
    "output_directory": "/opt/carma-simulation/tests/output/scenario_runner",
    "collect": {
        "mosaic_logs": "/opt/carma-simulation/logs",
        "rosbags": "/opt/carma/logs",
        "v2xhub_logs": "/tmp/cdasim-scenario-runner",
        "carmacloud_logs": "/opt/carma/logs/carmacloud",
    },
}


def load_topology_config(path: Path = TOPOLOGY_CONFIG_PATH) -> Dict[str, Any]:
    """Load the repository-controlled topology file."""

    with Path(path).open("r", encoding="utf-8") as topology_file:
        topology = json.load(topology_file)

    required = {
        "networks",
        "shared_services",
        "instance_topology_templates",
    }
    missing = required - topology.keys()
    if missing:
        raise ValueError(f"Missing topology sections: {', '.join(sorted(missing))}")
    return topology


class ScenarioTopologyAllocator:
    """Resolve network values for repeatable scenario instances."""

    def __init__(
        self,
        topology: Optional[Mapping[str, Any]] = None,
    ):
        self.config = (
            deepcopy(topology) if topology is not None else load_topology_config()
        )

        self.networks = [
            {
                "name": network["name"],
                "driver": "bridge",
            }
            for network in self.config["networks"].values()
        ]
        self.core = self._shared_service_allocations()

    def _shared_service_allocations(self) -> Dict[str, str]:
        simulation = self.config["networks"]["simulation"]
        cloud = self.config["networks"]["cloud"]
        result = {
            "SIM_NETWORK_NAME": simulation["name"],
            "CLOUD_NETWORK_NAME": cloud["name"],
        }

        for service in self.config["shared_services"].values():
            for interface in service["interfaces"]:
                result[interface["env"]] = interface["host"]
        return result

    def _private_network(
        self, template: Mapping[str, Any], index: int
    ) -> Dict[str, str]:
        network = {
            "name": template["network_name_template"].format(index=index),
        }
        self.networks.append({"name": network["name"], "driver": "bridge"})
        return network

    @staticmethod
    def _endpoint_hosts(
        endpoints: Mapping[str, Any],
        index: int,
        conditions: Optional[Mapping[str, bool]] = None,
    ) -> Dict[str, str]:
        result = {}
        conditions = conditions or {}
        for endpoint in endpoints.values():
            condition = endpoint.get("conditional")
            if condition and not conditions.get(condition, False):
                continue
            if "alias_template" in endpoint:
                host = endpoint["alias_template"].format(index=index)
            else:
                host = endpoint["host"]
            result[endpoint["env"]] = host
        return result

    def _allocate_instance(self, name: str, index: int) -> Dict[str, str]:
        template = self.config["instance_topology_templates"][name]
        network = self._private_network(template, index)
        return {
            "PRIVATE_NETWORK_NAME": network["name"],
            **self._endpoint_hosts(template["private_endpoints"], index),
            **self._endpoint_hosts(template["simulation_endpoints"], index),
        }

    def allocate_vehicle(self, index: int) -> Dict[str, str]:
        """Allocate one ROS 2 Platform vehicle."""

        return self._allocate_instance("platform", index)

    def allocate_messenger(self, index: int) -> Dict[str, str]:
        """Allocate one ROS 2 Messenger vehicle."""

        return self._allocate_instance("messenger", index)

    def allocate_street(self, index: int, evc_enabled: bool) -> Dict[str, str]:
        """Allocate one Street/V2X Hub instance."""

        template = self.config["instance_topology_templates"]["street"]
        network = self._private_network(template, index)
        conditions = {"evc_enabled": evc_enabled}
        return {
            "PRIVATE_NETWORK_NAME": network["name"],
            **self._endpoint_hosts(
                template["private_endpoints"], index, conditions
            ),
            **self._endpoint_hosts(
                template["simulation_endpoints"], index, conditions
            ),
        }


def _data_output(configured):
    result = deepcopy(DEFAULT_DATA_OUTPUT)
    if configured:
        result.update(
            {key: value for key, value in configured.items() if key != "collect"}
        )
        result["collect"].update(configured.get("collect", {}))
    return result


def _spawn_point(settings):
    spawn = settings.get("SPAWN_POINT")
    if isinstance(spawn, dict):
        settings["SPAWN_POINT"] = ",".join(
            str(spawn[key]) for key in ("x", "y", "z", "roll", "pitch", "yaw")
        )


def apply_scenario_topology(
    case: Dict[str, Any], topology_path: Path = TOPOLOGY_CONFIG_PATH
) -> Dict[str, Any]:
    """Add ROS 2 topology allocations to one scenario."""

    result = deepcopy(case)
    env_settings = result["env_settings"]
    vehicles = env_settings.get("vehicles", [])
    data_output = _data_output(result.get("data_output"))
    result["data_output"] = data_output
    topology = ScenarioTopologyAllocator(load_topology_config(topology_path))

    vehicle_indexes = {"platform": 0, "messenger": 0}
    for vehicle in vehicles:
        component = vehicle.get("COMPONENT", "platform")
        if component not in vehicle_indexes:
            raise ValueError(f"Unknown vehicle component: {component}")
        vehicle_indexes[component] += 1
        index = vehicle_indexes[component]
        settings = vehicle["settings"]
        _spawn_point(settings)
        allocation = (
            topology.allocate_vehicle(index)
            if component == "platform"
            else topology.allocate_messenger(index)
        )
        settings.update(
            {
                "ROLE_NAME": settings["VEHICLE_ID"],
                "CARMA_LOG_ROOT": data_output["collect"]["rosbags"],
                "CONFIG_CONTAINER_NAME": f"{vehicle['PROJECT_NAME']}-config",
                "DOCKER_ORG": "scenario-runner-placeholder",
                "DOCKER_TAG": "scenario-runner-placeholder",
                "CARLA_HOST": "carla-server",
                "SIM_NETWORK_NAME": topology.core["SIM_NETWORK_NAME"],
                "CDASIM_SIM_HOST": topology.core["CDASIM_SIM_HOST"],
                **allocation,
            }
        )
        if component == "messenger":
            messenger_root = f"/opt/carma-messenger/{settings['VEHICLE_ID']}"
            settings.setdefault(
                "MESSENGER_LOG_ROOT",
                f"{data_output['collect']['rosbags'].rstrip('/')}/"
                f"{settings['VEHICLE_ID']}",
            )
            settings.setdefault("MESSENGER_ROS_ROOT", f"{messenger_root}/.ros")
            settings.setdefault(
                "MESSENGER_ROUTE_ROOT", f"{messenger_root}/routes"
            )
            settings.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    for index, street in enumerate(env_settings.get("streets", []), 1):
        settings = street["settings"]
        allocation = topology.allocate_street(
            index, bool((settings.get("EVC") or {}).get("enable", False))
        )
        settings.update(
            {
                "STREET_ID": f"street_{index}",
                "SIMULATION_MODE": True,
                "SIMULATION_HOST": topology.core["CDASIM_SIM_HOST"],
                "SIMULATION_IP": topology.core["CDASIM_SIM_HOST"],
                "SIMULATION_REGISTRATION_PORT": 1615,
                "TIME_SYNC_PORT": 7575,
                "SIM_V2X_PORT": 1517,
                "SIM_INTERACTION_PORT": 7576,
                "V2X_PORT": 8686,
                "V2XHUB_LOG_ROOT": data_output["collect"]["v2xhub_logs"],
                "SIM_NETWORK_NAME": topology.core["SIM_NETWORK_NAME"],
                "INFRASTRUCTURE_HOST": allocation[
                    "STREET_INFRASTRUCTURE_HOST"
                ],
                "V2XHUB_HOST": allocation["V2XHUB_SIM_HOST"],
                "INFRASTRUCTURE_IP": allocation[
                    "STREET_INFRASTRUCTURE_HOST"
                ],
                "V2XHUB_IP": allocation["V2XHUB_SIM_HOST"],
                **allocation,
            }
        )

    cdasim = env_settings["cdasim"]
    cdasim["settings"].update(
        {
            "CDASIM_LOG_ROOT": data_output["collect"]["mosaic_logs"],
            **topology.core,
        }
    )

    carma_cloud = env_settings.get("carma_cloud")
    if carma_cloud:
        cloud_settings = carma_cloud.setdefault("settings", {})
        cloud_settings.setdefault(
            "CARMA_CLOUD_WORK_ROOT",
            "/opt/carma-simulation/carma-cloud/work",
        )
        cloud_settings.update(
            {
                "CARMA_CLOUD_LOG_ROOT": data_output["collect"][
                    "carmacloud_logs"
                ],
                **topology.core,
            }
        )

    env_settings["runner_networks"] = topology.networks
    return result
