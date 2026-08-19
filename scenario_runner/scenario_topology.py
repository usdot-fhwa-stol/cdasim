#  Copyright (C) 2025 LEIDOS.
#
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
#
#  http://www.apache.org/licenses/LICENSE-2.0

"""Allocate Scenario Runner network settings from topology.json.

The JSON file owns ROS 2 XIL network relationships, endpoint host slots, and
legacy preferred addresses. Python tracks addresses used by the current
scenario and generates the required env values.
"""

from copy import deepcopy
import json
from pathlib import Path
from typing import Any, Dict, Mapping, Optional


TOPOLOGY_CONFIG_PATH = (
    Path(__file__).resolve().parent / "config" / "topology.json"
)
SUPPORTED_ARCHITECTURE = "ros2"
DEFAULT_MESSENGER_SERVICES = [
    "messenger_ros2",
    "carma_messenger_bridge",
    "messenger_v2x_ros_driver",
    "messenger_vehicle_registration_service",
    "carma-messenger-vehicle-plugin",
]
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
        "allocation",
        "shared_services",
        "components",
    }
    missing = required - topology.keys()
    if missing:
        raise ValueError(f"Missing topology sections: {', '.join(sorted(missing))}")
    return topology


class ScenarioTopologyAllocator:
    """Allocate unique XIL private octets and shared simulation hosts."""

    def __init__(
        self,
        topology: Optional[Mapping[str, Any]] = None,
    ):
        self.config = (
            deepcopy(topology) if topology is not None else load_topology_config()
        )

        allocation = self.config["allocation"]
        self.private_pool = range(
            allocation["private_octet_pool"]["start"],
            allocation["private_octet_pool"]["end"] + 1,
        )
        self.simulation_pool = range(
            allocation["simulation_host_pool"]["start"],
            allocation["simulation_host_pool"]["end"] + 1,
        )
        self.reserved_private_octets = set(
            allocation["reserved_private_octets"]
        )
        self.reserved_sim_hosts = set(allocation["reserved_simulation_hosts"])
        self.used_private_octets = {
            network["fixed_octet"] for network in self.config["networks"].values()
        }
        self.used_sim_hosts = set()
        self.networks = [
            {
                "name": network["name"],
                "driver": "bridge",
                "subnet": network["subnet"],
            }
            for network in self.config["networks"].values()
        ]
        self.core = self._shared_service_allocations()

    @staticmethod
    def _address(subnet: str, host: int) -> str:
        """Build a known XIL 172.X.0.HOST address from its /16 subnet."""

        prefix = subnet.split("/", 1)[0].rsplit(".", 1)[0]
        return f"{prefix}.{host}"

    def _shared_service_allocations(self) -> Dict[str, str]:
        simulation = self.config["networks"]["simulation"]
        cloud = self.config["networks"]["cloud"]
        result = {
            "SIM_NETWORK_NAME": simulation["name"],
            "SIM_SUBNET": simulation["subnet"],
            "CLOUD_NETWORK_NAME": cloud["name"],
            "CLOUD_SUBNET": cloud["subnet"],
        }

        for service in self.config["shared_services"].values():
            for interface in service["interfaces"]:
                network_key = interface["network"]
                host = interface["host"]
                if network_key == "simulation":
                    if host in self.used_sim_hosts:
                        raise ValueError(f"Duplicate fixed simulation host: {host}")
                    self.used_sim_hosts.add(host)
                network = self.config["networks"][network_key]
                result[interface["env"]] = self._address(network["subnet"], host)
        return result

    def _allocate_private_octet(self, preferred: Optional[int]) -> int:
        if preferred is not None and preferred not in self.used_private_octets:
            self.used_private_octets.add(preferred)
            return preferred

        for octet in self.private_pool:
            if (
                octet not in self.used_private_octets
                and octet not in self.reserved_private_octets
            ):
                self.used_private_octets.add(octet)
                return octet
        raise RuntimeError("No unused private network octet is available")

    def _allocate_sim_host(self, preferred: Optional[int]) -> int:
        if preferred is not None and preferred not in self.used_sim_hosts:
            self.used_sim_hosts.add(preferred)
            return preferred

        for host in self.simulation_pool:
            if host not in self.used_sim_hosts and host not in self.reserved_sim_hosts:
                self.used_sim_hosts.add(host)
                return host
        raise RuntimeError("No unused simulation network host is available")

    def _private_network(
        self, component: Mapping[str, Any], index: int
    ) -> Dict[str, str]:
        preferred = component.get("preferred_private_octets", {}).get(str(index))
        octet = self._allocate_private_octet(preferred)
        network = {
            "name": component["network_name_template"].format(index=index),
            "subnet": component["subnet_template"].format(octet=octet),
        }
        self.networks.append(
            {"name": network["name"], "driver": "bridge", "subnet": network["subnet"]}
        )
        return network

    def _endpoint_allocations(
        self,
        endpoints: Mapping[str, Any],
        index: int,
        private_subnet: Optional[str] = None,
        conditions: Optional[Mapping[str, bool]] = None,
    ) -> Dict[str, str]:
        result = {}
        conditions = conditions or {}
        simulation_subnet = self.config["networks"]["simulation"]["subnet"]
        for endpoint in endpoints.values():
            condition = endpoint.get("conditional")
            if condition and not conditions.get(condition, False):
                continue
            if private_subnet:
                host = endpoint["host"]
                subnet = private_subnet
            else:
                preferred = endpoint.get("preferred_instance_hosts", {}).get(
                    str(index)
                )
                host = self._allocate_sim_host(preferred)
                subnet = simulation_subnet
            result[endpoint["env"]] = self._address(subnet, host)
        return result

    def _allocate_component(self, name: str, index: int) -> Dict[str, str]:
        component = self.config["components"][name]
        network = self._private_network(component, index)
        return {
            "PRIVATE_NETWORK_NAME": network["name"],
            "VEHICLE_SUBNET": network["subnet"],
            **self._endpoint_allocations(
                component["private_endpoints"], index, network["subnet"]
            ),
            **self._endpoint_allocations(
                component["simulation_endpoints"], index
            ),
        }

    def allocate_vehicle(self, index: int) -> Dict[str, str]:
        """Allocate one ROS 2 Platform vehicle."""

        return self._allocate_component("platform", index)

    def allocate_messenger(self, index: int) -> Dict[str, str]:
        """Allocate one ROS 2 Messenger vehicle."""

        return self._allocate_component("messenger", index)

    def allocate_street(self, index: int, evc_enabled: bool) -> Dict[str, str]:
        """Allocate one Street/V2X Hub instance."""

        street = self.config["components"]["street"]
        network = self._private_network(street, index)
        conditions = {"evc_enabled": evc_enabled}
        return {
            "PRIVATE_NETWORK_NAME": network["name"],
            "STREET_SUBNET": network["subnet"],
            **self._endpoint_allocations(
                street["private_endpoints"], index, network["subnet"], conditions
            ),
            **self._endpoint_allocations(
                street["simulation_endpoints"], index, conditions=conditions
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


def _validate_ros2_architecture(configured, owner):
    if configured not in (None, SUPPORTED_ARCHITECTURE):
        raise ValueError(
            f"Unsupported {owner} architecture {configured!r}; "
            "Scenario Runner supports ROS 2 only"
        )


def apply_scenario_topology(
    case: Dict[str, Any], topology_path: Path = TOPOLOGY_CONFIG_PATH
) -> Dict[str, Any]:
    """Add ROS 2 topology allocations to one scenario."""

    result = deepcopy(case)
    _validate_ros2_architecture(result.get("architecture"), "scenario")
    env_settings = result["env_settings"]
    vehicles = env_settings.get("vehicles", [])
    data_output = _data_output(result.get("data_output"))
    result["data_output"] = data_output
    topology = ScenarioTopologyAllocator(load_topology_config(topology_path))

    vehicle_indexes = {"platform": 0, "messenger": 0}
    for vehicle in vehicles:
        component = vehicle.get("COMPONENT", "platform")
        _validate_ros2_architecture(vehicle.get("architecture"), "vehicle")
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
                "CDASIM_SIM_IP": topology.core["CDASIM_SIM_IP"],
                **allocation,
            }
        )
        if component == "messenger":
            messenger_root = f"/opt/carma-messenger/{settings['VEHICLE_ID']}"
            vehicle["SERVICES"] = vehicle.get(
                "SERVICES", list(DEFAULT_MESSENGER_SERVICES)
            )
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
                "SIMULATION_IP": topology.core["CDASIM_SIM_IP"],
                "SIMULATION_REGISTRATION_PORT": 1615,
                "TIME_SYNC_PORT": 7575,
                "SIM_V2X_PORT": 1517,
                "SIM_INTERACTION_PORT": 7576,
                "V2X_PORT": 8686,
                "V2XHUB_LOG_ROOT": data_output["collect"]["v2xhub_logs"],
                "SIM_NETWORK_NAME": topology.core["SIM_NETWORK_NAME"],
                "INFRASTRUCTURE_IP": allocation["STREET_INFRASTRUCTURE_IP"],
                "V2XHUB_IP": allocation["V2XHUB_SIM_IP"],
                **allocation,
            }
        )

    cdasim = env_settings["cdasim"]
    default_services = [
        "cdasim", "carla-sensor-lib", "xml_rpc_server"
    ]
    cdasim["SERVICES"] = cdasim.get("SERVICES", default_services)
    cdasim["settings"].update(
        {
            "CDASIM_LOG_ROOT": data_output["collect"]["mosaic_logs"],
            **topology.core,
        }
    )

    carma_cloud = env_settings.get("carma_cloud")
    if carma_cloud:
        carma_cloud["SERVICES"] = carma_cloud.get(
            "SERVICES", ["carma-cloud"]
        )
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
