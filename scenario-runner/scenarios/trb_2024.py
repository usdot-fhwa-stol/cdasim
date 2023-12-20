# Copyright 2023 Leidos
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO

import carla

import py_trees

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (
    Idle,
    KeepVelocity,
    WaypointFollower,
)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (
    DriveDistance,
    InTriggerDistanceToVehicle,
)
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.scenarioconfigs.scenario_configuration import ScenarioConfiguration

"""
Configurations
"""

WALKING_PERSON_SPEED_IN_MS = 1.34
WALKING_PERSON_TRIGGER_WALKING_DISTANCE_IN_METERS = 20.0

class Trb2024(BasicScenario):
    def __init__(
        self,
        world,
        ego_vehicles,
        config: ScenarioConfiguration,
        randomize: bool = False,
        debug_mode: bool = False,
        criteria_enable: bool = True,
        timeout=60,
    ) -> None:
        """
        :param world: CARLA world in which the scenario is running
        :param list[carla.Vehicle] ego_vehicles: CARLA vehicle objects created based on scenario XML configuration
        :param ScenarioConfiguration config: Specifications from scenario XML configuration
        :param bool randomize:
        :param bool debug_mode:
        :param bool criteria_enable:
        :param float timeout: Threshold (in seconds) after which test automatically fails

        :return: None
        :rtype: None
        """
        # Must be defined before super() call because BasicScenario
        # references is in its __init__() function.
        self.timeout = timeout

        super(Trb2024, self).__init__(
            "Trb2024",
            ego_vehicles,
            config,
            world,
            debug_mode=debug_mode,
            criteria_enable=criteria_enable,
        )

        self.world_map = CarlaDataProvider.get_map()

        # This is where the action takes place
        spectator = world.get_spectator()
        spectator.set_transform(
            carla.Transform(
                carla.Location(266.4068, -160.1683, 16.2939),
                carla.Rotation(-34.0360, -126.9920, 0.0),
            )
        )

        # self.carma_vehicle = ego_vehicles[0]
        self.other_actors_dict = {}

    def _initialize_actors(self, config: ScenarioConfiguration) -> None:
        """
        Note: this function overrides the one in BasicScenario (parent
        class), so this override is responsible for adding the actors
        defined in the scenario XML configuration.

        :param ScenarioConfiguration config: Specifications from
        scenario XML configuration
        :return: None
        """
        actors = CarlaDataProvider.request_new_actors(config.other_actors)

        self.other_actors_dict = {
            actor_config.rolename: actor
            for actor_config, actor in zip(config.other_actors, actors)
        }

    def _setup_scenario_trigger(self, _: ScenarioConfiguration) -> None:
        """
        Set up the scenario start trigger

        Note: this function overrides the abstract one in the
        BasicScenario parent class. The base class's implementation adds
        a trigger that prevents the scenario from starting until the
        ego vehicle drives some distance. We don't want that trigger
        for this scenario because the ego vehicle will not move. Override this
        function to create custom scenario start triggers. Follow this link
        for more information:
        https://carla-scenariorunner.readthedocs.io/en/latest/creating_new_scenario/

        :return: None
        """
        pass

    def _create_behavior(self):
        """
        Setup the behavior for NewScenario

        Note: this function overrides the abstract one in the
        BasicScenario parent class.

        :return: Behavior tree root
        """
        carma_vehicle = None
        for actor in CarlaDataProvider.get_world().get_actors():
            if "role_name" in actor.attributes:
                print(f"ACTOR ROLE: {actor.attributes['role_name']}")

        for actor in CarlaDataProvider.get_world().get_actors():
            if (
                "role_name" in actor.attributes
                and actor.attributes["role_name"] == "carma_1"
            ):
                carma_vehicle = actor
                break

        CarlaDataProvider.register_actor(actor)
        crossing_person = self.other_actors_dict["crossing_person"]

        # start_condition = Idle(5, name="start_condition")
        start_condition = InTriggerDistanceToVehicle(
            crossing_person, carma_vehicle, WALKING_PERSON_TRIGGER_WALKING_DISTANCE_IN_METERS
        )

        walk_across_street = KeepVelocity(
            crossing_person, WALKING_PERSON_SPEED_IN_MS, 10.0, name="walk_across_street"
        )

        dao = GlobalRoutePlannerDAO(CarlaDataProvider.get_map(), 2)
        global_route_planner = GlobalRoutePlanner(dao)
        global_route_planner.setup()

        actor_behaviors = py_trees.composites.Parallel(name="actor_behaviors")
        actor_behaviors.add_child(walk_across_street)
        # actor_behaviors.add_child(drive_through_intersection)

        end_condition = DriveDistance(carma_vehicle, 10)

        root = py_trees.composites.Sequence(name="root_sequence")
        root.add_child(start_condition)
        root.add_child(actor_behaviors)
        root.add_child(end_condition)

        return root

    def _create_test_criteria(self) -> list:
        """
        Setup the evaluation criteria for NewScenario

        Note: this function overrides the one in BasicScenario (parent class).

        :return: List of test criteria
        """
        return [
            # This is an example usage for including test criteria in
            # ScenarioRunner.
            # CollisionTest(self.carma_vehicle)
        ]

    def __del__(self):
        self.remove_all_actors()
