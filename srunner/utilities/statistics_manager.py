#!/usr/bin/env python

# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module contains a statistics manager for the CARLA AD leaderboard
"""

from __future__ import print_function

import py_trees.common
from dictor import dictor
import math
import sys
import os

from srunner.scenariomanager.traffic_events import TrafficEventType

from srunner.utilities.checkpoint_tools import fetch_dict, create_default_json_msg, save_dict

from srunner.tools.route_manipulation import interpolate_trajectory
from srunner.scenariomanager.scenarioatomics.atomic_criteria import (CollisionTest,
																	 InRouteTest,
																	 RouteCompletionTest,
																	 OutsideRouteLanesTest,
																	 RunningRedLightTest,
																	 RunningStopTest,
																	 ActorSpeedAboveThresholdTest)
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenario_manager import ScenarioManager
from srunner.scenarios.open_scenario_with_agent import OpenScenarioWithAgent


DATA_COLLECTION = os.environ.get('DATA_COLLECTION', None)

PENALTY_COLLISION_PEDESTRIAN = 0.50
PENALTY_COLLISION_VEHICLE = 0.60
PENALTY_COLLISION_STATIC = 0.65
PENALTY_TRAFFIC_LIGHT = 0.70
PENALTY_STOP = 0.80





class RouteRecord():
    def __init__(self):
        self.route_id = None
        self.index = None
        self.status = 'Started'
        self.infractions = {
            'collisions_pedestrian': [],
            'collisions_vehicle': [],
            'collisions_layout': [],
            'red_light': [],
            'stop_infraction': [],
            'outside_route_lanes': [],
            'route_dev': [],
            'route_timeout': [],
            'vehicle_blocked': []
        }

        self.scores = {
            'score_route': 0,
            'score_penalty': 0,
            'score_composed': 0
        }

        self.meta = {}

def convert_transform_to_location(transform_vec):
    """
    Convert a vector of transforms to a vector of locations
    """
    location_vec = []
    # print("!!!!!!!transformvec")
    # print(transform_vec)
    for transform_tuple in transform_vec:
        location_vec.append((transform_tuple[0].location, transform_tuple[1]))

    return location_vec

def to_route_record(record_dict):
    record = RouteRecord()
    for key, value in record_dict.items():
        setattr(record, key, value)

    return record


def compute_route_length(config):
    trajectory = config.trajectory

    route_length = 0.0
    previous_location = None
    for location in trajectory:
        if previous_location:
            dist = math.sqrt((location.x-previous_location.x)*(location.x-previous_location.x) +
                             (location.y-previous_location.y)*(location.y-previous_location.y) +
                             (location.z - previous_location.z) * (location.z - previous_location.z))
            route_length += dist
        previous_location = location

    return route_length


class StatisticsManager(object):

    """
    This is the statistics manager for the CARLA leaderboard.
    It gathers data at runtime via the scenario evaluation criteria.
    """

    def __init__(self):
        self._master_scenario = None
        self._registry_route_records = []

    def resume(self, endpoint):
        data = fetch_dict(endpoint)

        if data and dictor(data, '_checkpoint.records'):
            records = data['_checkpoint']['records']

            for record in records:
                self._registry_route_records.append(to_route_record(record))

    def set_route(self, route_id, index):

        self._master_scenario = None
        route_record = RouteRecord()
        route_record.route_id = route_id
        route_record.index = index

        if index < len(self._registry_route_records):
            # the element already exists and therefore we update it
            self._registry_route_records[index] = route_record
        else:
            self._registry_route_records.append(route_record)

        print("!!!!route record")
        print(vars(route_record))

    def set_scenario(self, scenario):
        """
        Sets the scenario from which the statistics willb e taken
        """
        self._master_scenario = scenario

        # print("!!!!!!!")
        # print(scenario)
        # print(vars(scenario))



    def compute_route_statistics(self, config, duration_time_system=-1, duration_time_game=-1, failure=""):
        """
        Compute the current statistics by evaluating all relevant scenario criteria
        """
        index = 0

        # print("!!!!!config")
        # print(vars(config))
        # client = config.client
        # criteria = []
        # world = client.load_world(config.town)
        # if world is None:
        #     raise RuntimeError(
        #         "World is not set. Ensure the CARLA server is running and CarlaDataProvider is initialized.")
        # gps_route, route = interpolate_trajectory(world, config.trajectory)
        # route = convert_transform_to_location(route)
        #
        # # we terminate the route if collision or red light infraction happens during data collection
        #
        # if DATA_COLLECTION:
        #     collision_criterion = CollisionTest(config.ego_vehicles[0], terminate_on_failure=True)
        #     red_light_criterion = RunningRedLightTest(config.ego_vehicles[0], terminate_on_failure=True)
        # else:
        #     collision_criterion = CollisionTest(config.ego_vehicles[0], terminate_on_failure=False)
        #     red_light_criterion = RunningRedLightTest(config.ego_vehicles[0], terminate_on_failure=False)
        # route_criterion = InRouteTest(config.ego_vehicles[0],
        #                               route=route,
        #                               offroad_max=30,
        #                               terminate_on_failure=True)
        #
        # completion_criterion = RouteCompletionTest(config.ego_vehicles[0], route=route)
        #
        # outsidelane_criterion = OutsideRouteLanesTest(config.ego_vehicles[0], route=route)
        #
        # stop_criterion = RunningStopTest(config.ego_vehicles[0])
        #
        # blocked_criterion = ActorSpeedAboveThresholdTest(config.ego_vehicles[0],
        #                                                  speed_threshold=0.1,
        #                                                  below_threshold_max_time=180.0,
        #                                                  terminate_on_failure=True,
        #                                                  name="AgentBlockedTest")
        #
        # criteria.append(completion_criterion)
        # criteria.append(outsidelane_criterion)
        # criteria.append(collision_criterion)
        # criteria.append(red_light_criterion)
        # criteria.append(stop_criterion)
        # criteria.append(route_criterion)
        # criteria.append(blocked_criterion)


        if not self._registry_route_records or index >= len(self._registry_route_records):
            raise Exception('Critical error with the route registry.')

        # fetch latest record to fill in
        route_record = self._registry_route_records[index]

        target_reached = False
        score_penalty = 1.0
        score_route = 0.0

        route_record.meta['duration_system'] = duration_time_system
        route_record.meta['duration_game'] = duration_time_game
        route_record.meta['route_length'] = compute_route_length(config)

        route_record.meta['total_steps'] = config.agent.step



        print("!!!!! score_route 1: ")
        print(score_route)
        if self._master_scenario:
            print("mastrscenario" )
            print(self._master_scenario)
            print(vars(self._master_scenario))
            if self._master_scenario.timeout_node.timeout:
                route_record.infractions['route_timeout'].append('Route timeout.')
                failure = "Agent timed out"
            print("!!!!! score_route 3: ")
            print(score_route)
            # print("criteria")
            # print(criteria)
            for node in self._master_scenario.get_criteria():
                print("!!!!!!!!!getcriteria")
                print(self._master_scenario.criteria_tree)
                print(vars(self._master_scenario.criteria_tree))
                print("!!!!! score_route 4: ")
                print(score_route)
                print("!!!!!!!!!node")
                print(node)
                print(vars(node))
                if hasattr(node, 'list_traffic_events'):
                    print(f"Node: {node.name}, Traffic Events: {node.list_traffic_events}")
                    # analyze all traffic events
                    print("!!!!!!!!!traffic events")
                    print(node.list_traffic_events)
                    print("!!!!! score_route 5: ")
                    print(score_route)
                    for event in node.list_traffic_events:
                        print("!!!!! score_route 6: ")
                        print(score_route)
                        print(event.get_type())
                        if event.get_type() == TrafficEventType.COLLISION_STATIC:
                            score_penalty *= PENALTY_COLLISION_STATIC
                            route_record.infractions['collisions_layout'].append(event.get_message())

                        elif event.get_type() == TrafficEventType.COLLISION_PEDESTRIAN:
                            score_penalty *= PENALTY_COLLISION_PEDESTRIAN
                            route_record.infractions['collisions_pedestrian'].append(event.get_message())

                        elif event.get_type() == TrafficEventType.COLLISION_VEHICLE:
                            score_penalty *= PENALTY_COLLISION_VEHICLE
                            route_record.infractions['collisions_vehicle'].append(event.get_message())

                        elif event.get_type() == TrafficEventType.OUTSIDE_ROUTE_LANES_INFRACTION:
                            score_penalty *= (1 - event.get_dict()['percentage'] / 100)
                            route_record.infractions['outside_route_lanes'].append(event.get_message())

                        elif event.get_type() == TrafficEventType.TRAFFIC_LIGHT_INFRACTION:
                            score_penalty *= PENALTY_TRAFFIC_LIGHT
                            route_record.infractions['red_light'].append(event.get_message())

                        elif event.get_type() == TrafficEventType.ROUTE_DEVIATION:
                            route_record.infractions['route_dev'].append(event.get_message())
                            # failure = "Agent deviated from the route"
                            score_penalty *= 0.8  # Reduce score by 20% for deviation

                        # elif event.get_type() == TrafficEventType.STOP_INFRACTION:
                        #     score_penalty *= PENALTY_STOP
                        #     route_record.infractions['stop_infraction'].append(event.get_message())

                        elif event.get_type() == TrafficEventType.VEHICLE_BLOCKED:
                            route_record.infractions['vehicle_blocked'].append(event.get_message())
                            failure = "Agent got blocked"

                        # elif event.get_type() == TrafficEventType.ROUTE_COMPLETED:
                        #     score_route = 100.0
                        #     target_reached = True
                        # elif event.get_type() == TrafficEventType.ROUTE_COMPLETION:
                        #     if not target_reached:
                        #         if event.get_dict():
                        #             score_route = event.get_dict()['route_completed']
                        #         else:
                        #             score_route = 0
                elif node.name == "RouteCompleted" and node.status == py_trees.common.Status.SUCCESS:
                    score_route = 100.0
                    target_reached = True
                    print("!!!!!!osc")
                    print(node._osc_position)
                    print(dir(node._osc_position))
                else:
                    if not target_reached and node.name == "RouteCompleted":
                        print("!!!!!!!!!os positon")
                        # client = config.client
                        # world = client.load_world(config.town)
                        world = config.world
                        if world is None:
                            raise RuntimeError(
                                "World is not set. Ensure the CARLA server is running and CarlaDataProvider is initialized.")
                        gps_route, route = interpolate_trajectory(world, config.trajectory)
                        route = convert_transform_to_location(route)
                        print("!!!!")
                        print(route)
                        completion_criterion = RouteCompletionTest(node._actor, route=route)
                        print("!!!!!completion")
                        print(completion_criterion)
                        print(vars(completion_criterion))
                        print(node._actor)
                        print(node._osc_position)
                        print(dir(node._osc_position))




        # update route scores
        print("!!!!!!!!!!after traffic events")
        route_record.scores['score_route'] = score_route
        route_record.scores['score_penalty'] = score_penalty
        route_record.scores['score_composed'] = max(score_route*score_penalty, 0.0)

        # update status
        if target_reached:
            route_record.status = 'Completed'
        else:
            route_record.status = 'Failed'
            if failure:
                route_record.status += ' - ' + failure

        return route_record

    def compute_global_statistics(self, total_routes):
        global_record = RouteRecord()
        global_record.route_id = -1
        global_record.index = -1
        global_record.status = 'Completed'

        if self._registry_route_records:
            for route_record in self._registry_route_records:
                global_record.scores['score_route'] += route_record.scores['score_route']
                global_record.scores['score_penalty'] += route_record.scores['score_penalty']
                global_record.scores['score_composed'] += route_record.scores['score_composed']

                for key in global_record.infractions.keys():
                    route_length_kms = max(route_record.scores['score_route'] * route_record.meta['route_length'] / 1000.0, 0.001)
                    if isinstance(global_record.infractions[key], list):
                        global_record.infractions[key] = len(route_record.infractions[key]) / route_length_kms
                    else:
                        global_record.infractions[key] += len(route_record.infractions[key]) / route_length_kms

                if route_record.status is not 'Completed':
                    global_record.status = 'Failed'
                    if 'exceptions' not in global_record.meta:
                        global_record.meta['exceptions'] = []
                    global_record.meta['exceptions'].append((route_record.route_id,
                                                             route_record.index,
                                                             route_record.status))

        global_record.scores['score_route'] /= float(total_routes)
        global_record.scores['score_penalty'] /= float(total_routes)
        global_record.scores['score_composed'] /= float(total_routes)

        return global_record

    @staticmethod
    def save_record(route_record, index, endpoint):
        data = fetch_dict(endpoint)
        if not data:
            data = create_default_json_msg()

        stats_dict = route_record.__dict__
        record_list = data['_checkpoint']['records']
        if index > len(record_list):
            print('Error! No enough entries in the list')
            sys.exit(-1)
        elif index == len(record_list):
            record_list.append(stats_dict)
        else:
            record_list[index] = stats_dict

        save_dict(endpoint, data)

    @staticmethod
    def save_global_record(route_record, sensors, total_routes, endpoint):
        data = fetch_dict(endpoint)
        if not data:
            data = create_default_json_msg()

        stats_dict = route_record.__dict__
        print("!!stats")
        print(stats_dict)
        data['_checkpoint']['global_record'] = stats_dict
        data['values'] = ['{:.3f}'.format(stats_dict['scores']['score_composed']),
                          '{:.3f}'.format(stats_dict['scores']['score_route']),
                          '{:.3f}'.format(stats_dict['scores']['score_penalty']),
                          # infractions
                          '{:.3f}'.format(stats_dict['infractions']['collisions_pedestrian']),
                          '{:.3f}'.format(stats_dict['infractions']['collisions_vehicle']),
                          '{:.3f}'.format(stats_dict['infractions']['collisions_layout']),
                          '{:.3f}'.format(stats_dict['infractions']['red_light']),
                          '{:.3f}'.format(stats_dict['infractions']['stop_infraction']),
                          '{:.3f}'.format(stats_dict['infractions']['outside_route_lanes']),
                          '{:.3f}'.format(stats_dict['infractions']['route_dev']),
                          '{:.3f}'.format(stats_dict['infractions']['route_timeout']),
                          '{:.3f}'.format(stats_dict['infractions']['vehicle_blocked'])
                          ]

        data['labels'] = ['Avg. driving score',
                          'Avg. route completion',
                          'Avg. infraction penalty',
                          'Collisions with pedestrians',
                          'Collisions with vehicles',
                          'Collisions with layout',
                          'Red lights infractions',
                          'Stop sign infractions',
                          'Off-road infractions',
                          'Route deviations',
                          'Route timeouts',
                          'Agent blocked'
                          ]

        entry_status = "Finished"
        eligible = True

        route_records = data["_checkpoint"]["records"]
        print("!!!!!!route_records")
        print(route_records)
        print("!!!!!!!!data")
        print(data)

        progress = data["_checkpoint"]["progress"]
        print("!!!!progress")
        print(progress)

        if progress[1] != total_routes:
            raise Exception('Critical error with the route registry.')

        if len(route_records) != total_routes or progress[0] != progress[1]:
            entry_status = "Finished with missing data"
            eligible = False
        else:
            for route in route_records:
                route_status = route["status"]
                if "Agent" in route_status:
                    entry_status = "Finished with agent errors"
                    break

        data['entry_status'] = entry_status
        data['eligible'] = eligible

        save_dict(endpoint, data)

    @staticmethod
    def save_sensors(sensors, endpoint):
        data = fetch_dict(endpoint)
        if not data:
            data = create_default_json_msg()

        if not data['sensors']:
            data['sensors'] = sensors

            save_dict(endpoint, data)

    @staticmethod
    def save_entry_status(entry_status, eligible, endpoint):
        data = fetch_dict(endpoint)
        if not data:
            data = create_default_json_msg()

        data['entry_status'] = entry_status
        data['eligible'] = eligible
        save_dict(endpoint, data)

    @staticmethod
    def clear_record(endpoint):
        if not endpoint.startswith(('http:', 'https:', 'ftp:')):
            with open(endpoint, 'w') as fd:
                fd.truncate(0)
