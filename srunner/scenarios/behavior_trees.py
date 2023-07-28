# This script contains pytrees built out of atomic behaviors, atomic criteria and atomic trigger conditions. 
# Import this script into your scenario description script to easily make your behavior tree.

import py_trees
import carla
from agents.navigation.local_planner import RoadOption
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      ActorDestroy, ChangeActorTargetSpeed, HandBrakeVehicle,
                                                                      KeepVelocity,
                                                                      StopVehicle,
                                                                      WaypointFollower,
                                                                      LaneChange,
                                                                      BasicAgentBehavior)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToVehicle,
                                                                               InTriggerDistanceToNextIntersection,
                                                                               DriveDistance,
                                                                               StandStill)
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance, generate_target_waypoint, generate_target_waypoint_list

class DriveToNextIntersection():
    def __init__(self, vehicle, vehicle_speed, distance_from_intersection):
        self.vehicle = vehicle
        self.vehicle_speed = vehicle_speed
        self.distance_from_intersection = distance_from_intersection
        self.behavior = None
    
    def create_tree(self):
        self.behavior = py_trees.composites.Parallel("DrivingTowardsIntersection", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        self.behavior.add_child(WaypointFollower(self.vehicle, self.vehicle_speed))
        self.behavior.add_child(InTriggerDistanceToNextIntersection(
            self.vehicle, self.distance_from_intersection))
        return self.behavior
    
class DriveAndTurnAtNextIntersection():
    def __init__(self, vehicle, vehicle_speed, start_location, turn='left'):
        self.vehicle = vehicle
        self.vehicle_speed = vehicle_speed
        self.start_location = start_location
        self.turn = turn
        self.map = CarlaDataProvider.get_map()
        self.turn_value = -1 #Default to left turn
        self.behavior = None

    def create_tree(self):
        if self.turn=='straight': self.turn_value = 0
        elif self.turn=='right': self.turn_value = 1
        plan = generate_target_waypoint(self.map.get_waypoint(self.start_location), self.turn_value)
        self.behavior = WaypointFollower(self.vehicle, self.vehicle_speed, plan=plan)
        return self.behavior, plan
    
class DriveAndSharpStopAtNextIntersection():
    def __init__(self, vehicle, vehicle_speed, distance_from_intersection):
        self.vehicle = vehicle
        self.vehicle_speed = vehicle_speed
        self.distance_from_intersection = distance_from_intersection
        self.behavior = None
    
    def create_tree(self):
        drive_until_intersection_subtree = DriveToNextIntersection(self.vehicle, self.vehicle_speed, self.distance_from_intersection).create_tree()
        self.behavior =  py_trees.composites.Sequence("Sequence Behavior")
        self.behavior.add_child(drive_until_intersection_subtree)
        self.behavior.add_child(StopVehicle(self.vehicle, 2.0))
        return self.behavior
    

class DriveAndTurnAtNextIntersectionTwice():
    def __init__(self, vehicle, vehicle_speed, start_location, turn_1='left', turn_2='left'):
        self.vehicle = vehicle
        self.vehicle_speed = vehicle_speed
        self.start_location = start_location
        self.turn_1 = turn_1
        self.turn_2 = turn_2
        self.map = CarlaDataProvider.get_map()
        self.turn_value_1 = -1 #Default to left turn
        self.turn_value_2 = -1
        self.behavior = None

    def create_tree(self):
        seq = py_trees.composites.Sequence("Sequence Behavior")
        behavior_tree_first_turn, plan = DriveAndTurnAtNextIntersection(self.vehicle, self.vehicle_speed, self.start_location, self.turn_1).create_tree()
        behavior_tree_second_turn, plan = DriveAndTurnAtNextIntersection(self.vehicle, self.vehicle_speed, plan[-1][0].transform.location, self.turn_2).create_tree()
        seq.add_child(behavior_tree_first_turn)
        seq.add_child(behavior_tree_second_turn)
        return seq
    
class DriveAndSlowDownAtNextIntersection():
    def __init__(self, vehicle, vehicle_speed, slow_down_speed):
        self.vehicle = vehicle
        self.vehicle_speed = vehicle_speed
        # self.distance_from_intersection = distance_from_intersection
        self.slow_down_speed = slow_down_speed
        self.behavior = None
    
    # Drive and slow down at intersection and increase velocity again right after
    def create_tree(self):
        # drive_until_intersection_subtree = DriveToNextIntersection(self.vehicle, self.vehicle_speed, self.distance_from_intersection).create_tree()
        self.behavior =  py_trees.composites.Sequence("Sequence Behavior")
        # self.behavior.add_child(drive_until_intersection_subtree)
        self.behavior.add_child(KeepVelocity(self.vehicle, self.slow_down_speed, 2.0))
        self.behavior.add_child(KeepVelocity(self.vehicle, self.vehicle_speed, 2.0))
        return self.behavior