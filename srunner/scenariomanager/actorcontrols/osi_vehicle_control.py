#!/usr/bin/env python


"""
This module provides an osi controller for actors, that forwards actions as 
OSI messages to an external software for longitudinal and lateral control command calculation.

To generate grpc files use
 python -m grpc_tools.protoc -I ScenarioRunner/osi/src/ --python_out ScenarioRunner/osi/generated/ --grpc_python_out ScenarioRunner/osi/generated/ ScenarioRunner/osi/src/*.proto
or similar. Make sure to configure osi_version.proto.in or osi_version.proto won't exist

This module is not intended for modification.
"""

from math import radians
from itertools import count
from datetime import timedelta
import math
import numpy as np

import carla
from srunner.scenariomanager.actorcontrols.actor_control import ActorControl
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenario_manager import GameTime

import grpc
from osi3.osi_common_pb2 import Identifier as OSIIdentifier, Vector2d as OSIVec2d, Vector3d as OSIVec3d, Orientation3d as OSIOrientation3d, Timestamp as OSITimestamp
import osi3.osi_trafficcommand_pb2 as osi_tc
import srunner.osi.client.ScenarioRunner_pb2
import srunner.osi.client.ScenarioRunner_pb2_grpc

#class OSIVehicleControl(ActorControl):
class OSIVehicleControl(object):

    """
    Actor control class for actors, with externally implemented longitudinal and
    lateral controllers using OSI messages for forwarding to the given gRPC host.

    Args:
        actor (carla.Actor): Actor that should be controlled by the agent.
        args.host (string): Hostname and port of the gRPC service.

    Note:
        This is intended to use with openscenario and the arguments (host) must be defined
        the xosc file.
    """

    def __init__(self, actor, args=None):
        # super(OSIVehicleControl, self).__init__(actor)
        self._actor = actor
        if args is None or "osiService" not in args:
            self._host = "http://carlaproxy.dp.ts.dlr.de"
        else:
            self._host = args["osiService"]
        print("Try to connect to OSI-Service at: " + self._host)
        self._channel = grpc.insecure_channel(self._host, options=(('grpc.enable_http_proxy', 0),))
        self._client = srunner.osi.client.ScenarioRunner_pb2_grpc.OSIVehicleControllerStub(
            self._channel)
        # action id has to be unique within all traffic command messages exchanged with _one_ traffic participant
        # the same id can be reused in another action for a different traffic participant
        self._counter = 0
        self._time_step = 0.0
        self._end_waypoint = []

    def reset(self):
        """
        Reset the controller
        """
        print("Reset OSI Controller")
        pass

    def run_step(self):
        """
        The control loop and setting the actor controls is implemented externally.
        """
        print("osi_vehicle_control: called run_step")
        if self._end_waypoint:
            command = self._make_traffic_command()
            action = command.action.add()
            #send AcquireGlobalPositionAction
            action.acquire_global_position_action.action_header.action_id.value = self._next_id()
            position, orientation = self.to_osi_transform(self._end_waypoint)
            action.acquire_global_position_action.position.CopyFrom(position)
            action.acquire_global_position_action.orientation.CopyFrom(orientation)
            
            self._try_send_command(command)

    def _make_traffic_command(self):
        command = osi_tc.TrafficCommand()
        #TODO find a reliable way to determine interface version
        # command.version =
        command.traffic_participant_id.CopyFrom(self.to_osi_id(self._actor.id))
        command.timestamp.CopyFrom(
            self.to_osi_timestamp(GameTime.get_carla_time()))
        return command

    def _next_id(self):
        self._counter += 1
        return self._counter

    def _try_send_command(self, command: osi_tc.TrafficCommand):
        try:
            self._time_step = self._client.SendCommand(command)
            #TODO use values from config
            step_width = 0.03
            tolerance = 0.0001
            time_value = self._time_step.value
            if  abs(step_width - time_value) >= tolerance:
                raise Exception('Returned step size is not expected. Shall be 0.03, but is: ' + str(self._time_step))
        except Exception as e:
            print("Failed to send osi traffic command:", e, sep='\n')
            #TODO pass exception on
            raise e

    def update_target_speed(self, target_speed):
        """
        Update the actor's target speed.

        This method is always called with an absolute target speed - other target speed
        types are handled and translated by the calling ChangeActorTargetSpeed atomic behaviour

        Args:
            target_speed (float): New target speed [m/s].
        """
        print("osi_vehicle_control: called update_target_speed")
        self._send_speed_action(target_speed)

    def set_init_speed(self):
        """
        Update the actor's initial speed setting
        """
        print("osi_vehicle_control: called set_init_speed")
        current_speed = math.sqrt(
            self._actor.get_velocity().x**2 + self._actor.get_velocity().y**2)
        self._send_speed_action(
            current_speed, osi_tc.SpeedAction.DynamicsShape.DYNAMICS_SHAPE_STEP)

    def _send_speed_action(self, target_speed: float, dynamics_shape=osi_tc.SpeedAction.DYNAMICS_SHAPE_UNSPECIFIED):
        #TODO use DYNAMICS_SHAPE_STEP as default to immediately request target speed (when duration == 0 == distance)?
        command = self._make_traffic_command()
        action = command.action.add()
        action.speed_action.action_header.action_id.value = self._next_id()
        action.speed_action.absolute_target_speed = target_speed
        action.speed_action.dynamics_shape = dynamics_shape
        action.speed_action.duration = 0
        action.speed_action.distance = 0
        #raise Exception('Do not send speed action seperately. It will break the system!')
        #self._try_send_command(command)

    def update_waypoints(self, waypoints):
        """
        Update the actor's waypoints

        Args:
            waypoints (List of carla.Transform): List of new waypoints.
        """
        print("osi_vehicle_control: called update_waypoints")

        #save waypoint for decision if target is reached
        if 0 < len(waypoints):
            self._end_waypoint = waypoints[-1]
        else:
            self._end_waypoint = None
            raise Exception('No end waypoint defined')

    def update_trajectory(self, vertexes, start_time: float):
        """
        Custom code extension
        Update the actor's trajectory

        Args:
            vertexes (List of carla.Transform and timestamp): List of new trajectory(vertexes).
            start_time (float): Start time of the new "maneuver" [s].
        """
        print("osi_vehicle_control: called update_trajctory")
        pass

    def check_reached_waypoint_goal(self):
        """
        Check if the actor reached the end of the waypoint list

        returns:
            True if the end was reached, False otherwise.
        """
        #TODO
        return self._end_waypoint.location.distance(self._actor.get_transform().location) < 1.0

    def check_reached_trajectory_goal(self):
        """
        Check if the actor reached the end of the waypoint list

        returns:
            True if the end was reached, False otherwise.
        """
        return False

    @staticmethod
    def to_osi_id(actor_id: int):
        """
        Translate carla actor id to OSI identifier value using the mapping of
        the DLR's CARLA OSI interface

        OSI identifier value has 64 bit while carla actor id has only 32 bits
        and stored in the upper 32 bits of the osi identifier value.
        """
        return OSIIdentifier(value=(actor_id << 32) + (1 << 24))

    @staticmethod
    def to_osi_timestamp(timestamp: carla.Timestamp):
        t = timedelta(
            seconds=timestamp.elapsed_seconds if timestamp is carla.Timestamp else timestamp)
        #TODO microseconds at least initially is less precise than the fractional of the float
        return OSITimestamp(seconds=int(t.total_seconds()), nanos=t.microseconds * 1000)

    @staticmethod
    def to_osi_vector3d(vec: carla.Vector3D):
        return OSIVec3d(x=vec.x, y=-vec.y, z=vec.z)

    @staticmethod
    def to_osi_orientation3d(rot: carla.Rotation):
        #TODO OSI prefers values in angular range [pi,pi]
        return OSIOrientation3d(yaw=-radians(rot.yaw), pitch=radians(rot.pitch), roll=radians(rot.roll))

    @staticmethod
    def to_osi_transform(transform: carla.Transform):
        return OSIVehicleControl.to_osi_vector3d(transform.location), OSIVehicleControl.to_osi_orientation3d(transform.rotation)


# redirect to correct capitalization
OsiVehicleControl = OSIVehicleControl
