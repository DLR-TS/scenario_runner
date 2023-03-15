"""
This module provides an example control for vehicles which follows a trajectory
"""

import math
import numpy as np
import carla

from srunner.scenariomanager.actorcontrols.basic_control import BasicControl
from srunner.scenariomanager.timer import GameTime

from collections import deque, namedtuple


class Pose:

    def __init__(self):
        x = 0.0
        y = 0.0
        yaw = 0.0
        dyaw = 0.0
        v = 0.0
        vx = 0.0
        vy = 0.0
        ax = 0.0
        ay = 0.0
        t = 0.0


class FollowTrajectoryControl(BasicControl):

    """
    Controller class for vehicles derived from BasicControl.

    Args:
        actor (carla.Actor): Vehicle actor that should be controlled.
    """

    _args = {'K_P': 1.0, 'K_D': 0.01, 'K_I': 0.0, 'dt': 0.05}
    _vertexes = []
    _start_time = None
    _vertexes_updated = False
    _velocity_map = []

    def __init__(self, actor, args=None):

        self.new_pose_long_ctrl = Pose()
        self.new_pose_lat_ctrl = Pose()
        self._actor = actor

        self.elapsed_time = 0.0
        self.last_elapsed_time = 0.0
        self.start_time = 0.0

        super(FollowTrajectoryControl, self).__init__(actor)

    def update_trajectory(self, vertexes, start_time=None):
        """
        Update the actor's vertexes

        Args:
            vertexes (List of carla.Transform): List of new vertexes.
        """
        if start_time:
            self.start_time = start_time
            self.elapsed_time = start_time
        self._vertexes = vertexes
        self._vertexes_updated = True

    def reset(self):
        """
        Reset the controller
        """
        if self._actor and self._actor.is_alive:
            self._actor = None

    def check_reached_trajectory_goal(self):
        """
        Check if the actor reached the end of the vertex list

        returns:
            True if the end was reached, False otherwise.
        """
        return self._reached_goal

    def interpolate_quadratic(self, vertex_i, sim_time):

        """
        Args:
            vertex_i:
            sim_time:

        Returns: interpolated pose

        """

        s0 = self._vertexes[vertex_i]
        s1 = self._vertexes[vertex_i+1]
        s2 = self._vertexes[vertex_i+2]

        x0 = s0.pos.location.x
        y0 = s0.pos.location.y
        h0 = s0.pos.rotation.yaw
        t0 = s0.time

        x1 = s1.pos.location.x
        y1 = s1.pos.location.y
        h1 = s1.pos.rotation.yaw
        t1 = s1.time

        x2 = s2.pos.location.x
        y2 = s2.pos.location.y
        h2 = s2.pos.rotation.yaw
        t2 = s2.time

        new_pose = Pose()

        # define formular L0, L1, L2 for x, y for each point

        l_0_factor = 1 / ((t0 - t1) * (t0 - t2))
        l_1_factor = 1 / ((t1 - t0) * (t1 - t2))
        l_2_factor = 1 / ((t2 - t0) * (t2 - t1))

        new_pose.x = x0 * (sim_time - t1) * (sim_time - t2) * l_0_factor + \
            x1 * (sim_time - t0) * (sim_time - t2) * l_1_factor + \
            x2 * (sim_time - t0) * (sim_time - t1) * l_2_factor
        new_pose.y = y0 * (sim_time - t1) * (sim_time - t2) * l_0_factor + \
            y1 * (sim_time - t0) * (sim_time - t2) * l_1_factor + \
            y2 * (sim_time - t0) * (sim_time - t1) * l_2_factor

        new_pose.vx = x0 * (2*sim_time - t1 - t2) * l_0_factor + \
            x1 * (2*sim_time - t0 - t2) * l_1_factor + \
            x2 * (2*sim_time - t0 - t1) * l_2_factor
        new_pose.vy = y0 * (2*sim_time - t1 - t2) * l_0_factor + \
            y1 * (2*sim_time - t0 - t2) * l_1_factor + \
            y2 * (2*sim_time - t0 - t1) * l_2_factor

        new_pose.ax = 2 * (x0 * l_0_factor + x1 * l_1_factor + x2 * l_2_factor)
        new_pose.ay = 2 * (y0 * l_0_factor + y1 * l_1_factor + y2 * l_2_factor)

        new_pose.yaw = math.atan2(new_pose.vy, new_pose.vy)
        new_pose.dyaw = (new_pose.ay * new_pose.vx - new_pose.vy * new_pose.ax) / \
            (new_pose.vx**2 + new_pose.vy**2)

        new_pose.v = math.sqrt(new_pose.vx**2 + new_pose.vy**2)

        return new_pose

    def interpolate_linear(self, vertex_i, sim_time):

        """
        linear interpolation between 2 vertex points
        """

        s0 = self._vertexes[vertex_i]
        s1 = self._vertexes[vertex_i+1]

        x0 = s0.pos.location.x
        y0 = s0.pos.location.y
        h0 = s0.pos.rotation.yaw
        t0 = s0.time

        x1 = s1.pos.location.x
        y1 = s1.pos.location.y
        h1 = s1.pos.rotation.yaw
        t1 = s1.time

        new_pose = Pose()

        new_pose.yaw = math.atan2(y1-y0, x1-x0)
        new_pose.v = 0

        if sim_time < t0:
            new_pose.x = x0
            new_pose.y = y0
            return 0

        if sim_time > t1:
            new_pose.x = x1
            new_pose.y = y1
            return 0

        dt = t1-t0

        alpha = (sim_time-t0) / dt
        new_pose.x = (1-alpha)*x0 + alpha*x1
        new_pose.y = (1-alpha)*y0 + alpha*y1
        new_pose.vx = (x1-x0) / dt
        new_pose.vy = (y1-y0) / dt
        new_pose.v = math.sqrt(new_pose.vx**2 + new_pose.vy**2)

        return new_pose

    @staticmethod
    def _convert_motion_control_long(current_vel, target_vel, current_pos, target_pos, dt):

        pid = PIDLongitudinalController()

        # ###################### ds-approach ####################
        # reverse = False
        # brake_cmd = 0

        # cmd = pid.pid_control_s(target_pos, current_pos, dt)

        # if cmd > 0:
        #     throttle_cmd = cmd
        #     brake_cmd = 0
        # else:
        #     brake_cmd = abs(cmd)
        #     throttle_cmd = 0

        # ####################### dv-approach ###################

        reverse = False
        brake_cmd = 0
        if target_vel < 0:
            reverse = True

        if abs(target_vel) <= 0.5:
            throttle_cmd = 0
            brake_cmd = 0.5
            if abs(current_vel) <= 0.1:
                brake_cmd = 0
        else:
            throttle_cmd = pid.pid_control_v(target_vel, current_vel, dt)
            if throttle_cmd < 0.0:
                brake_cmd = abs(throttle_cmd)
                throttle_cmd = 0.0

        return throttle_cmd, reverse, brake_cmd

    @staticmethod
    def _convert_motion_control_lat(current_pos, target_pos, dt):

        pid = PIDLateralController()
        steer_cmd = pid.pid_control(target_pos, current_pos, dt)

        return steer_cmd

    def run_step(self):
        """
        Execute on tick of the controller's control loop
        """
        if self._vertexes_updated:

            self._reached_goal = False

            # Assuming the time is in seconds
            self.last_elapsed_time = self.elapsed_time
            self.elapsed_time = GameTime.get_time() - self.start_time
            dt = self.elapsed_time - self.last_elapsed_time

            # this is for the lateral control
            look_ahead_time = 0.3

            i = None
            vertex = None
            time_check = None

            for i, vertex in enumerate(self._vertexes):
                time_check = vertex.time
                if self.elapsed_time < time_check:
                    break

            # trajectory is in the future
            if i == 0:

                self.new_pose_long_ctrl.v = 0
                self.new_pose_long_ctrl.x = self._vertexes[0].pos.location.x
                self.new_pose_long_ctrl.y = self._vertexes[0].pos.location.y
                self.new_pose_lat_ctrl = self.new_pose_long_ctrl

            else:

                # only change pose if time does not exceed trajectory time
                if self.elapsed_time <= time_check:
                    start_index = i - 1
                    if len(self._vertexes) == 2:
                        self.new_pose_long_ctrl = self.interpolate_linear(start_index, self.elapsed_time)
                        self.new_pose_lat_ctrl = self.interpolate_linear(start_index,
                                                                         self.elapsed_time + look_ahead_time)
                    else:
                        if i == (len(self._vertexes) - 1):
                            start_index = i - 2
                        self.new_pose_long_ctrl = self.interpolate_quadratic(start_index, self.elapsed_time)
                        self.new_pose_lat_ctrl = self.interpolate_quadratic(start_index,
                                                                            self.elapsed_time + look_ahead_time)

                        # self.new_pose_long_ctrl = self.interpolate_linear(start_index, self.elapsed_time)
                        # self.new_pose_lat_ctrl = self.interpolate_linear(start_index,
                        #                                                  self.elapsed_time + look_ahead_time)

                else:
                    self._reached_goal = True
                    self._vertexes_updated = False
                    self.new_pose_long_ctrl.v = 0

            target_speed = self.new_pose_long_ctrl.v

            # Hard set the velocity vector so actors dont start at 0 velocity, fix for setlevel truck problem
            if self.elapsed_time < 0.3:
                speed_vector = carla.Vector3D(self.new_pose_long_ctrl.vx,self.new_pose_long_ctrl.vy,0)
                self._actor.set_target_velocity(speed_vector)

            target_location = carla.Transform(carla.Location(x=self.new_pose_long_ctrl.x, y=self.new_pose_long_ctrl.y))

            current_location = self._actor.get_transform().location

            # euclidian_distance = math.sqrt((self.new_pose_long_ctrl.x-current_location.x)**2 +
            #                               (self.new_pose_long_ctrl.y-current_location.y)**2)

            # print("euclidian distance: ", euclidian_distance)

            control = self._actor.get_control()

            if isinstance(self._actor, carla.Walker):

                control.speed = target_speed
                direction = target_location.location - self._actor.get_location()
                direction_norm = math.sqrt(direction.x ** 2 + direction.y ** 2)
                control.direction = direction / direction_norm

            elif isinstance(self._actor, carla.Vehicle):

                v = self._actor.get_velocity()
                current_vel = math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)

                current_pos = self._actor.get_transform()

                print(self._actor)
                print("pos current: ", current_location.x, current_location.y,
                      "pos target: ", target_location.location.x, target_location.location.y,
                      "time: ", self.elapsed_time, dt)
                print("target vel: ", target_speed,
                      "current vel: ", current_vel,
                      "time: ", self.elapsed_time, dt)

                throttle_cmd, reverse_cmd, brake_cmd = self._convert_motion_control_long(current_vel, target_speed,
                                                                                         current_pos, target_location,
                                                                                         dt)

                target_location_lat = carla.Transform(carla.Location(x=self.new_pose_lat_ctrl.x,
                                                                     y=self.new_pose_lat_ctrl.y))

                steer_cmd = self._convert_motion_control_lat(current_pos, target_location_lat, dt)

                # print("throttle: ", throttle_cmd, "brake: ", brake_cmd)

                # use this just when v is the control value ###########################################
                _e = math.cos(math.radians(current_pos.rotation.yaw)) * (
                        target_location.location.x - current_pos.location.x) + \
                    math.sin(math.radians(current_pos.rotation.yaw)) * \
                    (target_location.location.y - current_pos.location.y)

                if _e > 1.0:
                    throttle_cmd = max(min(throttle_cmd + _e * 0.1, 1.0), 0.0)
                if _e * 0.1 < 0.0 and throttle_cmd == 0.0 and brake_cmd == 0.0:
                    brake_cmd = min(abs(_e * 0.5), 1.0)

                # print("adapted throttle: ", throttle_cmd,
                #     "brake command: ", brake_cmd,
                #      "_e: ", _e)

                #######################################################################################

                control.manual_gear_shift = False

                if current_vel <= 0.1:
                    control.manual_gear_shift = True
                    control.gear = 1

                control.steer = steer_cmd
                control.brake = brake_cmd
                control.throttle = throttle_cmd
                control.reverse = reverse_cmd

                if not self._vertexes_updated:
                    control.brake = 1

            else:
                raise Exception("unkown instance! No controller available")

            self._actor.apply_control(control)

            if self._vertexes[-1].pos.location.distance(self._actor.get_transform().location) < 1.0:
                self._reached_goal = True
                self._vertexes_updated = False
                control.brake = 1
                self._actor.apply_control(control)

# ==============================================================================
# -- PIDLongitudinalController -------------------------------------------------
# ==============================================================================


class PIDLongitudinalController(object):
    """
    PIDLongitudinalController implements longitudinal control using a PID.
    """

    def __init__(self, K_P=1.0, K_D=0.0, K_I=0.01, dt=0.05):
        """
        :param K_P: Proportional term
        :param K_D: Differential term
        :param K_I: Integral term
        :param dt: time differential in seconds
        """

        #todo: for dt use real time difference between calls
        self._K_P = K_P
        self._K_D = K_D
        self._K_I = K_I
        self._dt = dt
        self._e_buffer = deque(maxlen=30)

    def pid_control_v(self, target_speed, current_speed, dt):
        """
        Estimate the throttle of the vehicle based on the PID equations

        :param target_speed:  target speed in Km/h
        :param current_speed: current speed of the vehicle in Km/h
        :return: throttle control in the range [0, 1]
        """

        _e = (target_speed - current_speed)

        self._e_buffer.append(_e)

        self._dt = dt

        if len(self._e_buffer) >= 2:
            _de = (self._e_buffer[-1] - self._e_buffer[-2]) / self._dt
            _ie = sum(self._e_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        return np.clip((self._K_P * _e) + (self._K_D * _de / self._dt) + (self._K_I * _ie * self._dt), 0.0, 1.0)

    def pid_control_s(self, target_pos, current_pos, dt):
        """
        Estimate the throttle of the vehicle based on the PID equations

        :param target_speed:  target speed in Km/h
        :param current_speed: current speed of the vehicle in Km/h
        :return: throttle control in the range [0, 1]
        """

        _e = math.cos(math.radians(current_pos.rotation.yaw)) * (target_pos.location.x - current_pos.location.x) + \
            math.sin(math.radians(current_pos.rotation.yaw)) * (target_pos.location.y - current_pos.location.y)

        print("dx for control: ", _e)

        self._e_buffer.append(_e)

        self._dt = dt

        if len(self._e_buffer) >= 2:
            _de = (self._e_buffer[-1] - self._e_buffer[-2]) / self._dt
            _ie = sum(self._e_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        return np.clip((self._K_P * _e) + (self._K_D * _de / self._dt) + (self._K_I * _ie * self._dt), -1.0, 1.0)

# ==============================================================================
# -- PIDLateralController -------------------------------------------------
# ==============================================================================


class PIDLateralController(object):
    """
    PIDLateralController implements lateral control using a PID.
    """

    def __init__(self, K_P=1.95, K_D=0.2, K_I=0.00, dt=0.05):
        """
        Constructor method.

            :param vehicle: actor to apply to local planner logic onto
            :param K_P: Proportional term
            :param K_D: Differential term
            :param K_I: Integral term
            :param dt: time differential in seconds
        """
        self._k_p = K_P
        self._k_d = K_D
        self._k_i = K_I
        self._dt = dt
        self._e_buffer = deque(maxlen=30)

    def pid_control(self, target_pos, vehicle_transform, dt):
        """
        Estimate the steering angle of the vehicle based on the PID equations

            :param waypoint: target waypoint
            :param vehicle_transform: current transform of the vehicle
            :return: steering control in the range [-1, 1]
        """
        v_begin = vehicle_transform.location
        v_end = v_begin + carla.Location(x=math.cos(math.radians(vehicle_transform.rotation.yaw)),
                                         y=math.sin(math.radians(vehicle_transform.rotation.yaw)))

        v_vec = np.array([v_end.x - v_begin.x, v_end.y - v_begin.y, 0.0])
        w_vec = np.array([target_pos.location.x -
                          v_begin.x, target_pos.location.y -
                          v_begin.y, 0.0])
        _dot = math.acos(np.clip(np.dot(w_vec, v_vec) /
                                 (np.linalg.norm(w_vec) * np.linalg.norm(v_vec)), -1.0, 1.0))

        _cross = np.cross(v_vec, w_vec)

        self._dt = dt

        if _cross[2] < 0:
            _dot *= -1.0

        self._e_buffer.append(_dot)
        if len(self._e_buffer) >= 2:
            _de = (self._e_buffer[-1] - self._e_buffer[-2]) / self._dt
            _ie = sum(self._e_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        return np.clip((self._k_p * _dot) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)
