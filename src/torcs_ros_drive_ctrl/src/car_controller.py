import numpy as np
from os.path import expanduser


class State:
    v = 0.
    rpm = 0.
    gear = 0.
    lanes = [[0] * 3, [0] * 3, 0, 0]
    goal = [0, 0]
    pitch = 0.
    distance_passed = 0.
    x = 0.
    y = 0.

    def distance_to_goal(self):
        return np.sqrt(self.goal[0] ** 2 + self.goal[1] ** 2)


class Command:
    acceleration = 0.
    brake = 0.
    gear = 0.
    steering = 0.
    target_speed = 0.


class Controller:
    MAX_SPEED = 300
    MIN_SPEED = 20
    K_steer = 1.2
    K_curve = 1000.
    K_pitch = 5.
    K_distance = 12.
    K_reduce_speed_from_steer = 12.
    K_accelerate = 1.
    K_brake = 5.
    RPM_GEAR_UP = 6000
    RPM_GEAR_DOWN = 1000

    def __init__(self, add_postfix=''):
        home = expanduser('~')
        self.filename = home + '/.torcs/log/controller_params' + add_postfix + '.yaml'

    def setup_params(self):
        import rospy
        self.MAX_SPEED = rospy.get_param('/torcs_driver/MAX_SPEED', self.MAX_SPEED)
        self.MIN_SPEED = rospy.get_param('/torcs_driver/MIN_SPEED', self.MIN_SPEED)
        self.RPM_GEAR_UP = rospy.get_param('/torcs_driver/RPM_GEAR_UP', self.RPM_GEAR_UP)
        self.RPM_GEAR_DOWN = rospy.get_param('/torcs_driver/RPM_GEAR_DOWN', self.RPM_GEAR_DOWN)
        self.K_reduce_speed_from_steer = rospy.get_param('/torcs_driver/K_reduce_speed_from_steer', self.K_reduce_speed_from_steer)
        self.K_distance = rospy.get_param('/torcs_driver/K_distance', self.K_distance)
        self.K_steer = rospy.get_param('/torcs_driver/K_steer', self.K_steer)
        self.K_curve = rospy.get_param('/torcs_driver/K_curve', self.K_curve)
        self.K_pitch = rospy.get_param('/torcs_driver/K_pitch', self.K_pitch)
        self.K_accelerate = rospy.get_param('/torcs_driver/K_accelerate', self.K_accelerate)
        self.K_brake = rospy.get_param('/torcs_driver/K_brake', self.K_brake)

    def control(self, state=State()):
        cmd = Command()
        self.steer_control(cmd, state)
        self.speed_control(cmd, state)
        self.gear_control(cmd, state)
        return cmd

    def gear_control(self, cmd=Command(), state=State()):
        cmd.gear = max(state.gear, 0)
        if cmd.acceleration > 0:
            if state.rpm > self.RPM_GEAR_UP:
                cmd.gear += 1
            elif state.rpm < self.RPM_GEAR_DOWN:
                cmd.gear = max(cmd.gear - 1, 1)
        else:
            if state.rpm < self.RPM_GEAR_DOWN:
                cmd.gear = max(cmd.gear - 1, 1)

    def steer_control(self, cmd=Command(), state=State()):
        if state.goal[0] == 0:
            alpha = 0
        else:
            alpha = np.arctan(state.goal[1] / state.goal[0])
        cmd.steering = alpha * self.K_steer

    def speed_control(self, cmd=Command(), state=State()):
        steepest_curve = max(abs(state.lanes[0][0]), abs(state.lanes[1][0]))
        target_speed_slope = self.MAX_SPEED - steepest_curve * self.K_curve
        target_speed_goal = self.K_distance * state.distance_to_goal()

        pitch = max(0., min(1, state.pitch * self.K_pitch))
        target_speed_pitch = self.MAX_SPEED * (1 - pitch)
        target_speed_steer = self.MAX_SPEED * np.cos(max(min(np.pi / 2, cmd.steering), -np.pi / 2)) * self.K_reduce_speed_from_steer

        target_speed = min(target_speed_goal, target_speed_pitch, target_speed_slope, target_speed_steer)
        cmd.target_speed = self.saturate_target_speed(target_speed)

        acc = cmd.target_speed - state.v
        if acc > 0:
            cmd.acceleration = min(1., acc * self.K_accelerate)
            cmd.brake = 0.
        else:
            cmd.brake = min(1., abs(acc) * self.K_brake)
            cmd.acceleration = 0.

    def saturate_target_speed(self, target_speed):
        return min(max(self.MIN_SPEED, target_speed), self.MAX_SPEED)

    def write_controller_to_file(self):
        f = open(self.filename, 'w')
        f.write('MAX_SPEED: {}\n'.format(self.MAX_SPEED))
        f.write('MIN_SPEED: {}\n'.format(self.MIN_SPEED))
        f.write('K_steer: {}\n'.format(self.K_steer))
        f.write('K_curve: {}\n'.format(self.K_curve))
        f.write('K_pitch: {}\n'.format(self.K_pitch))
        f.write('K_distance: {}\n'.format(self.K_distance))
        f.write('K_reduce_speed_from_steer: {}\n'.format(self.K_reduce_speed_from_steer))
        f.write('K_accelerate: {}\n'.format(self.K_accelerate))
        f.write('K_brake: {}\n'.format(self.K_brake))
        f.write('RPM_GEAR_UP: {}\n'.format(self.RPM_GEAR_UP))
        f.write('RPM_GEAR_DOWN: {}\n'.format(self.RPM_GEAR_DOWN))
        f.close()

    def print_controller(self):
        with open(self.filename, 'r') as f:
            line = f.readline()
            while line:
                print line
                line = f.readline()