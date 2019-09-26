#!/usr/bin/env python
import rospy
from torcs_driver import Driver
import time
from logger import Logger, np
import subprocess
from ga_optimizer import Agent, ga_optimize


class LearningAgent(Agent):
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

    def params_to_list(self):
        return [self.MAX_SPEED, self.MIN_SPEED, self.K_steer, self.K_curve, self.K_pitch, self.K_distance,
                self.K_reduce_speed_from_steer, self.K_accelerate, self.K_brake, self.RPM_GEAR_UP, self.RPM_GEAR_DOWN]

    def list_to_params(self, param_list):
        self.MAX_SPEED, self.MIN_SPEED, self.K_steer, self.K_curve, self.K_pitch, self.K_distance, \
            self.K_reduce_speed_from_steer, self.K_accelerate, self.K_brake, self.RPM_GEAR_UP, self.RPM_GEAR_DOWN \
            = param_list

    def param_constraints(self):
        return ((130, 300), (10, 150), (0.1, 20), (50, 3000), (0.2, 20), (0.2, 50),
                (0.2, 50), (0.1, 10.), (0.1, 10.), (5000, 7500), (500, 3000))

    def __init__(self):
        Agent.__init__(self, len(self.params_to_list()), self.param_constraints())
        self.sim = None
        self.sim_client = None

    def __del__(self):
        if self.sim_client is not None:
            self.sim_client.kill()
        if self.sim is not None:
            self.sim.kill()

    def set_ros_params(self):
        rospy.set_param('/torcs_driver/MAX_SPEED', self.MAX_SPEED)
        rospy.set_param('/torcs_driver/MIN_SPEED', self.MIN_SPEED)
        rospy.set_param('/torcs_driver/RPM_GEAR_UP', self.RPM_GEAR_UP)
        rospy.set_param('/torcs_driver/RPM_GEAR_DOWN', self.RPM_GEAR_DOWN)
        rospy.set_param('/torcs_driver/K_reduce_speed_from_steer',
                                                         self.K_reduce_speed_from_steer)
        rospy.set_param('/torcs_driver/K_distance', self.K_distance)
        rospy.set_param('/torcs_driver/K_steer', self.K_steer)
        rospy.set_param('/torcs_driver/K_curve', self.K_curve)
        rospy.set_param('/torcs_driver/K_pitch', self.K_pitch)
        rospy.set_param('/torcs_driver/K_accelerate', self.K_accelerate)
        rospy.set_param('/torcs_driver/K_brake', self.K_brake)
        
    def test_driver(self, agent_id, generation):
        self.list_to_params(self.genes)
        self.set_ros_params()
        driver = Driver("_agent_{}_{}".format(generation, agent_id))
        start_time = time.time()
        self.sim_client = subprocess.Popen('roslaunch torcs_ros_bringup torcs_ros.launch rviz:=false driver:=false', shell=True)
        self.sim = subprocess.Popen('torcs -r /home/oded/.torcs/config/raceman/quickrace.xml', shell=True)
        test_duration = 10  # seconds
        while time.time() - start_time < test_duration:
            driver.spin_once()
        distance_passed = driver.get_distance()
        driver.controller.write_controller_to_file()
        # del driver
        # self.sim_client.kill()
        # time.sleep(1)
        # self.sim.kill()
        subprocess.Popen('rosnode kill /torcs_ros/torcs_img_publisher_node', shell=True)
        subprocess.Popen('rosnode kill /torcs_ros/torcs_ros_client_node', shell=True)
        subprocess.Popen('killall -9 torcs-bin', shell=True)
        return distance_passed

    def calc_fitness(self, agent_id, generation):
        distance_passed = self.test_driver(agent_id, generation)
        self.fitness = distance_passed


if __name__ == '__main__':
    gens = 3
    pops = 10
    agents = [LearningAgent() for _ in range(pops)]
    agents = ga_optimize(agents, gens, pops, selection_amount=0.3, mutate_probability=0.7)
    print agents[0].fitness
    print agents[0].genes
    print agents[1].fitness
    print agents[1].genes



