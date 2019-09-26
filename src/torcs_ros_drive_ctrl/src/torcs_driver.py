#!/usr/bin/env python
import rospy
from torcs_msgs.msg import TORCSCtrl, TORCSSensors
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped, TwistStamped, PoseStamped, Twist, Vector3Stamped
import matplotlib.pyplot as plt
from threading import Lock
from car_controller import *
from logger import Logger


TRACK_OPPONENT = False


class Driver:
    def get_distance(self):
        return self._state.distance_passed

    def __init__(self, add_postfix=''):
        self.logger = Logger(add_postfix=add_postfix)
        self._lanes = [[0]*3, [0]*3]
        self._is_stuck = False
        self.controller = Controller(add_postfix=add_postfix)
        self.controller.setup_params()
        self._lock = Lock()
        rospy.init_node('torcs_driver')
        self.local_point = PointStamped()
        self._state = State()
        self._emg = [0., 0.]
        self._path = Path()
        self.ctrl = rospy.Publisher('/torcs_ros/ctrl_cmd', TORCSCtrl, queue_size=1)
        self.local_point.header.frame_id = 'base_link'
        self.local_point_pub = rospy.Publisher('/local_point', PointStamped, queue_size=1)
        self.local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        rospy.Subscriber('/torcs_ros/scan_track', LaserScan, self.track_callback)
        rospy.Subscriber('/torcs_ros/scan_opponents', LaserScan, self.opp_callback)
        rospy.Subscriber('/torcs_ros/sensors_state', TORCSSensors, self.sensor_callback)
        rospy.Subscriber('/torcs_ros/speed', TwistStamped, self.speed_callback)
        rospy.Subscriber('/torcs_ros/torcs_global_rpy', Vector3Stamped, self.rpy_callback)
        rospy.Subscriber('/torcs_ros/torcs_global_pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/cmd_vel', Twist, self.user_callback)
        self._last_user_msg_time = rospy.Time(0)
        self._user_input = Twist()

    def pose_callback(self, msg=PoseStamped()):
        self._state.x = msg.pose.position.x
        self._state.y = msg.pose.position.y

    def rpy_callback(self, msg=Vector3Stamped()):
        self._state.pitch = msg.vector.y

    def stop(self):
        ctrl = TORCSCtrl()
        ctrl.gear = 0
        ctrl.accel = 0.0
        ctrl.brake = 1.
        self.ctrl.publish(ctrl)

    def drive_back(self):
        ctrl = TORCSCtrl()
        flip = 1. if abs(self._emg[1]) < np.pi / 2 else -1
        if self._emg[0] < 0:
            """ Right to track """
            ctrl.gear = int(1 * flip)
            ctrl.accel = 0.3
            ctrl.steering = 1. * flip
        else:
            """ Right to track """
            ctrl.gear = int(1 * flip)
            ctrl.accel = 0.3
            ctrl.steering = -1. * flip
        rospy.loginfo_throttle(1, 'Back to track')
        self.ctrl.publish(ctrl)

    def user_callback(self, msg=Twist()):
        self._last_user_msg_time = rospy.Time.now()
        self._user_input = msg

    def sensor_callback(self, msg=TORCSSensors):
        self._state.rpm = msg.rpm
        self._state.gear = msg.gear
        self._emg = [msg.trackPos, msg.angle]
        self._state.distance_passed = msg.distRaced

    def speed_callback(self, msg=TwistStamped()):
        self._state.v = msg.twist.linear.x

    def track_callback(self, msg=LaserScan()):
        if TRACK_OPPONENT:
            return
        with self._lock:
            if len(list(filter(lambda x: x != -1, msg.ranges))) < 2:
                self._is_stuck = True
                return
            self._is_stuck = False
            angle = msg.angle_min - msg.angle_increment
            i = 1
            x_avg = 0.
            y_avg = 0.
            n = len(msg.ranges)
            self._path.header.stamp = rospy.Time.now()
            self._path.header.frame_id = 'base_link'
            self._path.poses = []
            while angle <= msg.angle_max and i < n:
                angle = msg.angle_min + msg.angle_increment * i
                x_i = msg.ranges[i] * np.cos(angle)
                y_i = msg.ranges[i] * np.sin(angle)

                x_avg += x_i
                y_avg += y_i
                point = PoseStamped()
                point.pose.position.y = (y_i)
                point.pose.position.x = (x_i)
                point.pose.orientation.w = 1
                self._path.poses.append(point)
                i += 1

            xs = [point.pose.position.x for point in self._path.poses]
            ys = [point.pose.position.y for point in self._path.poses]
            idx, idx2, poly1, poly2 = find_lanes(xs, ys)
            # draw_lanes(xs, ys, idx, idx2, poly1, poly2)
            # self.local_path_pub.publish(self._path)
            self.local_point.header.stamp = rospy.Time.now()
            self.local_point.point.x = x_avg / len(msg.ranges)
            self.local_point.point.y = y_avg / len(msg.ranges)
            # self.local_point_pub.publish(self.local_point)
            self._state.lanes = [poly1, poly2, [idx], [idx2]]
            self._state.goal = [self.local_point.point.x, self.local_point.point.y]

    def opp_callback(self, msg=LaserScan()):
        if not TRACK_OPPONENT:
            return
        angle = msg.angle_min - msg.angle_increment
        self.local_point.point.x = 0.
        self.local_point.point.y = 0.
        i = -1
        while angle < msg.angle_max and i < len(msg.ranges) - 1:
            i += 1
            angle += msg.angle_increment
            if msg.ranges[i] == msg.range_min or msg.ranges[i] == msg.range_max:
                continue
            if msg.ranges[i] < 10:
                continue
            x_i = msg.ranges[i] * np.cos(angle)
            y_i = msg.ranges[i] * np.sin(angle)
            self.local_point.point.x = x_i
            self.local_point.point.y = y_i
            break

        self.local_point.header.stamp = rospy.Time.now()

        self.local_point_pub.publish(self.local_point)

    def drive_cmd(self):
        if self._is_stuck:
            self.drive_back()
            return
        ctrl = TORCSCtrl()
        cmd = self.controller.control(self._state)
        self.logger.write_to_file(self._state, cmd)
        ctrl.brake = cmd.brake
        ctrl.accel = cmd.acceleration
        ctrl.gear = max(min(6, int(cmd.gear)), 1)
        ctrl.steering = cmd.steering
        self.ctrl.publish(ctrl)

    def spin_once(self):
        rospy.sleep(0.01)
        with self._lock:
            self.drive_cmd()

    def spin(self):
        while not rospy.is_shutdown():
            self.spin_once()
        for i in range(5):
            self.stop()
            rospy.sleep(0.02)


def find_lanes(xs, ys):
    if len(xs) < 4:
        return -1, -1, None, None
    'Find right side '
    last_angle = None
    idx1 = len(xs) - 1
    for i in range(1, len(xs)):
        angle = np.arctan2(ys[i] - ys[i-1], xs[i] - xs[i-1])
        if last_angle is None:
            last_angle = angle
            continue
        if np.sign(angle) != np.sign(last_angle):
            idx1 = i - 1
            break
        last_angle = angle
    ''' find left lane '''
    last_angle = None
    idx2 = 0
    for i in range(len(xs)-1, 0, -1):
        angle = np.arctan2(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
        if last_angle is None:
            last_angle = angle
            continue
        if np.sign(angle) != np.sign(last_angle):
            idx2 = i + 1
            break
        last_angle = angle
    if idx1 < 3:
        p1 = [0, 0, 0]
    else:
        p1 = np.polyfit(xs[:idx1], ys[:idx1], 2)
    if idx2 > len(xs) - 3:
        p2 = [0, 0, 0]
    else:
        p2 = np.polyfit(xs[idx2:], ys[idx2:], 2)
    return idx1, idx2, p1, p2


def draw_lanes(xs, ys, idx, idx2, poly1, poly2):
    if idx == -1 or idx2 == -1:
        return
    plt.clf()
    plt.plot(np.array(ys) * -1., xs, '.b')
    p = np.poly1d(poly1)
    y1 = [p(xs[i]) for i in range(idx)]
    p = np.poly1d(poly2)
    y2 = [p(xs[i]) for i in range(idx2, len(xs))]
    plt.plot(np.array(y1) * -1., xs[:idx], 'r', label=', '.join(['%.3f' % elem for elem in poly1]))
    plt.plot(np.array(y2) * -1., xs[idx2:], 'g', label=', '.join(['%.3f' % elem for elem in poly2]))
    plt.xlabel('SideOfCar[m]')
    plt.ylabel('FrontOfCar[m]')
    plt.legend()
    plt.draw()
    plt.pause(0.0001)


if __name__ == '__main__':
    driver = Driver()
    driver.spin()
