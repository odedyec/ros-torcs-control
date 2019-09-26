import numpy as np
from car_controller import State, Command
import time
from os.path import expanduser
import pandas as pd
log_dict = {'t': 0, 'x':1, 'y': 2, 'v': 3, 'pitch': 4, 'rpm': 5, 'gear': 6, 'goal_x': 7, 'goal_y': 8}


class Logger:
    def __init__(self, read_or_write='w', add_postfix=''):
        home = expanduser('~')
        self.file_name = home + '/.torcs/log/torcs_drive_log' + add_postfix + '.csv'
        self.file_status = read_or_write
        self.f = None

    def __del__(self):
        if self.f is not None:
            self.f.close()

    def write_to_file(self, state=State(), command=Command()):
        if self.f is None:
            self.f = open(self.file_name, self.file_status)
            self.f.write('t, x, y, v, pitch, rpm, gear, goal_x, goal_y, lane_0_0, lane_0_1, lane_0_2, lane_1_0, lane_1_1, lane_1_2, cmd_gear, cmd_steer, cmd_acc, cmd_brake, cmd_target_speed, distance_passed\n')
        string_to_write = ', '.join(list(map(str, [time.time(),
                                                   state.x, state.y, state.v, state.pitch,
                                                   state.rpm, state.gear,
                                                   state.goal[0], state.goal[1],
                                                   state.lanes[0][0], state.lanes[0][1], state.lanes[0][2],
                                                   state.lanes[1][0], state.lanes[1][1], state.lanes[1][2],
                                                   command.gear, command.steering, command.acceleration, command.brake,
                                                   command.target_speed,
                                                   state.distance_passed
                                                   ]
                                             )
                                         )
                                    )
        self.f.write(string_to_write + '\n')

    def load(self):
        data = pd.read_csv(self.file_name)
        return data


if __name__ == '__main__':
    log = Logger('r')
    data = log.load()

    data_np = np.array(data)
    import matplotlib.pyplot as plt
    distance_passed = np.array(data.tail(1))[0, -1]
    print("Passed distance: {}".format(distance_passed))
    x = data_np[:, log_dict['x']]
    y = data_np[:, log_dict['y']]
    plt.plot(x, y, '.b')
    plt.xlabel('x[m]')
    plt.ylabel('y[m]')

    plt.show()

