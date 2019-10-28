from logger import *
from car_controller import Controller
analyze_gen_number = -1
analyze_agent = -1


def find_best_agent_for_gen(gen=0):
    agent = 0
    best_agent = (-1, -1)  # agent idx, agent distance
    # print "Gen: ", gen
    while True:
        try:
            log = Logger(read_or_write='r', add_postfix="_agent_{}_{}".format(gen, agent))
            data = log.load()
            distance_passed = np.array(data.tail(1))[0, -1]
            # print "Agent {}: {:.2f}".format(agent, distance_passed)
            if best_agent[1] < distance_passed:
                best_agent = (agent, distance_passed)
            agent += 1
        except:
            break
    return best_agent


def find_best_agent_in_all_gens():
    gen = 0
    best_agent = (-1, -1, -1)
    while True:
        try:
            agent = find_best_agent_for_gen(gen)
            print 'Gen {}: Best agent index is {} with distance of {}'.format(gen, agent[0], agent[1])
            c = Controller(add_postfix="_agent_{}_{}".format(gen, agent[0]))
            c.print_controller()
            if agent[0] == -1:
                break
            if agent[1] > best_agent[1]:
                best_agent = (agent[0], agent[1], gen)
            gen += 1
        except:
            break
    return best_agent, best_agent[2]


if __name__ == '__main__':
    if analyze_gen_number == -1:  # find best agent in all gens
        best_agent, analyze_gen_number = find_best_agent_in_all_gens()
        analyze_agent = best_agent[0]

    if analyze_agent == -1:  # find best agent
        best_agent = find_best_agent_for_gen(analyze_gen_number)
        analyze_agent = best_agent[0]
        print analyze_agent
    print "Best agent of run is _agent_{}_{}".format(analyze_gen_number, analyze_agent)
    log = Logger(read_or_write='r', add_postfix="_agent_{}_{}".format(analyze_gen_number, analyze_agent))
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
