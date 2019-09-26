import random
import numpy as np
from ga_agent import Agent


def ga_optimize(agents, generations, population, selection_amount=0.2, mutate_probability=0.3):

    for generation in range(generations):

        print ('Generation: ' + str(generation))

        agents = fitness(agents, generation)
        agents = selection(agents, selection_amount)
        agents = crossover(agents, population)
        agents = mutation(agents, mutate_probability)
    return agents


def fitness(agents, generation):
    for i, agent in enumerate(agents):
        agent.calc_fitness(i, generation)
    return agents


def selection(agents, selection_amount):
    agents = sorted(agents, key=lambda agent: agent.fitness, reverse=True)
    print '\n'.join(map(str, agents))
    agents = agents[:int(selection_amount * len(agents))]
    return agents


def crossover(agents, population):
    offspring = []
    for _ in range(population - len(agents)):
        parent1 = random.choice(agents)
        parent2 = random.choice(agents)
        alpha = random.random()
        child = parent1.crossover(parent2, alpha)
        offspring.append(child)

    agents.extend(offspring)

    return agents


def mutation(agents, mutation_prob):

    for i, agent in enumerate(agents):
        if i == 0:
            continue
        if random.random() < mutation_prob:
            agent.mutate()
    return agents


if __name__ == '__main__':
    class MyAgent(Agent):
        def calc_fitness(self):
            asdf = [3, 10]
            dist = np.sqrt(sum([(asdf[i] - self.genes[i]) ** 2 for i in range(2)]))
            if dist == 0:
                self.fitness = 100.
            else:
                self.fitness = max(100. - dist, 0)

        # def crossover(self, other, alpha):
        #     new_agent = MyAgent()
        #     new_agent.constraints = self.constraints
        #     for i in range(len(self.genes)):
        #         if random.random() < alpha:
        #             new_agent.genes.append(other.genes[i])
        #         else:
        #             new_agent.genes.append(self.genes[i])
        #     return new_agent

    gens = 200
    pops = 10
    agents = [MyAgent(2, ((-50, 50), (-8, 120))) for _ in range(pops)]
    ga_optimize(agents, gens, pops, selection_amount=0.3, mutate_probability=0.7)

