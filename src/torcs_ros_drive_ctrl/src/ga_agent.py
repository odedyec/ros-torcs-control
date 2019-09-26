import random
import numpy as np


class Agent:
    def __init__(self, num_of_genes=0, constraints=tuple()):
        self.fitness = -1
        self.genes = []
        self.constraints = constraints

        if num_of_genes == 0:
            return

        if len(constraints) != num_of_genes or len(constraints[0]) != 2:
            raise Exception("The constraints should be a vector of length {} of form [(min, max), ..., (min, max)] ".format(num_of_genes))

        for gene_num in range(num_of_genes):
            self.genes.append(random.uniform(constraints[gene_num][0],
                                             constraints[gene_num][1]
                                             )
                              )

    def __str__(self):
        return "genes = (" + ", ".join(map(str, self.genes)) + ")   | fitness = " + str(self.fitness)

    def __mul__(self, other):
        new_agent = self.__class__()
        new_agent.fitness = self.fitness
        new_agent.constraints = self.constraints
        if type(other) is float:
            new_agent.genes = [gene * other for gene in self.genes]
        else:
            raise Exception("No mul option for type except float")
        return new_agent

    def __add__(self, other):
        new_agent = self.__class__()
        new_agent.fitness = -1
        new_agent.constraints = self.constraints
        new_agent.genes = [a + b for a, b in zip(self.genes, other.genes)]
        return new_agent

    def calc_fitness(self, agent_id, generation):
        self.fitness = -1

    def set_fitness(self, fitness):
        self.fitness = fitness

    def mutate(self):
        self.genes = []
        for gene_num in range(len(self.constraints)):
            self.genes.append(random.uniform(self.constraints[gene_num][0],
                                             self.constraints[gene_num][1]
                                             )
                              )

    def crossover(self, other, alpha):
        if not 0. < alpha < 1.:
            raise Exception("Crossover fusion should be with value alpha in [0, 1]. Got alpha {}".format(alpha))
        new_agent = self * alpha + other * (1 - alpha)
        return new_agent


if __name__ == '__main__':
    agent = Agent(2, ((1, 10), (-10, 10)))
    print agent
    other = agent * 4.
    print other
    new = other + agent
    print new
    new.mutate()
    print new
    child = agent.crossover(other, 0.5)
    print child


