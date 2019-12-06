import numpy as np
import pprint as pp
import random as rand
from operator import attrgetter
import floki_3

from food_source import FoodSource


class ABC(object):
    """docstring for ABC"""

    food_sources = []

    def __init__(self, P, I, D, npopulation, nruns, trials_limit=50, employed_bees_percentage=0.5, lb, ub,dim):
        super(ABC, self).__init__()

        self.PID = [P, I, D]
        self.floki = floki_3.FLOKI(P, I, D)
        self.npopulation = npopulation
        self.nruns = nruns
        self.trials_limit = trials_limit
        self.dim = dim


        if not isinstance(lb, list):
            self.lb = [lb] * dim
        for i in range(dim):
            self.lb[i] = self.PID[i] - self.PID[i]*lb[i]
        if not isinstance(ub, list):
            self.ub = [ub] * dim
        for i in range(dim):
            self.ub[i] = self.PID[i] - self.PID[i]*ub[i]

        self.employed_bees = round(npopulation * employed_bees_percentage)
        self.onlooker_bees = npopulation - self.employed_bees

    def optimize(self):
        self.initialize()
        pp.pprint(self.food_sources)

        for nrun in range(1, self.nruns+1):
            self.employed_bees_stage()
            self.onlooker_bees_stage()
            self.scout_bees_stage()

        pp.pprint(self.food_sources)

        best_fs = self.best_source()

        return best_fs.solution


    def initialize(self):
        self.food_sources = [self.create_foodsource() for i in range(self.employed_bees)]

    def employed_bees_stage(self):
        for i in range(self.employed_bees):
            food_source = self.food_sources[i]
            new_solution = self.generate_solution(i)
            best_solution = self.best_solution(food_source.solution, new_solution)

            self.set_solution(food_source, best_solution)

    def onlooker_bees_stage(self):
        for i in range(self.onlooker_bees):
            probabilities = [self.probability(fs) for fs in self.food_sources]
            selected_index = self.selection(range(len(self.food_sources)), probabilities)
            selected_source = self.food_sources[selected_index]
            new_solution = self.generate_solution(selected_index)
            best_solution = self.best_solution(selected_source.solution, new_solution)

            self.set_solution(selected_source, best_solution)

    def scout_bees_stage(self):
        for i in range(self.employed_bees):
            food_source = self.food_sources[i]

            if food_source.trials > self.trials_limit:
                food_source = self.create_foodsource()


    def generate_solution(self, current_solution_index):
        pos=np.zeros((self.npopulation, self.dim))
        for i in range(self.dim):
            
            pos[:, i] = np.random.uniform(0,1, self.npopulation) * (self.ub[i] - self.lb[i]) + self.lb[i]


        return pos[current_solution_index,:]

    def random_solution_excluding(self, excluded_index):
        available_indexes = set(range(self.employed_bees))
        exclude_set = set(excluded_index)
        diff = available_indexes - exclude_set
        selected = rand.choice(list(diff))

        return selected

    def best_solution(self, current_solution, new_solution):
        if self.fitness(new_solution) > self.fitness(current_solution):
            return new_solution
        else:
            return current_solution

    def probability(self, solution_fitness):
        fitness_sum = sum([fs.fitness for fs in self.food_sources])
        probability = solution_fitness.fitness / fitness_sum

        return probability

    def fitness(self, solution):

        fitness = self.floki.controle(solution[0], solution[1], solution[2])

        return fitness

    def selection(self, solutions, weights):
        return rand.choices(solutions, weights)[0]

    def set_solution(self, food_source, new_solution):
        if np.array_equal(new_solution, food_source.solution):
            food_source.trials += 1
        else:
            food_source.solution = new_solution
            food_source.trials = 0

    def best_source(self):
        best = min(self.food_sources, key=attrgetter('fitness'))

        return best

    def create_foodsource(self):
        solution = self.generate_solution(1)
        fitness = self.fitness(solution)

        return FoodSource(solution, fitness)



P = 27.5
I = 0.163
D = 0.0407
abc = ABC(P, I, D, 50, 50, 50, 0.5, -1.5, 1.5, 3)
print("Otimizacao feita")
while True:
        self.floki.controle(abc[0], abc[1], abc[2])

    
