import numpy as np
import pprint as pp
import random as rand
from operator import attrgetter
import floki_3
from solution import solution
import time
from food_source import FoodSource
import csv


class ABC(object):
    """docstring for ABC"""

    food_sources = []

    def __init__(self, P, I, D, npopulation, nruns, trials_limit, employed_bees_percentage, lb, ub,dim,samples):
        super(ABC, self).__init__()

        self.PID = [P, I, D]
        self.floki = floki_3.FLOKI(P, I, D)
        self.npopulation = npopulation
        self.nruns = nruns
        self.trials_limit = trials_limit
        self.dim = dim
        self.s=solution()
        self.convergence_curve=np.zeros(nruns)
        self.kp_curve=np.zeros(nruns)
        self.ki_curve=np.zeros(nruns)
        self.kd_curve=np.zeros(nruns)
        self.amostras = samples
        self.bestGlobal = float("inf")


        if not isinstance(lb, list):
            self.lb = [lb] * dim
        for i in range(dim):
            self.lb[i] = self.PID[i] - self.PID[i]*self.lb[i]
        if not isinstance(ub, list):
            self.ub = [ub] * dim
        for i in range(dim):
            self.ub[i] = self.PID[i] - self.PID[i]*self.ub[i]

        self.employed_bees = round(npopulation * employed_bees_percentage)
        self.onlooker_bees = npopulation - self.employed_bees

    def optimize(self):
        self.initialize()

        for nrun in range(0, self.nruns):
            self.employed_bees_stage()
            self.onlooker_bees_stage()
            self.scout_bees_stage()
            best_fs = self.best_source()
            self.kp_curve[nrun] = best_fs.solution[0]
            self.ki_curve[nrun] = best_fs.solution[1]
            self.kd_curve[nrun] = best_fs.solution[2]
            self.convergence_curve[nrun] = self.bestGlobal

        best_fs = self.best_source()
        self.s.optimizer="ABC"
        self.s.objfname="Floki"
        self.s.convergence=self.convergence_curve
        self.s.kp_convergence = self.kp_curve
        self.s.ki_convergence = self.ki_curve
        self.s.kd_convergence = self.kd_curve
        self.s.kp = self.kp_curve[-1]
        self.s.ki = self.ki_curve[-1]
        self.s.kd = self.kd_curve[-1]

        return self.s


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
        pos=np.zeros((current_solution_index+1, self.dim))
        for i in range(0, self.dim):
            
            pos[:, i] = np.random.uniform(self.ub[i],self.lb[i], current_solution_index+1)

        solution = pos[0,:] 
        return solution

    def random_solution_excluding(self, excluded_index):
        available_indexes = set(range(self.employed_bees))
        exclude_set = set(excluded_index)
        diff = available_indexes - exclude_set
        selected = rand.choice(list(diff))

        return selected

    def best_solution(self, current_solution, new_solution):
        current = self.fitness(current_solution)
        new = self.fitness(new_solution)
        if new > current:
                if self.bestGlobal > current:
                        self.bestGlobal = current
                return current_solution
        else:
                if self.bestGlobal > new:
                        self.bestGlobal = new
                return new_solution

    def probability(self, solution_fitness):
        fitness_sum = sum([fs.fitness for fs in self.food_sources])
        probability = solution_fitness.fitness / fitness_sum

        return probability

    def fitness(self, solution):
        self.floki.controle(solution[0], solution[1], solution[2], self.amostras)
        fitness = self.floki.IAE
        print(fitness)
        return float(fitness)

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



P = 16.5
I = 0.163
D = 0.04075
iters = 50
amost = 5
abc = ABC(P, I, D,10,iters,50,0.5, -0.1, 0.1, 3, amost)
solution = abc.optimize()
print("Otimizacao feita")
print(solution.convergence)
ExportToFile="experiment"+time.strftime("%Y-%m-%d-%H-%M-%S")+".csv" 
Flag = False
Export=True
if(Export==True):
    with open(ExportToFile, 'a') as out:
        writer = csv.writer(out,delimiter=' ')
        if (Flag==False): 
            for k in range(0,iters):
                header= [k,solution.kp_convergence[k], solution.ki_convergence[k], solution.kd_convergence[k], solution.convergence[k]]
                writer.writerow(header)
    out.close()
    Flag = True

    
