class solution:
    def __init__(self):
        self.best = 0
        self.bestIndividual=[]
        self.convergence = []
        self.optimizer=""
        self.objfname=""
        self.startTime=0
        self.endTime=0
        self.executionTime=0
        self.lb=0
        self.ub=0
        self.dim=0
        self.popnum=0
        self.maxiers=0
        self.kp=0
        self.ki=0
        self.kd=0
        self.kp_convergence = []
        self.ki_convergence = []
        self.kd_convergence = []
        self.var = []
