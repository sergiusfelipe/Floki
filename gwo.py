import random
import numpy
import math
from solution import solution
import time
import floki_3
import csv
    

def GWO(P, I, D, lb, ub, dim, SearchAgents_no, ite):
    
    PID = [P, I, D]
    floki = floki_3.FLOKI(P, I, D, samples=3)

    #Max_iter=1000
    #lb=-100
    #ub=100
    #dim=30  
    #SearchAgents_no=5
    
    # initialize alpha, beta, and delta_pos
    Alpha_pos=numpy.zeros(dim)
    Alpha_score=float("inf")
    
    Beta_pos=numpy.zeros(dim)
    Beta_score=float("inf")
    
    Delta_pos=numpy.zeros(dim)
    Delta_score=float("inf")

    if not isinstance(lb, list):
        lb = [lb] * dim
        for i in range(dim):
            lb[i] = PID[i] - PID[i]*lb[i]
    if not isinstance(ub, list):
        ub = [ub] * dim
        for i in range(dim):
            ub[i] = PID[i] - PID[i]*ub[i]
    
    #Initialize the positions of search agents
    Positions = numpy.zeros((SearchAgents_no, dim))
    for i in range(dim):
        Positions[:, i] = numpy.random.uniform(0,1, SearchAgents_no) * (ub[i] - lb[i]) + lb[i]
    
    Convergence_curve=[]
    s=solution()
    kp_curve=numpy.zeros(ite)
    ki_curve=numpy.zeros(ite)
    kd_curve=numpy.zeros(ite)

     # Loop counter
    print("GWO is optimizing  Floki")    
    
    timerStart=time.time() 
    s.startTime=time.strftime("%Y-%m-%d-%H-%M-%S")
    # Main loop
    for l in range(0,ite):
        for i in range(0,SearchAgents_no):
            
            # Return back the search agents that go beyond the boundaries of the search space
            for j in range(dim):
                Positions[i,j]=numpy.clip(Positions[i,j], lb[j], ub[j])

            # Calculate objective function for each search agent
            fitness = floki.controle(pos[i,0],pos[i,1],pos[i,2])
            
            # Update Alpha, Beta, and Delta
            if fitness<Alpha_score :
                Alpha_score=fitness; # Update alpha
                Alpha_pos=Positions[i,:].copy()
                kp_curve[l]=Positions[i,0]
                ki_curve[l]=positions[i,1]
                kd_curve[l]=Positions[i,2]
            
            
            if (fitness>Alpha_score and fitness<Beta_score ):
                Beta_score=fitness  # Update beta
                Beta_pos=Positions[i,:].copy()
            
            
            if (fitness>Alpha_score and fitness>Beta_score and fitness<Delta_score): 
                Delta_score=fitness # Update delta
                Delta_pos=Positions[i,:].copy()
            
        
        
        
        a=2-l*((2)/ite); # a decreases linearly fron 2 to 0
        
        # Update the Position of search agents including omegas
        for i in range(0,SearchAgents_no):
            for j in range (0,dim):     
                           
                r1=random.random() # r1 is a random number in [0,1]
                r2=random.random() # r2 is a random number in [0,1]
                
                A1=2*a*r1-a; # Equation (3.3)
                C1=2*r2; # Equation (3.4)
                
                D_alpha=abs(C1*Alpha_pos[j]-Positions[i,j]); # Equation (3.5)-part 1
                X1=Alpha_pos[j]-A1*D_alpha; # Equation (3.6)-part 1
                           
                r1=random.random()
                r2=random.random()
                
                A2=2*a*r1-a; # Equation (3.3)
                C2=2*r2; # Equation (3.4)
                
                D_beta=abs(C2*Beta_pos[j]-Positions[i,j]); # Equation (3.5)-part 2
                X2=Beta_pos[j]-A2*D_beta; # Equation (3.6)-part 2       
                
                r1=random.random()
                r2=random.random() 
                
                A3=2*a*r1-a; # Equation (3.3)
                C3=2*r2; # Equation (3.4)
                
                D_delta=abs(C3*Delta_pos[j]-Positions[i,j]); # Equation (3.5)-part 3
                X3=Delta_pos[j]-A3*D_delta; # Equation (3.5)-part 3             
                
                Positions[i,j]=(X1+X2+X3)/3  # Equation (3.7)
                
            
        
        
        Convergence_curve.append(Alpha_score);

        if (l%1==0):
               print(['At iteration '+ str(l)+ ' the best fitness is '+ str(Alpha_score)]);
    
    timerEnd=time.time()  
    s.endTime=time.strftime("%Y-%m-%d-%H-%M-%S")
    s.executionTime=timerEnd-timerStart
    s.convergence=convergence_curve
    s.optimizer="PSO"
    s.objfname="Floki"
    s.kp_convergence = kp_curve
    s.ki_convergence = ki_curve
    s.kd_convergence = kd_curve
    s.kp = kp_curve[-1]
    s.ki = ki_curve[-1]
    s.kd = kd_curve[-1]
    
    return s


P = 20
I = 4.5
D = 0.001
iters = 50
gwo = GWO(P, I, D, -1.5, 1.5, 3, 50, iters)
print("Otimizacao feita")
Flag = False
Export=True
if(Export==True):
    with open(ExportToFile, 'a',newline='\n') as out:
        writer = csv.writer(out,delimiter=' ')
        if (Flag==False): # just one time to write the header of the CSV file
            for i in range(0,iters):
                header= numpy.concatenate([i,pso.kp_convergence[i], pso.ki_convergence[i], pso.kd_convergence[i], pso.convergence[i]])
                writer.writerow(header)
    out.close()
    Flag = True

if Flag == True:
    while True:
        floki.controle(pso.kp, pso.ki, pso.kd)
