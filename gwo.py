import random
import numpy
import math
from solution import solution
import time
import floki_3
import csv

    

def GWO(P, I, D, lb, ub, dim, SearchAgents_no,ite,samples):
    
    PID = [P, I, D]
    floki = floki_3.FLOKI(P, I, D)
    
    # initialize alpha, beta, and delta_pos
    Alpha_pos=numpy.zeros(dim)
    Alpha_score=float("inf")
    
    Beta_pos=numpy.zeros(dim)
    Beta_score=float("inf")
    
    Delta_pos=numpy.zeros(dim)
    Delta_score=float("inf")

    iBest=numpy.zeros(dim)
    iBestScore=float("inf")

    last_gbest=float("inf")
    bestVar=float("inf")

    if not isinstance(lb, list):
        lb = [lb] * dim
        for i in range(dim):
            lb[i] = PID[i] - PID[i]*lb[i]
    if not isinstance(ub, list):
        ub = [ub] * dim
        for i in range(dim):
            ub[i] = PID[i] - PID[i]*ub[i]

    
    #Initialize the positions of search agents
    pos = numpy.zeros((SearchAgents_no, dim))
    for i in range(dim):
        pos[:, i] = numpy.random.uniform(ub[i],lb[i], SearchAgents_no)
    
    convergence_curve=numpy.zeros(ite)
    kp_curve=numpy.zeros(ite)
    ki_curve=numpy.zeros(ite)
    kd_curve=numpy.zeros(ite)
    var_curve=numpy.zeros(ite)
    s=solution()

     # Loop counter
    print("GWO is optimizing  Floki")    
    
    timerStart=time.time() 
    s.startTime=time.strftime("%Y-%m-%d-%H-%M-%S")
    # Main loop
    for l in range(0,ite):
        for i in range(0,SearchAgents_no):
            
            print(pos[i,:])
            # Calculate objective function for each search agent
            floki.controle(float(pos[i,0]),float(pos[i,1]),float(pos[i,2]),samples)
            fitness = floki.IAE
            var = floki.var
            
            if(iBestScore>fitness):
                iBestScore=fitness
                iBest = pos[i,:].copy()

            # Update Alpha, Beta, and Delta
            if fitness<Alpha_score :
                Alpha_score=fitness; # Update alpha
                Alpha_pos=pos[i,:].copy()
                kp_curve[l]=pos[i,0]
                ki_curve[l]=pos[i,1]
                kd_curve[l]=pos[i,2]
                bestVar=var
            
            
            if (fitness>Alpha_score and fitness<Beta_score ):
                Beta_score=fitness  # Update beta
                Beta_pos=pos[i,:].copy()
            
            
            if (fitness>Alpha_score and fitness>Beta_score and fitness<Delta_score): 
                Delta_score=fitness # Update delta
                Delta_pos=pos[i,:].copy()
            
        
        
        
        a=2-l*((2)/ite); # a decreases linearly fron 2 to 0
        
        # Update the Position of search agents including omegas
        for i in range(0,SearchAgents_no):
            floki.controle(iBest[0],iBest[1],iBest[2],1)
            for j in range (0,dim):   
                           
                r1=random.random() # r1 is a random number in [0,1]
                r2=random.random() # r2 is a random number in [0,1]
                
                A1=2*a*r1-a; # Equation (3.3)
                C1=2*r2; # Equation (3.4)
                
                D_alpha=abs(C1*Alpha_pos[j]-pos[i,j]); # Equation (3.5)-part 1
                X1=Alpha_pos[j]-A1*D_alpha; # Equation (3.6)-part 1
                           
                r1=random.random()
                r2=random.random()
                
                A2=2*a*r1-a; # Equation (3.3)
                C2=2*r2; # Equation (3.4)
                
                D_beta=abs(C2*Beta_pos[j]-pos[i,j]); # Equation (3.5)-part 2
                X2=Beta_pos[j]-A2*D_beta; # Equation (3.6)-part 2       
                
                r1=random.random()
                r2=random.random() 
                
                A3=2*a*r1-a; # Equation (3.3)
                C3=2*r2; # Equation (3.4)
                
                D_delta=abs(C3*Delta_pos[j]-pos[i,j]); # Equation (3.5)-part 3
                X3=Delta_pos[j]-A3*D_delta; # Equation (3.5)-part 3             
                
                pos[i,j]=(X1+X2+X3)/3  # Equation (3.7)
                
            
        
        
        convergence_curve[l]=Alpha_score
        var_curve[l]=bestVar
        iBestScore=float("inf")
        if last_gbest == Alpha_score:
            kp_curve[l]=kp_curve[l-1]
            ki_curve[l]=ki_curve[l-1]
            kd_curve[l]=kd_curve[l-1]
        
        last_gbest = Alpha_score

        if (l%1==0):
            print(['At iteration '+ str(l+1)+ ' the best fitness is '+ str(Alpha_score)]);
    
    timerEnd=time.time()  
    s.endTime=time.strftime("%Y-%m-%d-%H-%M-%S")
    s.executionTime=timerEnd-timerStart
    s.convergence=convergence_curve
    s.optimizer="GWO"
    s.objfname="Floki"
    s.kp_convergence = kp_curve
    s.ki_convergence = ki_curve
    s.kd_convergence = kd_curve
    s.kp = kp_curve[-1]
    s.ki = ki_curve[-1]
    s.kd = kd_curve[-1]
    s.var = var_curve

    return s
    
P = 16.5
I = 0.163
D = 0.04075
iters = 50
amost = 10
gwo = GWO(P, I, D, -0.1, 0.1, 3, 6, iters, amost)
print("Otimizacao feita")
ExportToFile="experiment"+time.strftime("%Y-%m-%d-%H-%M-%S")+".csv" 
Flag = False
Export=True
if(Export==True):
    with open(ExportToFile, 'a') as out:
        writer = csv.writer(out,delimiter=' ')
        if (Flag==False): # just one time to write the header of the CSV file
            for i in range(0,iters):
                header= [i,gwo.kp_convergence[i], gwo.ki_convergence[i], gwo.kd_convergence[i], gwo.convergence[i]]
                writer.writerow(header)
    out.close()
    Flag = True
