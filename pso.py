import random
import numpy
import math
from colorama import Fore, Back, Style
from solution import solution
import time
import floki_3






def PSO(P, I, D, lb,ub,dim,pop):


    PID = [P, I, D]
    # PSO parameters
    floki = FLOKI.FLOKI(P, I, D)
    #    dim=30
    #    iters=200
    Vmax=6
    #    PopSize=50     #population size
    wMax=0.9
    wMin=0.2
    c1=2
    c2=2
    #    lb=-10
    #    ub=10
    #    
    s=solution()
    if not isinstance(lb, list):
        lb = [lb] * dim
        for i in range(dim):
            lb[i] = PID[i] - PID[i]*lb[i]
    if not isinstance(ub, list):
        ub = [ub] * dim
        for i in range(dim):
            ub[i] = PID[i] - PID[i]*ub[i]
    
    
    ######################## Initializations
    
    vel=numpy.zeros((PopSize,dim))
    
    pBestScore=numpy.zeros(PopSize) 
    pBestScore.fill(float("inf"))
    
    pBest=numpy.zeros((PopSize,dim))
    gBest=numpy.zeros(dim)
    
    
    gBestScore=float("inf")

    pos = numpy.zeros((pop, dim))
    for i in range(dim):
        pos[:, i] = numpy.random.uniform(0,1, pop) * (ub[i] - lb[i]) + lb[i]
    
    convergence_curve=[]
    
    ############################################
    print("PSO is optimizing Floki")    
    
    timerStart=time.time() 
    s.startTime=time.strftime("%Y-%m-%d-%H-%M-%S")
    
    #print("lb: ",lb)
    #print("ub: ",ub)
    #print("pos: ",pos)
    
    while True:
        for i in range(0,PopSize):
            #pos[i,:]=checkBounds(pos[i,:],lb,ub)
            for j in range(dim):
                pos[i, j] = numpy.clip(pos[i,j], lb[j], ub[j])
            #Calculate objective function for each particle
            fitness = floki.controle(pos[i,:])
    
            if(pBestScore[i]>fitness):
                pBestScore[i]=fitness
                pBest[i,:]=pos[i,:].copy()
                
            if(gBestScore>fitness):
                gBestScore=fitness
                gBest=pos[i,:].copy()
        
        #Update the W of PSO
        w=wMax-l*((wMax-wMin)/iters);
        
        for i in range(0,PopSize):
            for j in range (0,dim):
                r1=random.random()
                r2=random.random()
                vel[i,j]=w*vel[i,j]+c1*r1*(pBest[i,j]-pos[i,j])+c2*r2*(gBest[j]-pos[i,j])
                
                if(vel[i,j]>Vmax):
                    vel[i,j]=Vmax
                
                if(vel[i,j]<-Vmax):
                    vel[i,j]=-Vmax
                            
                pos[i,j]=pos[i,j]+vel[i,j]
        
        convergence_curve.append(gBestScore)
      
        if (l%1==0):
               print(['At iteration '+ str(l+1)+ ' the best fitness is '+ str(gBestScore)]);
    timerEnd=time.time()  
    s.endTime=time.strftime("%Y-%m-%d-%H-%M-%S")
    s.executionTime=timerEnd-timerStart
    s.convergence=convergence_curve
    s.optimizer="PSO"
    s.objfname="Floki"


pso = PSO(P, I, D, -1.5, 1.5, 3, 50)
