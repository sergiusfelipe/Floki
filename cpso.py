import random
import numpy
import math
from colorama import Fore, Back, Style
from solution import solution
import time
import floki_3
import csv






def CPSO(P, I, D, lb,ub,dim,pop,ite, samples):


    PID = [P, I, D]
    floki = floki_3.FLOKI(P, I, D)
    Vmax=6
    wMax=0.9
    wMin=0.2
    c1=2
    c2=2   
    s=solution()
    cr=random.random() #inicializacao do cr
    r = 4 #constante caotico

    if not isinstance(lb, list):
        lb = [lb] * dim
        for i in range(dim):
            lb[i] = PID[i] - PID[i]*lb[i]
    if not isinstance(ub, list):
        ub = [ub] * dim
        for i in range(dim):
            ub[i] = PID[i] - PID[i]*ub[i]
    
        
    vel=numpy.zeros((pop,dim))
    
    pBestScore=numpy.zeros(pop) 
    pBestScore.fill(float("inf"))
    
    pBest=numpy.zeros((pop,dim))
    gBest=numpy.zeros(dim)
    iBest=numpy.zeros(dim)
    
    
    gBestScore=float("inf")
    bestVar=float("inf")
    iBestScore=float("inf")
    last_gbest=float("inf")

    pos = numpy.zeros((pop, dim))
    for i in range(dim):
        pos[:, i] = numpy.random.uniform(ub[i],lb[i], pop)
    
    convergence_curve=numpy.zeros(ite)
    kp_curve=numpy.zeros(ite)
    ki_curve=numpy.zeros(ite)
    kd_curve=numpy.zeros(ite)
    var_curve=numpy.zeros(ite)
    
    print("CPSO is optimizing Floki")    
    
    timerStart=time.time() 
    s.startTime=time.strftime("%Y-%m-%d-%H-%M-%S")

    f=1
    sum_ite =0
    
    for l in range(0,ite):
        teste_ite = time.time()
        k=1
        sum_pop = 0
        sum_vel = 0
        h = 1
        for i in range(0,pop):
            teste_pop = time.time()
            #Calculate objective function for each particle
            print(pos[i,:])
            floki.controle(float(pos[i,0]),float(pos[i,1]),float(pos[i,2]),samples)
            fitness = floki.IAE
            var = floki.var

            if(iBestScore>fitness):
                iBestScore=fitness
                iBest = pos[i,:].copy()
                

            if(pBestScore[i]>fitness):
                pBestScore[i]=fitness
                pBest[i,:]=pos[i,:].copy()
                
            if(gBestScore>fitness):
                gBestScore=fitness
                gBest=pos[i,:].copy()
                bestVar=var
                kp_curve[l]=pos[i,0]
                ki_curve[l]=pos[i,1]
                kd_curve[l]=pos[i,2]
            delta_pop = time.time() - teste_pop
            sum_pop += delta_pop
            print("Execucao pop: ", sum_pop/k)
            k+=1
        #Update the W of PSO
        w=wMax-l*((wMax-wMin)/ite);
        
        for i in range(0,pop):
            teste_vel = time.time()
            floki.controle(iBest[0],iBest[1],iBest[2],1)
            for j in range (0,dim):
                cr = r*cr*(1-cr)
                vel[i,j]=w*vel[i,j]+c1*cr*(pBest[i,j]-pos[i,j])+c2*(1-cr)*(gBest[j]-pos[i,j])
                
                if(vel[i,j]>Vmax):
                    vel[i,j]=Vmax
                
                if(vel[i,j]<-Vmax):
                    vel[i,j]=-Vmax
                            
                pos[i,j]=pos[i,j]+vel[i,j]
            delta_vel = time.time() - teste_vel
            sum_vel += delta_vel
            print("Execucao velocidade: ", sum_vel/h)
            h +=1
        
        convergence_curve[l]=gBestScore
        var_curve[l]=bestVar
        delta_ite = time.time() - teste_ite
        sum_ite += delta_ite
        print("Execucao por iteracao: ", sum_ite/f)
        f += 1
        iBestScore=float("inf")
        if last_gbest == gBestScore:
            kp_curve[l]=kp_curve[l-1]
            ki_curve[l]=ki_curve[l-1]
            kd_curve[l]=kd_curve[l-1]
        
        last_gbest = gBestScore
      
        if (l%1==0):
               print(['At iteration '+ str(l+1)+ ' the best fitness is '+ str(gBestScore)]);
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
    s.var = var_curve

    return s
P = 16.5
I = 0.163
D = 0.04075
iters = 50
amost = 10
cpso = CPSO(P, I, D, -0.1, 0.1, 3, 10, iters, amost)
print("Otimizacao feita")
ExportToFile="experiment"+time.strftime("%Y-%m-%d-%H-%M-%S")+".csv" 
Flag = False
Export=True
if(Export==True):
    with open(ExportToFile, 'a') as out:
        writer = csv.writer(out,delimiter=' ')
        if (Flag==False): 
            for i in range(0,iters):
                header= [i,cpso.kp_convergence[i], cpso.ki_convergence[i], cpso.kd_convergence[i], cpso.convergence[i],cpso.var[i]]
                writer.writerow(header)
    out.close()
    Flag = True
'''
floki = floki_3.FLOKI(P,I,D,amost)
teste = time.time()
i=1
sum =0
while True:
    pi = time.time()
    floki.controle(P,I,D)
    delta_time = time.time() - pi
    sum += delta_time
    print('tempo medio de execucao: ',sum/i)
    i+=1	
if Flag == True:
    while True:
        floki.controle(pso.kp, pso.ki, pso.kd)'''
