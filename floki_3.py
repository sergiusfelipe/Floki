# Algoritmo para equilibrio do floki

# Importando todas as bibliotedas e classes necessarias
from mpu6050 import mpu6050
from time import sleep
import math
import PID_3
import RPi.GPIO as GPIO
import time


GPIO.setmode(GPIO.BCM) # Utilizando a pinagem no modo BCM 
GPIO.setwarnings(False) # Desabilitando avisos

        # Declarando os pino GPIO que serao utilizados
int1 = 21
int2 = 20
int3 = 16
int4 = 12

GPIO.setup(int1, GPIO.OUT)
GPIO.setup(int2, GPIO.OUT)
GPIO.setup(int3, GPIO.OUT)
GPIO.setup(int4, GPIO.OUT)

        # Pulse width modulation: a velocidade muda de acordo com o angulo
PWM1 = GPIO.PWM(21, 100)
PWM2 = GPIO.PWM(20, 100)
PWM3 = GPIO.PWM(16, 100)
PWM4 = GPIO.PWM(12, 100)

        # Da inicio aos PWM
PWM1.start(0)
PWM2.start(0)
PWM3.start(0)
PWM4.start(0)

def backward(velocity):
        PWM1.ChangeDutyCycle(velocity)
        GPIO.output(int2, GPIO.LOW)
        PWM3.ChangeDutyCycle(velocity)
        GPIO.output(int4, GPIO.LOW)

    # Assim como na funcao anterior, a entrada se da pelo valor do PID so que movendo os motores no sentido contrario
def forward(velocity):
        GPIO.output(int1, GPIO.LOW)
        PWM2.ChangeDutyCycle(velocity)
        GPIO.output(int3, GPIO.LOW)
        PWM4.ChangeDutyCycle(velocity)

    # Para caso o valor do PID for 0, ou seja, o robo esta em equilibrio
def equilibrium():
        GPIO.output(int1, False)
        GPIO.output(int2, False)
        GPIO.output(int3, False)
        GPIO.output(int4, False)

class FLOKI:

    def __init__(self, P, I, D, samples):

        self.sensor = mpu6050(0x68)
        # K e K1 --> COnstantes para o Filtro Complementar de Shane Colton
        self.K = 0.98
        self.K1 = 1 - self.K

        self.time_diff = 0.01
        self.ITerm = 0
        self.n_amostra = samples
        # Requisita os dados do MPU6050 
        self.accel_data = self.sensor.get_accel_data()
        self.gyro_data = self.sensor.get_gyro_data()

        aTempX = self.accel_data['x']
        aTempY = self.accel_data['y']
        aTempZ = self.accel_data['z']

        gTempX = self.gyro_data['x']
        gTempY = self.gyro_data['y']
        gTempZ = self.gyro_data['z']

        # Seta as posicoes iniciais do sensor
        self.last_x = self.x_rotation(aTempX, aTempY, aTempZ)
        self.last_y = self.y_rotation(aTempX, aTempY, aTempZ)

        self.gyro_offset_x = gTempX
        self.gyro_offset_y = gTempY

        self.gyro_total_x = (self.last_x) - self.gyro_offset_x
        self.gyro_total_y = (self.last_y) - self.gyro_offset_y

        # Inicializa o controlador
        self.pid = PID_3.PID(P, I, D)
        last_pidy = 0
        last_deltatime = 0
        t_i = 0
        t_f = 0
        self.tempo = time.time()

    # Funcoes basicas de matematica que serao utilizados 
    def distance(self,a, b):
        return math.sqrt((a*a) + (b*b))

    def y_rotation(self, x, y, z):
        radians = math.atan2(x, self.distance(y, z))
        return -math.degrees(radians)

    def x_rotation(self, x, y, z):
        radians = math.atan2(y, self.distance(x, z))
        return math.degrees(radians)

# O loop principal do algoritmo onde sera feita todo o o controle de equilibrio do robo
    def controle(self, Kp, Ki, Kd):
	
        self.pid.setKp(Kp)
        self.pid.setKi(Ki)
        self.pid.setKd(Kd)
        pi = self.pid.getCurrentTime()
        IAE = 0
        
        for i in range(0,self.n_amostra):
	    
            self.accel_data = self.sensor.get_accel_data()
            self.gyro_data = self.sensor.get_gyro_data()

            accelX = self.accel_data['x']
            accelY = self.accel_data['y']
            accelZ = self.accel_data['z']

            gyroX = self.gyro_data['x']
            gyroY = self.gyro_data['y']
            gyroZ = self.gyro_data['z']

            gyroX -= self.gyro_offset_x
            gyroY -= self.gyro_offset_y

            gyro_x_delta = (gyroX * self.time_diff)
            gyro_y_delta = (gyroY * self.time_diff)

            self.gyro_total_x += gyro_x_delta
            self.gyro_total_y += gyro_y_delta

            rotation_x = self.x_rotation(accelX, accelY, accelZ)
            rotation_y = self.y_rotation(accelX, accelY, accelZ)
    
            # Filtro Complementar de Shane Colton
            self.last_y = self.K * (self.last_y + gyro_y_delta) + (self.K1 * rotation_y)

            # Inicializando alguns parametros do controlador
   
            self.pid.Setpoint(0)
            self.pid.setSampleTime(0.01)
            self.pid.update(self.last_y)
            PIDy = self.pid.output

            # Se PIDy < 0 entao o sentido dos motores sera de re
            if PIDy < 0.0:
                print(PIDy)
                if PIDy < -100:
                    PIDy = -100
                backward(-float(PIDy))
                #StepperFor(-PIDy)
            # Se PIDy > entao o sentido dos motores sera frente
            elif PIDy > 0.0:
                print(PIDy)
                if PIDy > 100:
                    PIDy = 100
                forward(float(PIDy))
                #StepperBACK(PIDy)
            # E no caso de PIDy = 0 entao o robo esta em equilibrio 
            else:
                equilibrium()

            print((int(self.last_y), 'PID: ', int(PIDy)))
            last_pidy = PIDy
            deltatime = self.pid.getCurrentTime() - pi
            IAE += abs(self.pid.getError())*deltatime
            pi = self.pid.getCurrentTime()
            sleep(0.01)
            
        

        return IAE
