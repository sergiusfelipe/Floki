# Algoritmo para equilibrio do floki

# Importando todas as bibliotedas e classes necessarias
from mpu6050 import mpu6050
from time import sleep
import math
import PID_3
import RPi.GPIO as GPIO
import time



class FLOKI:

    def __init__(self, P, I, D):

        GPIO.setmode(GPIO.BCM) # Utilizando a pinagem no modo BCM 
        GPIO.setwarnings(False) # Desabilitando avisos

        # Declarando os pino GPIO que serao utilizados
        self.int1 = 21
        self.int2 = 20
        self.int3 = 16
        self.int4 = 12

        GPIO.setup(self.int1, GPIO.OUT)
        GPIO.setup(self.int2, GPIO.OUT)
        GPIO.setup(self.int3, GPIO.OUT)
        GPIO.setup(self.int4, GPIO.OUT)

        # Pulse width modulation: a velocidade muda de acordo com o angulo
        global PWM1 = GPIO.PWM(21, 100)
        global PWM2 = GPIO.PWM(20, 100)
        global PWM3 = GPIO.PWM(16, 100)
        global PWM4 = GPIO.PWM(12, 100)

        # Da inicio aos PWM
        PWM1.start(0)
        PWM2.start(0)
        PWM3.start(0)
        PWM4.start(0)

        self.sensor = mpu6050(0x68)
        # K e K1 --> COnstantes para o Filtro Complementar de Shane Colton
        self.K = 0.98
        self.K1 = 1 - self.K

        self.time_diff = 0.02
        self.ITerm = 0

        # Requisita os dados do MPU6050 
        accel_data = self.sensor.get_accel_data()
        gyro_data = self.sensor.get_gyro_data()

        aTempX = accel_data['x']
        aTempY = accel_data['y']
        aTempZ = accel_data['z']

        gTempX = gyro_data['x']
        gTempY = gyro_data['y']
        gTempZ = gyro_data['z']

        # Seta as posicoes iniciais do sensor
        last_x = self.x_rotation(aTempX, aTempY, aTempZ)
        self.last_y = self.y_rotation(aTempX, aTempY, aTempZ)

        self.gyro_offset_x = gTempX
        self.gyro_offset_y = gTempY

        self.gyro_total_x = (last_x) - self.gyro_offset_x
        self.gyro_total_y = (self.last_y) - self.gyro_offset_y

        # Inicializa o controlador
        self.pid = PID_3.PID(P, I, D)
        last_pidy = 0
        last_deltatime = 0
        t_i = 0
        t_f = 0
        self.tempo = time.time()

        # Funcao que tem como argumento de entrada o valor obtido do PID, movendo os motores de re
    def backward(sef, velocity):
        PWM1.ChangeDutyCycle(velocity)
        GPIO.output(self.int2, GPIO.LOW)
        PWM3.ChangeDutyCycle(velocity)
        GPIO.output(self.int4, GPIO.LOW)

    # Assim como na funcao anterior, a entrada se da pelo valor do PID so que movendo os motores no sentido contrario
    def forward(self, velocity):
        GPIO.output(self.int1, GPIO.LOW)
        PWM2.ChangeDutyCycle(velocity)
        GPIO.output(self.int3, GPIO.LOW)
        PWM4.ChangeDutyCycle(velocity)

    # Para caso o valor do PID for 0, ou seja, o robo esta em equilibrio
    def equilibrium(self):
        GPIO.output(self.int1, False)
        GPIO.output(self.int2, False)
        GPIO.output(self.int3, False)
        GPIO.output(self.int4, False)

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
        accel_data = self.sensor.get_accel_data()
        gyro_data = self.sensor.get_gyro_data()

        accelX = accel_data['x']
        accelY = accel_data['y']
        accelZ = accel_data['z']

        gyroX = gyro_data['x']
        gyroY = gyro_data['y']
        gyroZ = gyro_data['z']

        gyroX -= self.gyro_offset_x
        gyroY -= self.gyro_offset_y

        gyro_x_delta = (gyroX * self.time_diff)
        gyro_y_delta = (gyroY * self.time_diff)

        self.gyro_total_x += gyro_x_delta
        self.gyro_total_y += gyro_y_delta

        rotation_x = self.x_rotation(accelX, accelY, accelZ)
        rotation_y = self.y_rotation(accelX, accelY, accelZ)
    
    # Filtro Complementar de Shane Colton
        first_y = self.K * (self.last_y + gyro_y_delta) + (self.K1 * rotation_y)

    # Inicializando alguns parametros do controlador
   
        self.pid.Setpoint(0)
        self.pid.setSampleTime(0.02)
        self.pid.update(first_y)
        PIDy = self.pid.output

    # Se PIDy < 0 entao o sentido dos motores sera de re
        if PIDy < 0.0:
            print(PIDy)
            if PIDy < -100:
                PIDy = -100
            self.backward(-float(PIDy))
        #StepperFor(-PIDy)
    # Se PIDy > entao o sentido dos motores sera frente
        elif PIDy > 0.0:
            print(PIDy)
            if PIDy > 100:
                PIDy = 100
            self.forward(float(PIDy))
        #StepperBACK(PIDy)
    # E no caso de PIDy = 0 entao o robo esta em equilibrio 
        else:
            self.equilibrium()

        '''if last_pidy == 100 and PIDy < last_pidy:
            t_i = float(pid.getCurrentTime())

        elif last_pidy < PIDy and PIDy == 100 and t_i != 0:
            t_f = pid.getCurrentTime()
            p_cr = t_f - t_i
            print(p_cr)
            t_i = 0 
            t_f = 0  '''

        print((int(first_y), 'PID: ', int(PIDy)))
        last_pidy = PIDy
        sleep(0.02)

        accel_data = self.sensor.get_accel_data()
        gyro_data = self.sensor.get_gyro_data()

        accelX = accel_data['x']
        accelY = accel_data['y']
        accelZ = accel_data['z']

        gyroX = gyro_data['x']
        gyroY = gyro_data['y']
        gyroZ = gyro_data['z']

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
        pf = self.pid.getCurrentTime()
        delta_erro =  self.pid.getError() - (self.pid.getSetpoint() - self.last_y)
        delta_time = pf - pi

        return abs(delta_erro)/delta_time
