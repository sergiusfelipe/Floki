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

        sensor = mpu6050(0x68)
        # K e K1 --> COnstantes para o Filtro Complementar de Shane Colton
        self.K = 0.98
        self.K1 = 1 - K

        self.time_diff = 0.02
        self.ITerm = 0

        # Requisita os dados do MPU6050 
        accel_data = sensor.get_accel_data()
        gyro_data = sensor.get_gyro_data()

        aTempX = accel_data['x']
        aTempY = accel_data['y']
        aTempZ = accel_data['z']

        gTempX = gyro_data['x']
        gTempY = gyro_data['y']
        gTempZ = gyro_data['z']

        # Seta as posicoes iniciais do sensor
        last_x = x_rotation(aTempX, aTempY, aTempZ)
        self.last_y = y_rotation(aTempX, aTempY, aTempZ)

        gyro_offset_x = gTempX
        gyro_offset_y = gTempY

        gyro_total_x = (last_x) - gyro_offset_x
        gyro_total_y = (last_y) - gyro_offset_y

        # Inicializa o controlador
        self.pid = PID.PID(P, I, D)
        last_pidy = 0
        last_deltatime = 0
        t_i = 0
        t_f = 0
        tempo = time.time()

        # Funcao que tem como argumento de entrada o valor obtido do PID, movendo os motores de re
    def backward(sef, velocity):
        PWM1.ChangeDutyCycle(self.velocity)
        GPIO.output(self.int2, GPIO.LOW)
        PWM3.ChangeDutyCycle(self.velocity)
        GPIO.output(self.int4, GPIO.LOW)

    # Assim como na funcao anterior, a entrada se da pelo valor do PID so que movendo os motores no sentido contrario
    def forward(self, velocity):
        GPIO.output(self.int1, GPIO.LOW)
        PWM2.ChangeDutyCycle(self.velocity)
        GPIO.output(self.int3, GPIO.LOW)
        PWM4.ChangeDutyCycle(self.velocity)

    # Para caso o valor do PID for 0, ou seja, o robo esta em equilibrio
    def equilibrium(self):
        GPIO.output(self.int1, False)
        GPIO.output(self.int2, False)
        GPIO.output(self.int3, False)
        GPIO.output(self.int4, False)

    # Funcoes basicas de matematica que serao utilizados 
    def distance(self,a, b):
        return math.sqrt((self.a*self.a) + (self.b*self.b))

    def y_rotation(self, x, y, z):
        radians = math.atan2(self.x, distance(self.y, self.z))
        return -math.degrees(radians)

    def x_rotation(self, x, y, z):
        radians = math.atan2(self.y, distance(self.x, self.z))
        return math.degrees(radians)

# O loop principal do algoritmo onde sera feita todo o o controle de equilibrio do robo
    def controle(self, Kp, Ki, Kd):

        self.pid.setKp(Kp)
        self.pid.setKi(Ki)
        self.pid.setKd(Kd)
        pi = float(tempo)
        accel_data = sensor.get_accel_data()
        gyro_data = sensor.get_gyro_data()

        accelX = accel_data['x']
        accelY = accel_data['y']
        accelZ = accel_data['z']

        gyroX = gyro_data['x']
        gyroY = gyro_data['y']
        gyroZ = gyro_data['z']

        gyroX -= gyro_offset_x
        gyroY -= gyro_offset_y

        gyro_x_delta = (gyroX * time_diff)
        gyro_y_delta = (gyroY * time_diff)

        gyro_total_x += gyro_x_delta
        gyro_total_y += gyro_y_delta

        rotation_x = x_rotation(accelX, accelY, accelZ)
        rotation_y = y_rotation(accelX, accelY, accelZ)
    
    # Filtro Complementar de Shane Colton
        first_y = self.K * (last_y + gyro_y_delta) + (self.K1 * rotation_y)

    # Inicializando alguns parametros do controlador
   
        self.pid.Setpoint(0)
        self.pid.setSampleTime(0.02)
        self.pid.update(first_y)
        self.PIDy = self.pid.output

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

        accel_data = sensor.get_accel_data()
        gyro_data = sensor.get_gyro_data()

        accelX = accel_data['x']
        accelY = accel_data['y']
        accelZ = accel_data['z']

        gyroX = gyro_data['x']
        gyroY = gyro_data['y']
        gyroZ = gyro_data['z']

        gyroX -= gyro_offset_x
        gyroY -= gyro_offset_y

        gyro_x_delta = (gyroX * time_diff)
        gyro_y_delta = (gyroY * time_diff)

        gyro_total_x += gyro_x_delta
        gyro_total_y += gyro_y_delta

        rotation_x = x_rotation(accelX, accelY, accelZ)
        rotation_y = y_rotation(accelX, accelY, accelZ)
    
        # Filtro Complementar de Shane Colton
        last_y = self.K * (last_y + gyro_y_delta) + (self.K1 * rotation_y)
        pf = float(tempo)
        delta_erro =  self.pid.getError() - (self.pid.getSetpoint() - last_y)
        delta_time = pf - pi

        return abs(delta_erro)/delta_time
