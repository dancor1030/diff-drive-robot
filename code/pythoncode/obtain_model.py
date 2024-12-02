from machine import Pin, PWM
# import utime
# import time
import utime

# sample_time = 0.016145005862970
sample_time = 0.05

class Motor:
    def __init__(self, pin_pwm, pin_dir1, pin_dir2, pin_a, pin_b, cpr=48, threshold=5):
        self.pin_pwm = PWM(Pin(pin_pwm))
        self.pin_pwm.freq(1000)  # Frecuencia de PWM
        self.pin_dir1 = Pin(pin_dir1, Pin.OUT)
        self.pin_dir2 = Pin(pin_dir2, Pin.OUT)
        self.encoder_a = Pin(pin_a, Pin.IN, Pin.PULL_DOWN)
        self.encoder_b = Pin(pin_b, Pin.IN, Pin.PULL_DOWN)
        self.encoder_count = 0
        self.cpr = cpr  # cuentas por revolución (CPR)
        self.threshold = threshold  # Umbral de error
        self.GR = 29.86

        # Posición y control RST
        self.targetpos_deg = 0  # Posición objetivo en grados
        self.encoder_count = 0  # Posición actual en pulsos (a través del encoder)
        self.currpos_deg = 0 # posicion actual en grados
        self.prevpos_deg = 0
        self.prevT = 0
        self.vel = 0

        # Coeficientes Q calculados previamente
        self.q = [1.695790898891604, -1.564209101108395, 0.0] # [q0 q1 q2]

        self.prev_e = [0, 0] # ek-1 ek-2 
        self.prev_u = 0 # uk-1

        self.control_law = 0


        # Interrupciones del encoder
        self.encoder_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.encoder_callbackA)
        self.encoder_b.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.encoder_callbackB)

        self.maxlim = 50
        self.inflim = 21

    # Callback de interrupción del encoder (sensor A)
    def encoder_callbackA(self, pin):
        if self.encoder_a.value() == 1:
            if self.encoder_b.value() == 0:
                self.encoder_count += 1
            else:
                self.encoder_count -= 1
        else:
            if self.encoder_b.value() == 0:
                self.encoder_count -= 1
            else:
                self.encoder_count += 1
        self.update_pos()                

    # Callback de interrupción del encoder (sensor B)
    def encoder_callbackB(self, pin):
        if self.encoder_b.value() == 1:
            if self.encoder_a.value() == 0:
                self.encoder_count -= 1
            else:
                self.encoder_count += 1
        else:
            if self.encoder_a.value() == 0:
                self.encoder_count += 1
            else:
                self.encoder_count -= 1        
        self.update_pos()                

    def update_pos(self):
        # Lectura de la posición actual basada en el encoder y convertir a grados
        self.currpos_deg = (self.encoder_count / self.cpr) * 360 / self.GR

 
    def update_vel(self):
        self.update_pos()
        deltaT = sample_time
        currvel = (self.currpos_deg - self.prevpos_deg)/deltaT #deg/s
        self.vel = (currvel)/6 # RPM        
        self.prevpos_deg = self.currpos_deg #; // current tick 

    # def moving_average_filter(self, x):
    #     pass

    def move(self, control_effort):
        # Aplicar el control a los pines del motor
        if control_effort > 0:
            self.pin_dir1.value(1)
            self.pin_dir2.value(0)
        elif control_effort < 0:
            self.pin_dir1.value(0)
            self.pin_dir2.value(1)
        else:
            # Detener el motor si el control_effort es cero
            self.pin_dir1.value(0)
            self.pin_dir2.value(0)

        # Ajustar la potencia PWM en función del esfuerzo de control
        u = abs(control_effort)/100
        self.pin_pwm.duty_u16(int(u * 65535))

    # def saturate(self, u):
    #     if abs(u) > self.maxlim:
    #             u = self.maxlim if u > 0 else -1*self.maxlim  # upper limit
    #     elif abs(u) < self.inflim and u != 0:
    #         u = self.inflim if u > 0 else -1*self.inflim 
    #     else:
    #         pass  # Keep the speed unchanged if it's within the desired range
    #     return u

    # def control_speed(self, setpoint):
    #     self.update_pos()
    #     self.update_vel()
    #     self.ek = setpoint - self.vel # ERROR IN TIME K

    #     ekm1 = self.prev_e[0]
    #     ekm2 = self.prev_e[1]
    #     ukm1 = self.prev_u

    #     if abs(self.ek) < self.threshold:
    #         uk = 0
    #         return
    #     else:
    #         uk = self.ek*self.q[0] + ekm1*self.q[1] + ukm1
    #         uk = self.saturate(uk)
        
    #     self.move(uk)

    #     self.control_law = uk
    #     self.prev_e[0] = self.ek # ek-1
    #     self.prev_e[1] = ekm1 # ek-2
    #     self.prev_u = uk
            

# Uso de la clase Motor
motor1 = Motor(pin_pwm=28, pin_dir1=26, pin_dir2=27, pin_a=19, pin_b=18, cpr=12, threshold=1)
motor2 = Motor(pin_pwm=20, pin_dir1=22, pin_dir2=21, pin_a=2, pin_b=3, cpr=12, threshold=1)

# Establecer una posición objetivo en grados
# Bucle de control

# txt = open('data.txt', 'w')
u = 50 # deg
i = 0
i_time = 0

while i_time < 3:
    # motor.control_speed(sp)
    motor1.move(u)
    motor2.move(u)
    motor1.update_vel()
    motor2.update_vel()

    print(f'{i_time} {motor1.vel} {u}')
    # txt.write(f'{i_time} {motor1.vel} {u}\n')

    # txt.write(f'{i_time} {motor.vel} {sp} {motor.control_law}\n')
    utime.sleep(sample_time)  # Ts in seconds

    i += 1
    i_time = i*sample_time

# txt.close()
motor1.move(0)
motor2.move(0)
    
