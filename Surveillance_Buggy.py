import time
import RPi.GPIO as GPIO
import serial

EN_A = 17  
IN1 = 27  
enable
IN2 = 22  
enable
IN3 = 23    
enable
IN4 = 24   
enable
EN_B = 25  

SERVO_PIN = 18 
TRIG = 19  
ECHO = 13 
R_S = 5 
L_S = 6 

BT_BAUDRATE = 9600


mode = 0             # 0: Manual, 1: Object Follower, 2: Obstacle Avoider
Speed = 130
maxSpeed = 255
minSpeed = 0
command_timeout = 0.2  
last_command_time = time.time()


GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


for pin in (IN1, IN2, IN3, IN4, TRIG):
    GPIO.setup(pin, GPIO.OUT)
for pin in (ECHO, R_S, L_S):
    GPIO.setup(pin, GPIO.IN)


pwm_en_a = GPIO.PWM(EN_A, 1000)
pwm_en_b = GPIO.PWM(EN_B, 1000)
pwm_servo = GPIO.PWM(SERVO_PIN, 50)  # 50Hz
pwm_en_a.start(Speed)
pwm_en_b.start(Speed)
pwm_servo.start(0)

bt_serial = serial.Serial(BT_SERIAL_PORT, BT_BAUDRATE, timeout=0.1)

def servo_pulse(angle):
    duty = (angle / 18.0) + 2.5
    pwm_servo.ChangeDutyCycle(duty)
    time.sleep(0.05)

def forward():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def backward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def turn_left():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def turn_right():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

def ultrasonic_read():
    GPIO.output(TRIG, False)
    time.sleep(0.01)
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    start = time.time()
    while GPIO.input(ECHO) == 0:
        start = time.time()
    while GPIO.input(ECHO) == 1:
        stop = time.time()

    duration = stop - start
    dist = (duration * 34300) / 2
    return dist

try:
    while True:
        data = bt_serial.read()
        if data:
            bt_val = data[0]
            if bt_val > 20:
                Speed = min(max(bt_val, minSpeed), maxSpeed)
                pwm_en_a.ChangeDutyCycle(Speed)
                pwm_en_b.ChangeDutyCycle(Speed)

            if bt_val == 8:
                mode = 0
                stop()
            elif bt_val == 9:
                mode = 1
                pwm_en_a.ChangeDutyCycle(130)
                pwm_en_b.ChangeDutyCycle(130)
            elif bt_val == 10:
                mode = 2
                pwm_en_a.ChangeDutyCycle(255)
                pwm_en_b.ChangeDutyCycle(255)

            last_command_time = time.time()

        if time.time() - last_command_time > command_timeout:
            stop()

        if mode == 0:
            # Manual controls
            if data:
                if bt_val == 1:
                    forward()
                elif bt_val == 2:
                    backward()
                elif bt_val == 3:
                    turn_left()
                elif bt_val == 4:
                    turn_right()
                elif bt_val == 5:
                    stop()
                elif bt_val == 6:
                    turn_left()
                    time.sleep(0.4)
                    stop()
                elif bt_val == 7:
                    turn_right()
                    time.sleep(0.4)
                    stop()

        elif mode == 1:
            # Object Follower
            right_ir = GPIO.input(R_S)
            left_ir = GPIO.input(L_S)
            distance = ultrasonic_read()

            # If object centered
            if left_ir == GPIO.LOW and right_ir == GPIO.LOW:
                if 2 < distance < 17:
                    forward()
                else:
                    stop()
            elif left_ir == GPIO.LOW and right_ir == GPIO.HIGH:
                turn_right()
                time.sleep(0.3)
                stop()
            elif left_ir == GPIO.HIGH and right_ir == GPIO.LOW:
                turn_left()
                time.sleep(0.3)
                stop()
            else:
                stop()

        elif mode == 2:
            # Obstacle avoidance
            dist = ultrasonic_read()
            if dist > 20:
                forward()
            else:
                stop()
                for angle in range(70, 141, 5): servo_pulse(angle)
                time.sleep(0.3)
                left_dist = ultrasonic_read()
                for angle in range(140, -1, -5): servo_pulse(angle)
                time.sleep(0.5)
                right_dist = ultrasonic_read()
                for angle in range(0, 71, 5): servo_pulse(angle)
                time.sleep(0.3)

                if left_dist > right_dist:
                    turn_left(); time.sleep(0.35)
                elif right_dist > left_dist:
                    turn_right(); time.sleep(0.35)
                else:
                    backward(); time.sleep(0.3)
                    turn_right(); time.sleep(0.6)

        time.sleep(0.01)

except KeyboardInterrupt:
    pass

finally:
    pwm_en_a.stop()
    pwm_en_b.stop()
    pwm_servo.stop()
    GPIO.cleanup()
