from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.tools import wait, StopWatch, multitask, run_task
from pybricks.robotics import DriveBase
from umath import pi

global hub, motor_A, motor_B, motor_C, motor_D, Drive_base,Chassis_sensor,color_sensor
hub = PrimeHub()
hub.system.set_stop_button(Button.CENTER)
motor_A = Motor(Port.A) 
motor_B = Motor(Port.E,Direction.COUNTERCLOCKWISE) 
motor_C = Motor(Port.F) 
motor_D = Motor(Port.D)
Chassis_sensor = ColorSensor(Port.D) 

Drive_base = DriveBase(motor_B,motor_C,wheel_diameter=62.4,axle_track=165)
Drive_base.use_gyro(True)
Drive_base.settings(turn_rate=60 * 3.6)
timer_move = StopWatch()
Blue = 225
Red  = 350
Green = 150
Yellow= 51
Black = 9  
White = 99
color_counts = {"Blue": 0, "Red": 0, "Green": 0, "Yellow": 0}
recent_colors = []

def Gyro():
    Gyro = hub.imu.heading()
    Gyro = Gyro % 360
    if  Gyro > 180:
        Gyro -= 360
    return Gyro

def pd_move_loop(Power, Kp, Kd, prev_err):
    angle_B = motor_B.angle()
    angle_C = motor_C.angle()
    err = angle_B - angle_C

    P = Kp * err
    D = Kd * (err - prev_err)
    pid_adj = P + D

    motor_B.dc(Power - pid_adj)
    motor_C.dc(Power + pid_adj)

    wait(10)
    return err

def pd_move_enc(Power, Kp, Kd, Deg, Brake=True):
    prev_err = 0
    motor_B.reset_angle(0)
    motor_C.reset_angle(0)

    while (abs(motor_B.angle()) + abs(motor_C.angle())) / 2 <= Deg:
        prev_err = move_sync_pd(Power, Kp, Kd, prev_err)

    if Brake:
        Drive_base.stop()


def pd_move_spin(Power, Kp, Kd, Total_Deg, Brake=True):
    motor_B.reset_angle(0)
    motor_C.reset_angle(0)
    prev_err = 0

    while (abs(motor_B.angle()) + abs(motor_C.angle())) / 2 <= Total_Deg:
        angle_B = motor_B.angle()
        angle_C = motor_C.angle()

        
        err = angle_B + angle_C

        P = Kp * err
        D = Kd * (err - prev_err)
        pid_adj = P + D

        motor_B.dc(Power - pid_adj)   
        motor_C.dc(-Power - pid_adj)  

        prev_err = err
        wait(10)

    if Brake:
        Drive_base.stop()


def move_time(Power_B, Power_C, ms, Brake=True):
    timer_move.reset()
    while timer_move.time() < ms:
        motor_B.dc(Power_B)
        motor_C.dc(Power_C)
    if Brake:
        Drive_base.stop()


def move_deg(Power_B, Power_C, Deg, Brake=True):
    motor_B.reset_angle(0)
    motor_C.reset_angle(0)
    while (abs(motor_B.angle()) + abs(motor_C.angle())) / 2 <= Deg:
        motor_B.dc(Power_B)
        motor_C.dc(Power_C)
    if Brake:
        Drive_base.stop()


def turn_gyro(hdg, Brake=True):
    Drive_base.settings(turn_rate=60 * 3.6)
    Drive_base.turn(hdg)
    if Brake:
        Drive_base.stop()

def turn_gyrospeed(speed,hdg, Brake=True):
    Drive_base.settings(turn_rate= speed * 3.6)
    Drive_base.turn(hdg)
    if Brake:
        Drive_base.stop()

def gyro_track(Kp, Kd, Deg1, Deg2, GyroDeg, Lowpower, Bigpower, Lowpower2, Deg,Brake=True):
    motor_B.reset_angle(0)
    motor_C.reset_angle(0)
    Last_error = 0
    MotorDeg = 0
    while MotorDeg <= Deg:
        MotorDeg = (abs(motor_B.angle()) + abs(motor_C.angle())) / 2
        if MotorDeg > Deg1 and MotorDeg < Deg-Deg2:
            power = Bigpower
        else:
            if MotorDeg < Deg / 2:
                inMin = 0
                inMax = Deg1
                outMin = Lowpower
                outMax = Bigpower
            else:
                inMin = Deg - Deg2
                inMax = Deg
                outMin = Bigpower
                outMax = Lowpower2
            power = map_value(MotorDeg, inMin, inMax, outMin, outMax)
        error = GyroDeg - Gyro()
        derivative = error - Last_error
        correction = (Kp * error) + (Kd * derivative)
        motor_B.dc(power + correction)
        motor_C.dc(power - correction)
        Last_error = error
        wait(10)
    if Brake:
        Drive_base.stop()

def chassis_sensor_track_L(Kp, Kd, Power, side):
    prev_error = 0

    error = (Chassis_sensor.reflection() - (Black+White)/2) * side
    derivative = error - prev_error
    correction = (Kp * error) + (Kd * derivative)

    motor_B.dc(Power - correction)
    motor_C.dc(Power + correction)
    prev_error = error

    wait(10)

def chassis_sensor_acc_L(Kp, Kd, Acc, Dec, LowPower, HighPower, EndPower, Deg, side, Brake=True):
    motor_B.reset_angle(0)
    motor_C.reset_angle(0)
    prev_error = 0

    while (abs(motor_B.angle()) + abs(motor_C.angle())) / 2 <= Deg:
        MotorDeg = (abs(motor_B.angle()) + abs(motor_C.angle())) / 2
       
        if Acc < MotorDeg < Deg - Dec:
            power = HighPower
        else:
            if MotorDeg < Deg / 2:
                inMin, inMax, outMin, outMax = 0, Acc, LowPower, HighPower
            else:
                inMin, inMax, outMin, outMax = Deg - Dec, Deg, HighPower, EndPower
            power = map_value(MotorDeg, inMin, inMax, outMin, outMax)
        
        error = (Chassis_sensor.reflection() - (Black+White)/2) * side
        derivative = error - prev_error
        correction = (Kp * error) + (Kd * derivative)

        motor_B.dc(power - correction)
        motor_C.dc(power + correction)

        prev_error = error
        wait(10)

    if Brake:
        Drive_base.stop()

def a_motor(Power, Deg, is_wait=True):
    motor_A.run_angle(Power * 11, Deg, wait=is_wait)

def d_motor(Power, Deg, is_wait=True):
    motor_D.run_angle(Power * 11, Deg, wait=is_wait)

def bc_stop():
    motor_B.dc(0)
    motor_C.dc(0)

def map_value(value, from_low, from_high, to_low, to_high):
    return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low

def color_Hsv():
    value = color_sensor.hsv()
    h = value.h  

    if 200 <= h <= 240:
        return "Blue"
    elif h >= 340 or h <= 20:
        return "Red"
    elif 120 <= h <= 178:
        return "Green"
    elif 30 <= h <= 80:
        return "Yellow"

def record_color():
    current_color = color_Hsv()
   
    recent_colors.append(current_color)
    
    if len(recent_colors) > 4:
        recent_colors.pop(0)

def find_missing_colors():
    last_three_colors = recent_colors[-3:]  
    all_colors = {"Blue", "Red", "Green", "Yellow"}  
    detected_colors = set(last_three_colors)  
    
    missing_colors = all_colors - detected_colors
   
    return ", ".join(missing_colors)
