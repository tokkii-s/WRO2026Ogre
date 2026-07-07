"""
WRO SPIKE Prime Control Program
Pybricks v3.x | Line Trace (PD) only








Port assignment:
A : Left  motor (COUNTERCLOCKWISE for forward)
D : Right motor (CLOCKWISE for forward)
C : Color sensor
"""






import sys, math
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Direction, Port, Stop, Button, Side, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, multitask, run_task








# ============================================================
# Hardware initialization
# ============================================================
hub         = PrimeHub()
left_motor  = Motor(Port.B, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.E)
base_arm    = Motor(Port.F)
tip_arm     = Motor(Port.A)
color       = ColorSensor(Port.D)
obj_color   = ColorSensor(Port.C)
WHEEL_RADIUS = 28.0
WHEEL_BASE   = 192.0
robot       = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=192)
timer       = StopWatch()
# 直進速度, 直進加速度, 旋回速度, 旋回加速度 の順で指定
robot.settings(550, 700, 400, 300)
robot.use_gyro(True)








# ============================================================
# Constants
#   Tune WHEEL_DIAMETER and AXLE_TRACK to your robot.
#   Tune BASE_SPEED, gains, and reflection values to your field.
# ============================================================
BASE_SPEED        = 200         # mm/s








REFLECT_MAX       = 100         # white surface (measure on your field)
REFLECT_MIN       = 16          # black line    (measure on your field)
REFLECT_THRESHOLD = 70








LT_KP             = 1
LT_KD             = 0.7








KP         = 0.5   # x座標 → 目標ヨー角 ゲイン
KS         = 2.0   # 角度誤差 → 操舵量 ゲイン
INTERVAL   = 10    # 制御ループ周期 [ms]








x, y   = 0.0, 0.0
prev_L = 0.0
prev_R = 0.0






# ============================================================
# Odometry
# ============================================================
def update_odometry():
    global x, y, prev_L, prev_R


    curr_L = math.radians(left_motor.angle())
    curr_R = math.radians(right_motor.angle())


    dL = WHEEL_RADIUS * (curr_L - prev_L)
    dR = WHEEL_RADIUS * (curr_R - prev_R)
    dd = (dL + dR) / 2.0


    phi = math.radians(hub.imu.heading())
    x += dd * math.sin(phi)
    y += dd * math.cos(phi)


    prev_L, prev_R = curr_L, curr_R


    return x, y, phi




# ============================================================
# Line trace
# ============================================================
def line_trace(side: str, stop_condition) -> None:
  """
  Follow the edge of a line using one color sensor (PD control).








  Args:
      side           : "Left"  -> follow left  edge (sensor rides right of line)
                       "Right" -> follow right edge (sensor rides left  of line)
      stop_condition : callable() -> bool  -- return True to stop
  """
  norm           = (REFLECT_MAX - REFLECT_MIN) / 100.0
  direction_sign = 1 if side == "Left" else -1
  prev_error     = 0








  while not stop_condition():
      reflect    = color.reflection()
      error      = (reflect - REFLECT_THRESHOLD) * direction_sign
      derivative = error - prev_error
      prev_error = error
      steering   = (LT_KP * error + LT_KD * derivative) * norm








      robot.drive(BASE_SPEED, steering)
      wait(10)








  robot.straight(0, then=Stop.HOLD)






# ============================================================
# Odometry-based straight drive
# ============================================================






def drive_straight(stop_condition):
    """
    オドメトリで座標を追跡しながら直進する。


    Parameters
    ----------
    stop_condition : callable
        停止条件を返す引数なしの関数。Trueになった時点で停止。


    Examples
    --------
    # y座標が500mmを超えたら停止
    drive_straight(stop_condition=lambda: y >= 500)


    # 右モーターの角度が590度を超えたら停止
    drive_straight(stop_condition=lambda: right_motor.angle() > 590)


    # 2秒後に停止（事前に deadline を作っておく）
    from pybricks.tools import StopWatch
    sw = StopWatch()
    drive_straight(stop_condition=lambda: sw.time() > 2000)
    """
    global x, y, prev_L, prev_R


    # 状態リセット
    x, y = 0.0, 0.0
    prev_L = math.radians(left_motor.angle())
    prev_R = math.radians(right_motor.angle())
    hub.imu.reset_heading(0)


    while not stop_condition():
        cx, cy, phi = update_odometry()


        target_heading_deg  = cx * KP
        current_heading_deg = math.degrees(phi)
        steering = (target_heading_deg - current_heading_deg) * KS


        left_motor.run(BASE_SPEED + steering)
        right_motor.run(BASE_SPEED - steering)


        wait(INTERVAL)


    left_motor.stop()
    right_motor.stop()






# ============================================================
# Example mission sequence
# ============================================================




def main_first():




   wait(500)
  
   base_arm.run_time(-1500, 1500, then=Stop.HOLD) #アーム初期位置0
   base_arm.reset_angle(0)
   robot.straight(285, then=Stop.HOLD)
   robot.turn(-93)
   line_trace("Left", stop_condition=lambda: color.reflection() < 18)
   robot.straight(-100)
   robot.turn(180)
   robot.straight(0) #まずかったら90に戻す
   right_motor.reset_angle(0)
   line_trace("Right", stop_condition=lambda: right_motor.angle() > 150)
   right_motor.run_angle(300, -130)
   left_motor.run_angle(300, -130)
   robot.straight(-205) #まずかったら95に戻す
   base_arm.run_angle(500, 180, then=Stop.HOLD) #180
   right_motor.reset_angle(0)
   wait(500)
   while right_motor.angle() >= -102: #赤塔保持
       robot.drive(-300, 0)
       wait(5)
   robot.stop()
   wait(1000)
   hub.imu.reset_heading(-90)
   robot.straight(120)
   robot.turn(-90)
   base_arm.run_angle(500, 180, then=Stop.HOLD) #360
   robot.straight(66)
   base_arm.run_angle(100, 100) #460
   tip_arm.run_time(500, 1400) #黃塔1個保持
   base_arm.run_angle(500, -255, then=Stop.HOLD) #205
   print(base_arm.angle())
   robot.straight(-82) #要調整
   robot.turn(93)
   right_motor.reset_angle(0)
   line_trace("Right", stop_condition=lambda: color.reflection() < 18 and right_motor.angle() > 200)
   robot.straight(-27) #要調整
   robot.turn(90)
   base_arm.run_angle(500, 175, then=Stop.HOLD) #380 一旦おろすfor黃塔保持
   tip_arm.run_angle(500, -200)
   base_arm.run_time(-2000, 1200, then=Stop.HOLD) #アーム初期位置0
   right_motor.reset_angle(0)
   wait(500)
   while right_motor.angle() >= -270: #黄塔保持
       robot.drive(-100, hub.imu.heading() * -1)
       base_arm.run(-600)
       wait(5)
   robot.stop()
   base_arm.hold()
   base_arm.run_angle(500, 300, then=Stop.HOLD) #300 アーム閉め
   timer.reset()
   while timer.time() < 400: #黄塔完全保持
       robot.drive(-120, 0)
       wait(10)
   robot.stop()
   wait(500)
   hub.imu.reset_heading(0)
   wait(500)
   base_arm.hold()
   robot.straight(155)
   base_arm.run_angle(500, 120, then=Stop.HOLD) #400
   tip_arm.run_time(500, 1300) #黃塔1個目再保持
   base_arm.run_angle(500, -215, then=Stop.HOLD) #205
   '''timer.reset()
   while timer.time() < 1000: #壁当て・ジャイロリセット
       robot.drive(-350, 0)
       wait(10)
   robot.stop()
   print("角度:", hub.imu.heading(),"角速度:", hub.imu.angular_velocity())'''
   
   right_motor.reset_angle()
   while right_motor.angle() <= 1000: #人取りに行く
       robot.drive(400, hub.imu.heading() * -3)
       wait(5)
   robot.turn(-90)


def main_second():
   
   #robot.settings(300, 100, 400, 300)
   right_motor.reset_angle(0)
   wait(500)
   while right_motor.angle() > -700:
       robot.drive(-200, 0)
       wait(10)
   robot.stop()
   hub.imu.reset_heading(-90)
   right_motor.reset_angle(0)
   wait(500)
   print(right_motor.angle())
   while right_motor.angle() < 350:
       robot.drive(200, 0)
       wait(10)
   robot.stop()
   while color.reflection() > 18:
       robot.drive(200, 0)
       wait(10)
   robot.stop()
   robot.straight(285)
   base_arm.run_angle(500, 175, then=Stop.HOLD) #380 一旦おろす
   tip_arm.run_time(-500, 1300)
   base_arm.run_time(-500, 1500, then=Stop.HOLD) #0
   base_arm.run_angle(500, 205, then=Stop.HOLD) #205
   while color.reflection() < 90:
       robot.drive(-200, 0)
       wait(10)
   robot.stop()
   while color.reflection() > 18:
       robot.drive(-200, 0)
       wait(10)
   robot.stop()
   robot.straight(85)
   robot.turn(90)
   right_motor.reset_angle(0)
   while color.color() != Color.BLUE and right_motor.angle() > 200:
       robot.drive(-200, hub.imu.heading() * -1.2)
       wait(10)
   robot.stop()
   robot.straight(40)
   line_trace("Left", stop_condition=lambda: color.reflection() < 18)
   while color.reflection() > 18:
       robot.drive(200, 0)
       wait(10)
   robot.stop()
   right_motor.run_angle(400, -450)
   left_motor.run_angle(400, -450)
   while color.reflection() < 90:
       robot.drive(200, hub.imu.heading() * -0.6)
       wait(10)
   robot.stop()
   while color.reflection() > 18:
       robot.drive(200, hub.imu.heading() * -0.6)
       wait(10)
   robot.stop()
   right_motor.reset_angle(0)
   while right_motor.angle() < 18:
       robot.drive(200, hub.imu.heading() * -0.6)
       wait(10)
   robot.stop()
   right_motor.reset_angle(0)
   while right_motor.angle() < 100:
       robot.drive(200, hub.imu.heading() * -0.6)
       wait(10)
   robot.stop()
   base_arm.run_angle(200, 300)
   while right_motor.angle() < 110:
       robot.drive(200, hub.imu.heading() * -0.6)
       wait(10)
   robot.stop()
   robot.straight(-100)
   robot.turn(-90)




   


   
   right_motor.run_angle(200)
   left_motor.run_angle(200)
   right_motor.reset_angle(0)
   line_trace("Right", stop_condition=lambda: right_motor.angle() > 590)
   robot.turn(90)
   base_arm.run_time(-500, 3000, then=Stop.HOLD)
   robot.straight(-140)
   base_arm.run_angle(500, 220, then=Stop.HOLD)
   robot.straight(-140)
   robot.straight(100)
   robot.turn(-30)
   robot.straight(300)
   robot.turn(30)
   right_motor.reset_angle(0)
   line_trace("Right", stop_condition=lambda: right_motor.angle() > 350)
   sys.exit()
   right_motor.run_angle(300, -150)
   hub.imu.reset_heading(0)
   while hub.imu.heading() >= -180: #黄塔取りに回る
       speed = max(-300, (hub.imu.heading() + 180) * -2 - 100)
       left_motor.run(speed)
   robot.stop()
   base_arm.run_angle(500, -300, then=Stop.HOLD)
   robot.straight(-200)
   base_arm.run_angle(500, 100, then=Stop.HOLD) #黄塔保持
   robot.straight(-50)
   robot.settings(800, 500, 600, 270)
   robot.straight(480)
   robot.turn(-90)
   robot.straight(200)
   right_motor.reset_angle(0)
   line_trace("Right", stop_condition=lambda: right_motor.angle() > 350)
   robot.straight(1850)
   right_motor.reset_angle(0)
   line_trace("Right", stop_condition=lambda: right_motor.angle() > 150 and color.color() == Color.BLUE)
   robot.straight(-30)
   robot.turn(90)
   robot.straight(230) #黄塔一個目に接近
   base_arm.run_angle(500, 100, then=Stop.HOLD)
   tip_arm.run_angle(500, -170) #黄塔一個目置き
   robot.straight(-230)
   robot.turn(-90)












  




  
def port_view():
   while True:
       print(color.reflection())




def arm():
   base_arm.run_angle(700, 270)
   tip_arm.run_angle(700, 190)
   base_arm.run_angle(700, -250)
   robot.straight(150)
   wait(100)
   base_arm.run_angle(250, 150)
   tip_arm.run_angle(500, -170)
   base_arm.run_angle(250, -150)
   robot.straight(-100)




  
def color_reading():
   robot.straight(-100)
   robot.turn(-90)
   robot.straight()
   while line_sensor.color() != Color.YELLOW:
       drive_base.drive(200, 0)
   drive_base.stop()
  








if __name__ == "__main__":


   
   #right_motor.run_angle(400, -550)
   #left_motor.run_angle(400, -550)
   #print(color.reflection())
   #robot.straight(-10000)
   main_first()
   main_second()
   #base_arm.run_time(-700, 1000)




  #timer = StopWatch()








  # Right edge, 3 seconds
  #timer.reset()
  #line_trace("Right", stop_condition=lambda: timer.time() > 4000)