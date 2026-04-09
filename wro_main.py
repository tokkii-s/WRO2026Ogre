"""
WRO SPIKE Prime Control Program
Pybricks v3.x | Line Trace (PD) only

Port assignment:
  A : Left  motor (COUNTERCLOCKWISE for forward)
  D : Right motor (CLOCKWISE for forward)
  C : Color sensor
"""

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Direction, Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from pybricks.tools import StopWatch

# ============================================================
# Hardware initialization
# ============================================================
hub         = PrimeHub()
left_motor  = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.D)
color       = ColorSensor(Port.C)
robot       = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=192)

# ============================================================
# Constants
#   Tune WHEEL_DIAMETER and AXLE_TRACK to your robot.
#   Tune BASE_SPEED, gains, and reflection values to your field.
# ============================================================
BASE_SPEED        = 200         # mm/s

REFLECT_MAX       = 100         # white surface (measure on your field)
REFLECT_MIN       = 16          # black line    (measure on your field)
REFLECT_THRESHOLD = (REFLECT_MAX + REFLECT_MIN) // 2

LT_KP             = 1.2
LT_KD             = 0.8

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

    robot.stop()


# ============================================================
# Example mission sequence
# ============================================================

if __name__ == "__main__":

    timer = StopWatch()

    # Right edge, 3 seconds
    timer.reset()
    line_trace("Right", stop_condition=lambda: timer.time() > 3000)
