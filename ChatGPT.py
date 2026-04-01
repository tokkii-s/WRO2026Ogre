from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait

# ============================================================
# Hub / Devices
# ============================================================

hub = PrimeHub()

# モーターの正方向を物理的な前進に揃える
left_motor = Motor(Port.A)
right_motor = Motor(Port.D, Direction.COUNTERCLOCKWISE)

# 未使用だが指定どおり定義
object_sensor = ColorSensor(Port.E)
kurukuruarm = Motor(Port.F, Direction.COUNTERCLOCKWISE)

# ライントレース用カラーセンサー
line_sensor = ColorSensor(Port.C)

# IMU が安定するまで待つ
while not hub.imu.ready():
    wait(10)

# ============================================================
# Global settings
# ============================================================

# 共通速度
base_speed = 220
turn_speed = 180

# 反射光の最大/最小値
MAX_REFLECTION = 80
MIN_REFLECTION = 10

# 制御周期
CONTROL_INTERVAL = 20  # ms

# ライントレース
LINE_TRACE_KP = 1.10
LINE_TRACE_KD = 0.20
LINE_START_SCALE = 0.30
LINE_STABLE_ERROR_THRESHOLD = 5
LINE_STABLE_CYCLES = 20

# ジャイロ直進
GYRO_STRAIGHT_KP = 2.20
GYRO_STRAIGHT_KD = 0.35
STRAIGHT_TOLERANCE = 2

# ジャイロターン
GYRO_TURN_KP = 2.40
GYRO_TURN_KD = 0.45
GYRO_TURN_KI = 0.08
TURN_I_ENABLE_THRESHOLD = 20
TURN_TOLERANCE = 2
TURN_INTEGRAL_LIMIT = 200.0

# 共通
MAX_MOTOR_SPEED = 1000

# ライントレースの閾値
LINE_THRESHOLD = (MAX_REFLECTION + MIN_REFLECTION) / 2

# 環境差に応じたライントレースのスケーリング
LINE_GAIN_SCALE = (MAX_REFLECTION - MIN_REFLECTION) / 100

# ラインの左右判定
LINE_SIDE_LEFT = -1
LINE_SIDE_RIGHT = 1


# ============================================================
# Helpers
# ============================================================

def clamp(value, low, high):
    return max(low, min(high, value))


def stop_drive():
    left_motor.stop()
    right_motor.stop()


def set_drive(left_speed, right_speed):
    left_motor.run(clamp(left_speed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED))
    right_motor.run(clamp(right_speed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED))


def estimate_line_side(target_heading, current_heading, current_reflection, previous_reflection):
    """
    反射光の変化と進行方向から、ラインの左側/右側を推定する。

    ルール:
    - 左を向いている状態で反射光が増えたら「ラインの左側」
    - 右を向いている状態で反射光が増えたら「ラインの右側」
    - 直前の周期と反射光差がなければ「ラインの左側」
    """
    heading_delta = current_heading - target_heading
    reflection_delta = current_reflection - previous_reflection

    if reflection_delta == 0:
        return LINE_SIDE_LEFT

    if heading_delta < 0:
        return LINE_SIDE_LEFT if reflection_delta > 0 else LINE_SIDE_RIGHT

    if heading_delta > 0:
        return LINE_SIDE_RIGHT if reflection_delta > 0 else LINE_SIDE_LEFT

    return LINE_SIDE_LEFT


# ============================================================
# 1) Line trace
# ============================================================

def line_trace(target_heading):
    """
    1つのカラーセンサーの反射光を使った PD ライントレース。
    開始直後は速度・ゲインとも 0.3 倍、安定後に通常倍率へ切り替える。
    """
    previous_reflection = line_sensor.reflection()
    previous_error = previous_reflection - LINE_THRESHOLD
    stable_count = 0
    mode_scale = LINE_START_SCALE

    while True:
        current_heading = hub.imu.heading()
        reflection = line_sensor.reflection()

        error = reflection - LINE_THRESHOLD
        derivative = error - previous_error

        # ズレが小さい状態が一定回数続いたら通常モードへ
        if abs(error) < LINE_STABLE_ERROR_THRESHOLD:
            stable_count += 1
            if stable_count >= LINE_STABLE_CYCLES:
                mode_scale = 1.0
        else:
            stable_count = 0

        line_side = estimate_line_side(
            target_heading,
            current_heading,
            reflection,
            previous_reflection,
        )

        # ゲインは (最大反射光 - 最小反射光) / 100 を掛けて環境差に追従
        p_term = LINE_TRACE_KP * LINE_GAIN_SCALE * mode_scale * error
        d_term = LINE_TRACE_KD * LINE_GAIN_SCALE * mode_scale * derivative

        steering = line_side * (p_term + d_term)

        left_speed = base_speed * mode_scale + steering
        right_speed = base_speed * mode_scale - steering

        set_drive(left_speed, right_speed)

        previous_error = error
        previous_reflection = reflection
        wait(CONTROL_INTERVAL)


# ============================================================
# 2) Gyro straight
# ============================================================

def gyro_straight(target_heading):
    """
    ジャイロ直進。
    正確さ重視の PD 制御。
    target_heading は進みたい角度。
    """
    previous_error = target_heading - hub.imu.heading()

    while True:
        current_heading = hub.imu.heading()
        error = target_heading - current_heading
        derivative = error - previous_error

        if abs(error) <= STRAIGHT_TOLERANCE and abs(derivative) < 1:
            break

        correction = GYRO_STRAIGHT_KP * error + GYRO_STRAIGHT_KD * derivative

        left_speed = base_speed + correction
        right_speed = base_speed - correction

        set_drive(left_speed, right_speed)

        previous_error = error
        wait(CONTROL_INTERVAL)

    stop_drive()


# ============================================================
# 3) Gyro turn
# ============================================================

def gyro_turn(rotate_angle):
    """
    ジャイロターン。
    ベースは PD 制御、目標角度との差が 20 度未満になったら I 制御を加える。
    rotate_angle は現在角度からの回転量。
    """
    target_heading = hub.imu.heading() + rotate_angle

    previous_error = target_heading - hub.imu.heading()
    integral = 0.0

    while True:
        current_heading = hub.imu.heading()
        error = target_heading - current_heading
        derivative = error - previous_error

        if abs(error) <= TURN_TOLERANCE and abs(derivative) < 1:
            break

        # 目標付近だけ I 制御を使う
        if abs(error) < TURN_I_ENABLE_THRESHOLD:
            integral += error * (CONTROL_INTERVAL / 1000)
            integral = clamp(integral, -TURN_INTEGRAL_LIMIT, TURN_INTEGRAL_LIMIT)
        else:
            integral = 0.0

        correction = (
            GYRO_TURN_KP * error
            + GYRO_TURN_KI * integral
            + GYRO_TURN_KD * derivative
        )

        turn_drive = clamp(turn_speed + correction, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED)

        left_speed = turn_drive
        right_speed = -turn_drive

        set_drive(left_speed, right_speed)

        previous_error = error
        wait(CONTROL_INTERVAL)

    stop_drive()


# ============================================================
# Example usage
# ============================================================
# 必要に応じて、ここを走行順に書き換えてください。
#
# gyro_straight(0)
# line_trace(0)
# gyro_turn(90)
# gyro_straight(90)
#
# 例:
# 1. 0度で直進
# 2. 0度基準のままライントレース
# 3. 90度右回転
# 4. 90度向きで直進