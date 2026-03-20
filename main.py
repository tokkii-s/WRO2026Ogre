from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait

# ==============================================================================
# ハードウェア初期化
# ==============================================================================
hub = PrimeHub()
motor_left = Motor(Port.A)
motor_right = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE)
color_sensor = ColorSensor(Port.C)
object_sensor = ColorSensor(Port.E)
kurukuruarm = Motor(Port.F)
# ==============================================================================
# 定数・変数の定義
# ==============================================================================

# --- ベーススピード ---
base_speed = 40          # ライントレース・ジャイロ直進のベースパワー (duty %)
turn_speed = 30          # ジャイロターンのベースパワー (duty %)

# --- ライントレース PD制御ゲイン（安定性重視: Kp控えめ、Kd大きめ） ---
KP_LINE = 0.4            # 比例ゲイン
KD_LINE = 2.0            # 微分ゲイン
TARGET_REFLECTION = 50   # 白と黒の境界の反射光目標値（要キャリブレーション）

# --- ジャイロターン PD+I制御ゲイン ---
KP_TURN = 1.5            # 比例ゲイン
KD_TURN = 0.5            # 微分ゲイン
KI_TURN = 0.02           # 積分ゲイン（差が20度未満の場合のみ適用）
TURN_TOLERANCE = 1       # ターン完了の許容誤差（度）
TURN_SETTLE_TIME = 200   # ターン完了判定の安定時間（ms）

# --- ジャイロ直進 PD制御ゲイン（正確性重視: Kp大きめ） ---
KP_STRAIGHT = 2.0        # 比例ゲイン
KD_STRAIGHT = 1.0        # 微分ゲイン

# --- 制御周期 ---
CONTROL_INTERVAL = 10    # 制御ループの待機時間 (ms)


# ==============================================================================
# 関数定義
# ==============================================================================

def clamp(value, min_val, max_val):
    """値を指定範囲内に収める"""
    if value > max_val:
        return max_val
    if value < min_val:
        return min_val
    return value


def line_trace(distance_deg):
    """
    ライントレース（PD制御・カラーセンサー1つ）

    反射光の目標値との偏差を用いたPD制御。
    安定性重視のためKdを大きめに設定。

    Args:
        distance_deg: 進む距離（モーター回転角度の平均値、度）
    """
    # モーター角度をリセットして距離計測の起点とする
    motor_left.reset_angle(0)
    motor_right.reset_angle(0)

    prev_error = 0

    while True:
        # 現在の走行距離を計算（左右モーターの平均）
        distance = (motor_left.angle() + motor_right.angle()) / 2
        if distance >= distance_deg:
            break

        # 反射光を読み取り、偏差を計算
        reflection = color_sensor.reflection()
        error = reflection - TARGET_REFLECTION

        # PD制御の操舵量を計算
        derivative = error - prev_error
        steering = KP_LINE * error + KD_LINE * derivative
        prev_error = error

        # 左右モーターへのパワーを計算
        left_power = clamp(base_speed + steering, -100, 100)
        right_power = clamp(base_speed - steering, -100, 100)

        motor_left.dc(left_power)
        motor_right.dc(right_power)

        wait(CONTROL_INTERVAL)

    # 停止
    motor_left.brake()
    motor_right.brake()


def gyro_turn(angle):
    """
    ジャイロターン（PD制御 + 条件付きI制御）

    PD制御をベースとし、目標角度との差が20度未満になったら
    誤差の積分（I制御）を追加してモータースピードに反映する。
    ターンごとにI制御の蓄積誤差はリセットされる。

    ジャイロ角度は都度リセットしない。

    Args:
        angle: 回転する角度（正=時計回り、負=反時計回り）
    """
    # 現在のヘディングを記録し、目標角度を算出
    start_heading = hub.imu.heading()
    target_heading = start_heading + angle

    # I制御の蓄積誤差をリセット
    integral = 0
    prev_error = 0

    # 安定判定用タイマー
    settle_count = 0
    settle_threshold = TURN_SETTLE_TIME // CONTROL_INTERVAL

    while True:
        # 現在の角度と目標角度の偏差
        current_heading = hub.imu.heading()
        error = target_heading - current_heading

        # 収束判定: 誤差が許容範囲内に一定時間留まったら終了
        if abs(error) < TURN_TOLERANCE:
            settle_count += 1
            if settle_count >= settle_threshold:
                break
        else:
            settle_count = 0

        # 微分項
        derivative = error - prev_error

        # I制御: 目標角度との差が20度未満の場合のみ誤差を蓄積
        if abs(error) < 20:
            integral += error
        else:
            # 20度以上の場合は蓄積しない（蓄積値もリセットしない）
            pass

        # PD(+I)制御の操舵量を計算
        power = KP_TURN * error + KD_TURN * derivative + KI_TURN * integral
        power = clamp(power, -turn_speed, turn_speed)

        prev_error = error

        # ターン: 左右モーターを逆方向に駆動
        motor_left.dc(power)
        motor_right.dc(-power)

        wait(CONTROL_INTERVAL)

    # 停止
    motor_left.brake()
    motor_right.brake()


def gyro_straight(distance_deg):
    """
    ジャイロ直進（PD制御）

    ジャイロセンサーのヘディング角度を用いたPD制御で
    直進性を維持する。正確性重視のためKpを大きめに設定。

    ジャイロ角度は都度リセットしない。

    Args:
        distance_deg: 進む距離（モーター回転角度の平均値、度）
    """
    # モーター角度をリセットして距離計測の起点とする
    motor_left.reset_angle(0)
    motor_right.reset_angle(0)

    # 現在のヘディングを目標角度として記録
    target_heading = hub.imu.heading()
    prev_error = 0

    while True:
        # 現在の走行距離を計算（左右モーターの平均）
        distance = (motor_left.angle() + motor_right.angle()) / 2
        if distance >= distance_deg:
            break

        # ジャイロの偏差を計算
        current_heading = hub.imu.heading()
        error = target_heading - current_heading

        # PD制御の操舵量を計算
        derivative = error - prev_error
        steering = KP_STRAIGHT * error + KD_STRAIGHT * derivative
        prev_error = error

        # 左右モーターへのパワーを計算
        left_power = clamp(base_speed - steering, -100, 100)
        right_power = clamp(base_speed + steering, -100, 100)

        motor_left.dc(left_power)
        motor_right.dc(right_power)

        wait(CONTROL_INTERVAL)

    # 停止
    motor_left.brake()
    motor_right.brake()


# ==============================================================================
# メイン実行部（使用例）
# 使いたい関数を組み合わせてここに記述する
# ==============================================================================

# 例: ライントレースで進み、90度右ターン、直進
# line_trace(1000)       # モーター1000度分ライントレース
# gyro_turn(90)          # 右に90度ターン
# gyro_straight(500)     # モーター500度分直進
line_trace(370)
gyro_straight(100)
gyro_turn(-90)
line_trace(950)
hub.imu.reset_heading()  #ジャイロリセット
gyro_straight(-200)
gyro_turn(180)
gyro_straight(500)
motor_right.run_angle(500, 100)
motor_left.run_angle(500, 100)
gyro_straight(-250)
#黄塔アーム下す
gyro_straight(-250)
#黄塔アーム若干上げる
gyro_straight(500)
motor_left.run_angle(500,-100)
motor_right.run_angle(500,-100)
gyro_turn(90)
gyro_straight(490)
gyro_turn(-90)
line_trace(500)    #根本トレース
gyro_straight(1500)    #曲線強行突破
line_trace(500)
gyro_turn(90)
motor_left.run_angle(500,200)
motor_right.run_angle(500,200)
gyro_straight(800)
hub.imu.reset_heading()    #ジャイロリセット
gyro_straight(-100)
gyro_turn(-90)
#オブジェクト取り開始
object_list = []
for i in range(4):
    gyro_straight(-290)
    if object_sensor.color() == Color.YELLOW:
        object_list.append(0)
    elif object_sensor.color() == Color.BLUE:
        object_list.append(1)
    elif object_sensor.color() == Color.BLACK:
        object_list.append(2)
    elif object_sensor.color() == Color.GREEN:
        object_list.append(3)
    elif object_sensor.color() == Color.RED:
        object_list.append(4)
    kurukuruarm.run_angle(75,90)
#オブジェクト取り終了
motor_right.run_angle(500,-300)
motor_left.run_angle(500,-300)
gyro_turn(90)
gyro_straight(2100)
hub.imu.reset_heading()    #ジャイロリセット
gyro_straight(100)
gyro_turn(90)
gyro_straight(-500)
while color_sensor.color() != Color.RED:
    motor_left.dc(-50.0)
    motor_right.dc(-50.0)
#610 290
#オブジェクト置き開始
kurukuru_target = 0
kurukuru_current = 0
gyro_straight(320)
for i in range(5):
    gyro_straight(290)
    if i in object_list:
        kurukuru_target = object_list.index(i)
        rotate = (kurukuru_target - kurukuru_current) % 4 * 90   #回す角度
        kurukuruarm.run_angle(75,rotate)
        kurukuru_current = kurukuru_target
gyro_straight(400)
motor_right.run_angle(500,300)
motor_left.run_angle(500,300)
gyro_turn(90)
gyro_straight(200)
hub.imu.reset_heading()    #ジャイロリセット
gyro_straight(-800)
gyro_turn(90)
line_trace(500)
gyro_straight(1500)
line_trace(400)
gyro_turn(90)
gyro_straight(-390)
#黄塔置く
gyro_straight(110)
gyro_turn(90)
#赤塔置く
