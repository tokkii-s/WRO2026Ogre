from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction, Stop
from pybricks.tools import wait, StopWatch

# --- 1. 定数・変数の設定 ---
# 速度設定
base_speed = 200  # ライントレース・直進用
turn_speed = 150  # 旋回用

# カラーセンサー設定（反射光）
MAX_REF = 80   # 白の部分の反射光値（環境に合わせて調整）
MIN_REF = 10   # 黒の部分の反射光値（環境に合わせて調整）
THRESHOLD = (MAX_REF + MIN_REF) / 2  # しきい値（平均）

# 制御ゲイン（適宜調整してください）
# ライントレース用
LINE_KP = 1.2
LINE_KD = 0.5
# ジャイロ直進用
STRAIGHT_KP = 2.0
STRAIGHT_KD = 1.0
# ジャイロターン用
TURN_KP = 1.5
TURN_KI = 0.1
TURN_KD = 1.0

# 制御周期 (ms)
CONTROL_INTERVAL = 10

# --- 2. 機器の初期化 ---
hub = PrimeHub()
left_motor = Motor(Port.A)
right_motor = Motor(Port.D, Direction.COUNTER_CLOCKWISE) # Dモーターは反時計回り
color_sensor = ColorSensor(Port.C)

# 今回は使用しないが設定
object_sensor = ColorSensor(Port.E)
kurukuruarm = Motor(Port.F, Direction.COUNTER_CLOCKWISE) # 反時計回り

# --- 3. 制御関数の定義 ---

def line_trace_pd(target_rotation):
    """
    反射光PD制御によるライントレース。
    ジャイロを参照してラインの左右どちらにいるかを動的に判断する。
    """
    # 初期状態の記録
    start_angle = left_motor.angle()
    start_gyro = hub.imu.heading()
    
    prev_error = 0
    prev_refl = color_sensor.reflection()
    side_sign = -1  # 初期は左側判定とする
    
    stable_count = 0
    is_soft_start = True
    
    while abs(left_motor.angle() - start_angle) < target_rotation:
        current_refl = color_sensor.reflection()
        current_gyro = hub.imu.heading()
        
        # 左右判定ロジック
        refl_diff = current_refl - prev_refl
        gyro_diff = current_gyro - start_gyro
        
        if refl_diff == 0:
            side_sign = -1 # 変化がない場合は左
        elif (gyro_diff < 0 and refl_diff > 0): # 左を向いて明るくなった
            side_sign = -1 # ラインの左側
        elif (gyro_diff > 0 and refl_diff > 0): # 右を向いて明るくなった
            side_sign = 1  # ラインの右側
            
        # ソフトスタート判定
        error = current_refl - THRESHOLD
        if abs(error) < 5:
            stable_count += 1
        else:
            stable_count = 0
            
        if is_soft_start and stable_count >= 20:
            is_soft_start = False # 安定したら通常モードへ
            
        # ゲインと速度の決定
        current_kp = LINE_KP * ((MAX_REF - MIN_REF) / 100)
        speed = base_speed
        
        if is_soft_start:
            current_kp *= 0.3
            speed *= 0.3
            
        # PD計算
        p_term = error * current_kp
        d_term = (error - prev_error) * LINE_KD
        steering = (p_term + d_term) * side_sign
        
        left_motor.run(speed + steering)
        right_motor.run(speed - steering)
        
        # 次のループへの準備
        prev_error = error
        prev_refl = current_refl
        wait(CONTROL_INTERVAL)
        
    left_motor.stop()
    right_motor.stop()

def gyro_straight_pd(target_rotation):
    """
    ジャイロセンサーを用いたPD制御による直進。
    """
    start_angle = left_motor.angle()
    target_heading = hub.imu.heading() # 開始時の向きを維持
    prev_error = 0
    
    while abs(left_motor.angle() - start_angle) < target_rotation:
        current_heading = hub.imu.heading()
        error = target_heading - current_heading
        
        p_term = error * STRAIGHT_KP
        d_term = (error - prev_error) * STRAIGHT_KD
        steering = p_term + d_term
        
        left_motor.run(base_speed + steering)
        right_motor.run(base_speed - steering)
        
        prev_error = error
        wait(CONTROL_INTERVAL)
        
    left_motor.stop()
    right_motor.stop()

def gyro_turn_pid(target_angle):
    """
    ジャイロPD+I制御による旋回。
    目標誤差20度未満でI制御を有効化し、蓄積誤差は関数ごとにリセット。
    """
    # 目標角度は絶対角として計算（現在の角度 + 指定の回転量）
    goal_heading = hub.imu.heading() + target_angle
    integral = 0
    prev_error = 0
    
    while True:
        current_heading = hub.imu.heading()
        error = goal_heading - current_heading
        
        # 到達判定（誤差が極小になったら終了）
        if abs(error) < 1 and abs(error - prev_error) < 1:
            break
            
        # I制御：20度未満で蓄積開始
        if abs(error) < 20:
            integral += error
        
        p_term = error * TURN_KP
        i_term = integral * TURN_KI
        d_term = (error - prev_error) * TURN_KD
        
        steering = p_term + i_term + d_term
        
        # 旋回スピードに反映（turn_speedをベースに調整）
        left_motor.run(steering)
        right_motor.run(-steering)
        
        prev_error = error
        wait(CONTROL_INTERVAL)
        
    left_motor.stop()
    right_motor.stop()

# --- 4. メイン処理 ---
# 例：90度右旋回して、500度分ライントレースし、1000度分直進する
# gyro_turn_pid(90)
# line_trace_pd(500)
# gyro_straight_pd(1000)