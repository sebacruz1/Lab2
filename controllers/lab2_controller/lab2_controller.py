"""lab2_controller controller."""
from controller import Robot

# Parametros
TIME_STEP          = 64      # Paso de simulación [ms]
WHEEL_RADIUS       = 0.0205  # Radio de rueda e-puck [m]
FORWARD_SPEED      = 5.0     # Velocidad de avance [rad/s]
TURN_SPEED         = 3.5     # Rueda rápida al girar [rad/s]
INNER_SPEED        = 0.0     # Rueda interior al girar
THRESHOLD_HIGH     = 95.0    # Umbral para activar giro
TURN_DURATION      = 10      # Duración del giro en pasos
ESCAPE_DURATION    = 8       # Pasos de avance forzado tras el giro
ALPHA              = 0.3     # Coeficiente filtro EMA
Q                  = 5.0     # Varianza ruido de proceso
R                  = 200.0   # Varianza ruido de medición
IR_SCALE           = 500.0   # Factor de conversión avance -> unidades IR

# Inicialización
robot = Robot()

left_motor  = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

ps = []
for i in range(8):
    sensor = robot.getDevice(f'ps{i}')
    sensor.enable(TIME_STEP)
    ps.append(sensor)

left_encoder  = robot.getDevice('left wheel sensor')
right_encoder = robot.getDevice('right wheel sensor')
left_encoder.enable(TIME_STEP)
right_encoder.enable(TIME_STEP)

# Variables de estado
ema_front = 0.0
d_hat     = 0.0
P         = 100.0
prev_left_enc  = None
prev_right_enc = None

state      = 'FORWARD'
turn_steps = 0

# Funciones auxiliares
def get_turn_direction(sensors):
    # Evalúa los sensores laterales y frontales para decidir hacia dónde girar.
    left_side  = sensors[6].getValue() + sensors[5].getValue()
    right_side = sensors[1].getValue() + sensors[2].getValue()
    diff = left_side - right_side

    if abs(diff) >= 50:
        return 'TURN_RIGHT' if diff > 0 else 'TURN_LEFT'

    # Desempate frontal
    if sensors[0].getValue() >= sensors[7].getValue():
        return 'TURN_LEFT'
    else:
        return 'TURN_RIGHT'

# Bucle principal
while robot.step(TIME_STEP) != -1:

    # 1. Lectura de sensores
    front_raw = max(ps[0].getValue(), ps[7].getValue(), ps[1].getValue(), ps[6].getValue())

    # 2. Lectura de encoders y cálculo de odometría
    left_enc  = left_encoder.getValue()
    right_enc = right_encoder.getValue()

    if prev_left_enc is None:
        prev_left_enc  = left_enc
        prev_right_enc = right_enc
        ema_front = front_raw
        d_hat     = front_raw

    delta_left  = left_enc  - prev_left_enc
    delta_right = right_enc - prev_right_enc
    delta_s = WHEEL_RADIUS * (delta_left + delta_right) / 2.0

    prev_left_enc  = left_enc
    prev_right_enc = right_enc

    # 3. Filtros
    ema_front = ALPHA * front_raw + (1.0 - ALPHA) * ema_front

    # Predicción Kalman
    if state == 'FORWARD' or state == 'ESCAPE':
        d_hat_minus = d_hat + IR_SCALE * max(delta_s, 0.0)
    else:
        # Durante el giro, no predecimos acercamiento frontal por odometría
        d_hat_minus = d_hat

    P_minus = P + Q
    K       = P_minus / (P_minus + R)
    d_hat   = d_hat_minus + K * (front_raw - d_hat_minus)
    P       = (1.0 - K) * P_minus

    # 4. Máquina de estados
    if state == 'FORWARD':
        if d_hat >= THRESHOLD_HIGH:
            state = get_turn_direction(ps)
            turn_steps = 0

    elif state in ('TURN_LEFT', 'TURN_RIGHT'):
        turn_steps += 1
        if turn_steps >= TURN_DURATION:
            state = 'ESCAPE'
            turn_steps = 0

    elif state == 'ESCAPE':
        turn_steps += 1
        if d_hat >= THRESHOLD_HIGH:
            state = get_turn_direction(ps)
            turn_steps = 0
        elif turn_steps >= ESCAPE_DURATION:
            state = 'FORWARD'
            turn_steps = 0

    # 5. Actuadores
    if state in ('FORWARD', 'ESCAPE'):
        left_motor.setVelocity(FORWARD_SPEED)
        right_motor.setVelocity(FORWARD_SPEED)
    elif state == 'TURN_RIGHT':
        left_motor.setVelocity(TURN_SPEED)
        right_motor.setVelocity(INNER_SPEED)
    elif state == 'TURN_LEFT':
        left_motor.setVelocity(INNER_SPEED)
        right_motor.setVelocity(TURN_SPEED)
