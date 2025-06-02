from controller import Robot
import serial
import math

try:
    ser = serial.Serial(port='COM4', baudrate=115200, timeout=5)
except:
    print("Communication failed. Check the cable connections and serial settings.")
    raise

MAX_SPEED = 6.28
speed = 0.4 * MAX_SPEED

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Devices
ps = [robot.getDevice(name) for name in ['ps0','ps1','ps2','ps3','ps4','ps5','ps6','ps7']]
for sensor in ps:
    sensor.enable(timestep)

gs = [robot.getDevice(name) for name in ['gs0','gs1','gs2']]
for sensor in gs:
    sensor.enable(timestep)

encoders = [robot.getDevice(name) for name in ['left wheel sensor', 'right wheel sensor']]
for encoder in encoders:
    encoder.enable(timestep)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Odometry parameters (must match ESP32)
R = 0.0205  # wheel radius [m] (20.5mm)
D = 0.052   # wheel base [m] (52mm)

# Pose initialization (must match ESP32)
x = 0.0
y = 0.0
phi = 0.0

prev_left_enc = None
prev_right_enc = None
current_state = 'forward'

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle <= -math.pi:
        angle += 2 * math.pi
    return angle

while robot.step(timestep) != -1:
    # Read sensors
    gsValues = [gs[i].getValue() for i in range(3)]
    ps0_value = ps[0].getValue()
    left_enc = encoders[0].getValue()
    right_enc = encoders[1].getValue()
    
    dt = timestep / 1000.0  # convert ms to seconds
    
    # Initialize previous encoder values at first step
    if prev_left_enc is None:
        prev_left_enc = left_enc
        prev_right_enc = right_enc
        continue  # skip first iteration

    # Calculate delta encoders
    delta_left = left_enc - prev_left_enc
    delta_right = right_enc - prev_right_enc

    # Update previous encoder values
    prev_left_enc = left_enc
    prev_right_enc = right_enc

    # Calculate wheel angular velocities (rad/s)
    wl = delta_left / dt
    wr = delta_right / dt

    # Compute linear and angular velocity of robot
    u = R / 2.0 * (wr + wl)   # linear velocity (m/s)
    w = R / D * (wr - wl)     # angular velocity (rad/s)

    # Update pose using improved integration
    phi_new = normalize_angle(phi + w * dt)
    avg_phi = (phi + phi_new) / 2  # use average angle for better position integration
    delta_x = u * math.cos(avg_phi) * dt
    delta_y = u * math.sin(avg_phi) * dt
    
    x += delta_x
    y += delta_y
    phi = phi_new

    # Compose sensor flags (0=line detected, 1=no line)
    line_right = gsValues[0] > 600
    line_center = gsValues[1] > 600
    line_left = gsValues[2] > 600
    obstacle = ps0_value > 80

    # Create message with format: "LLCR|LEFT_ENC,RIGHT_ENC,DT"
    message = ''
    message += '1' if line_left else '0'
    message += '1' if line_center else '0'
    message += '1' if line_right else '0'
    message += '1' if obstacle else '0'
    message += f'|{left_enc:.4f},{right_enc:.4f},{dt:.4f}\n'
    
    # Send to ESP32
    ser.write(message.encode('utf-8'))

    # Read commands from ESP32
    if ser.in_waiting:
        value = ser.readline().decode('utf-8').strip()
        if value in ['forward', 'turn_right', 'turn_left', 'stop']:
            current_state = value

    # Set motor speeds according to state
    if current_state == 'forward':
        leftSpeed = speed
        rightSpeed = speed
    elif current_state == 'turn_right':
        leftSpeed = 0.5 * speed
        rightSpeed = 0
    elif current_state == 'turn_left':
        leftSpeed = 0
        rightSpeed = 0.5 * speed
    elif current_state == 'stop':
        leftSpeed = 0
        rightSpeed = 0

    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

    # Print only the estimated pose (no ground truth needed in HiL)
    print(f'Odometry: x={x:.4f}, y={y:.4f}, phi={phi:.4f}')

ser.close()