from machine import Pin, UART
from time import sleep, ticks_ms
import math

# Setup UART1 for communication with Webots
uart = UART(1, 115200, tx=1, rx=3)

# Constants for odometry (MUST MATCH WEBOTS)
R = 0.0205   # wheel radius [m] (20.5mm)
D = 0.052    # wheel base [m] (52mm)

# Initial robot pose (MUST MATCH WEBOTS)
x = 0.0
y = 0.0
phi = 0.0

# Previous encoder readings
prev_left_enc = None
prev_right_enc = None

# State machine variables
current_state = 'forward'
last_state_change = ticks_ms()
state_updated = True

def parse_message(msg):
    try:
        # Expected format: "LLCR|LEFT_ENC,RIGHT_ENC,DT"
        if not msg or '|' not in msg:
            return None
            
        sensors_str, enc_str = msg.strip().split('|')
        if len(sensors_str) < 4 or ',' not in enc_str:
            return None
            
        line_left = (sensors_str[0] == '1')
        line_center = (sensors_str[1] == '1')
        line_right = (sensors_str[2] == '1')
        obstacle = (sensors_str[3] == '1')
        
        enc_parts = enc_str.split(',')
        if len(enc_parts) != 3:
            return None
            
        left_enc = float(enc_parts[0])
        right_enc = float(enc_parts[1])
        dt = float(enc_parts[2])
        
        return line_left, line_center, line_right, obstacle, left_enc, right_enc, dt
    except:
        return None

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle <= -math.pi:
        angle += 2 * math.pi
    return angle

def update_pose(u, w, x, y, phi, dt):
    phi_new = normalize_angle(phi + w * dt)
    avg_phi = (phi + phi_new) / 2  # use average angle for better integration
    delta_x = u * math.cos(avg_phi) * dt
    delta_y = u * math.sin(avg_phi) * dt
    return x + delta_x, y + delta_y, phi_new

while True:
    if uart.any():
        msg_bytes = uart.readline()
        if not msg_bytes:
            continue
            
        try:
            msg_str = msg_bytes.decode('utf-8').strip()
        except:
            continue
            
        parsed = parse_message(msg_str)
        if not parsed:
            continue
            
        line_left, line_center, line_right, obstacle, left_enc, right_enc, dt = parsed
        
        # Initialize previous encoder values on first run
        if prev_left_enc is None:
            prev_left_enc = left_enc
            prev_right_enc = right_enc
            continue
            
        # Calculate wheel angular velocities (rad/s)
        wl = (left_enc - prev_left_enc) / dt
        wr = (right_enc - prev_right_enc) / dt
        
        # Update previous encoder values
        prev_left_enc = left_enc
        prev_right_enc = right_enc
        
        # Compute robot linear (u) and angular (w) speeds
        u = R / 2 * (wr + wl)
        w = R / D * (wr - wl)
        
        # Update robot pose
        x, y, phi = update_pose(u, w, x, y, phi, dt)
        
        # State machine logic
        new_state = current_state
        now = ticks_ms()
        
        if current_state == 'forward':
            if line_right and not line_left:
                new_state = 'turn_right'
            elif line_left and not line_right:
                new_state = 'turn_left'
            elif line_left and line_right and line_center:
                new_state = 'turn_left'
            elif obstacle:
                new_state = 'turn_left'
                
        elif current_state in ['turn_right', 'turn_left']:
            if now - last_state_change > 500:  # 500ms turns
                new_state = 'forward'
                
        # Update state if changed
        if new_state != current_state:
            current_state = new_state
            last_state_change = now
            state_updated = True
            
        # Send new state if updated
        if state_updated:
            uart.write((current_state + '\n').encode('utf-8'))
            state_updated = False
    
    sleep(0.001)  # minimal sleep to prevent CPU overload