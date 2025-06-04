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

# Current action received from Webots
current_action = 'stop'

def parse_message(msg):
    """
    Parses messages from Webots. Expected format: "LLCR|LEFT_ENC,RIGHT_ENC,DT|COMMAND|WP1_R,WP1_C;..."
    Returns a tuple: (line_left, line_center, line_right, obstacle, left_enc, right_enc, dt, command, waypoints)
    """
    try:
        if not msg or '|' not in msg:
            return None

        parts = msg.strip().split('|')
        if len(parts) < 3: # Expecting at least sensor data, encoder data, and command
            return None

        sensors_str = parts[0]
        enc_str = parts[1]
        command = parts[2]
        path_str = parts[3] if len(parts) > 3 else "" # Optional path string

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

        waypoints = []
        if path_str:
            for wp_str in path_str.split(';'):
                if wp_str:
                    wp_coords = wp_str.split(',')
                    if len(wp_coords) == 2:
                        waypoints.append((int(wp_coords[0]), int(wp_coords[1])))

        return line_left, line_center, line_right, obstacle, left_enc, right_enc, dt, command, waypoints
    except (ValueError, IndexError):
        # Handle cases where float/int conversion or list indexing fails
        return None

def normalize_angle(angle):
    """Normalizes an angle to be within the range (-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle <= -math.pi:
        angle += 2 * math.pi
    return angle

def update_pose(u, w, x, y, phi, dt):
    """
    Updates the robot's pose (x, y, phi) based on linear (u) and angular (w) velocities.
    Uses an improved integration method with average angle.
    """
    phi_new = normalize_angle(phi + w * dt)
    avg_phi = (phi + phi_new) / 2
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
        except UnicodeError: # Handle decoding errors
            continue

        parsed = parse_message(msg_str)
        if not parsed:
            continue

        line_left, line_center, line_right, obstacle, left_enc, right_enc, dt, received_command, _ = parsed

        # Initialize previous encoder values on first run
        if prev_left_enc is None:
            prev_left_enc = left_enc
            prev_right_enc = right_enc
            # On the very first data packet, we just initialize, don't calculate movement
            continue

        # Calculate wheel angular velocities (rad/s)
        wl = (left_enc - prev_left_enc) / dt
        wr = (right_enc - prev_right_enc) / dt

        # Update previous encoder values for the next iteration
        prev_left_enc = left_enc
        prev_right_enc = right_enc

        # Compute robot linear (u) and angular (w) speeds
        u = R / 2 * (wr + wl)
        w = R / D * (wr - wl)

        # Update robot pose using odometry
        x, y, phi = update_pose(u, w, x, y, phi, dt)

        # Update the current action based on the command received from Webots
        current_action = received_command

        # Send current pose back to Webots for localization and re-planning
        # Format: "ODOM|X,Y,PHI\n"
        odom_msg = f"ODOM|{x:.4f},{y:.4f},{phi:.4f}\n"
        uart.write(odom_msg.encode('utf-8'))

    # Set motor speeds based on the current action received from Webots
    leftSpeed = 0.0
    rightSpeed = 0.0

    # Adjust these speeds for your specific robot and desired movement characteristics
    # These are simplified actions for movement between grid cells
    if current_action == 'forward':
        leftSpeed = 0.5 # Example speed
        rightSpeed = 0.5
    elif current_action == 'turn_left':
        leftSpeed = 0.0
        rightSpeed = 0.3 # Reduced speed for turning
    elif current_action == 'turn_right':
        leftSpeed = 0.3
        rightSpeed = 0.0
    elif current_action == 'stop':
        leftSpeed = 0.0
        rightSpeed = 0.0

    # In a real robot, you'd apply these speeds to your motors (e.g., motor_left.set_speed(leftSpeed))
    # For this MicroPython script, this is just a placeholder.
    # print(f"Action: {current_action}, Left: {leftSpeed:.2f}, Right: {rightSpeed:.2f}")

    sleep(0.001)  # Minimal sleep to prevent CPU overload

