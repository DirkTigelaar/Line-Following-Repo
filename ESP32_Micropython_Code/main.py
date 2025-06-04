# main.py - MicroPython code for ESP32
# This code handles the main robot logic (pathfinding, state machine, PID, odometry)
# and communicates with Webots via serial.

from machine import Pin, UART
from time import sleep
import heapq

# --- Configuration ---
# Set serial to UART1 using the same pins as UART0 to communicate via USB
# IMPORTANT: Adjust tx and rx pins based on your ESP32 board and wiring.
# For many ESP32 dev kits, UART0 (pins 1 and 3) are used for USB serial.
# If you are using USB-serial for HIL, use UART0.
# If you are using physical UART pins for HIL, use UART1 or UART2 and adjust pins.
uart = UART(1, 115200, tx=1, rx=3)

# Debug LEDs (optional, adjust pins if needed)
led_board = Pin(2, Pin.OUT)     # Onboard LED
led_yellow = Pin(4, Pin.OUT)
led_blue = Pin(23, Pin.OUT)
led_green = Pin(22, Pin.OUT)
led_red = Pin(21, Pin.OUT)

# --- Robot Parameters ---
MAX_SPEED = 6.28 # Max motor speed in rad/s
R = 0.0205  # Wheel radius [m]
D = 0.057  # Distance between wheels [m]
delta_t = 0.064  # Basic timestep in Webots (64ms = 0.064s)

# PID parameters for smooth line following
kp = 2.0  # Proportional gain
ki = 0.2  # Integral gain
kd = 0.4  # Derivative gain

# Error tracking for PID
previous_error = 0.0
integral_error = 0.0
error_history = []
max_history_length = 5

# Line following state tracking
line_threshold = 600.0
consecutive_no_line = 0
max_no_line_steps = 10
last_known_direction = 'center'
base_speed = MAX_SPEED * 0.5
max_turn_speed_diff = MAX_SPEED * 0.4

# Intersection detection variables
cross_counter = 0
CROSS_COUNTER_MIN = 8  # Reduced for faster intersection detection
INTERSECTION_DRIVE_THROUGH_DURATION = 35  # Steps to drive through intersection
intersection_drive_through_counter = 0

# Enhanced turning variables
turn_action = 'forward'
expected_heading_change = 0.0

stop_counter = 0
turn_counter = 0
post_turn_counter = 0
line_search_counter = 0

STOP_DURATION = 15  # Steps to stop before turning
POST_TURN_PAUSE = 10  # Steps to pause after turn
LINE_SEARCH_DURATION = 30  # Steps to search for line after turn

# Turn completion tracking (IMPROVED with closed-loop)
turn_start_heading = 0.0
heading_tolerance = 0.05  # Radians (approx 3 degrees)
turn_kp = 2.0  # Proportional gain for turning

# Position tracking variables (Odometry)
phi = 0.0 # Will be initialized based on path
oldEncoderValues = [0, 0]
encoderValues = [0, 0]
first_data_received = False # Nieuwe vlag voor het bijhouden van de eerste data-ontvangst

# --- Grid Map Definition and Navigation (copy from Obstakeldetectie_try.py) ---
corrected_weighted_grid = {
    # Parking spots (P nodes)
    "P1": {"S": ("A1", 2.0)},
    "P2": {"S": ("A2", 2.0)},
    "P3": {"S": ("A3", 2.0)},
    "P4": {"S": ("A4", 2.0)},
    "P5": {"N": ("E3", 2.0)},
    "P6": {"N": ("E4", 2.0)},
    "P7": {"N": ("E5", 2.0)},
    "P8": {"N": ("E6", 2.0)},

    # A row
    "A1": {"N": ("P1", 2.0), "E": ("A2", 1.0), "S": ("C1", 2.5)},
    "A2": {"N": ("P2", 2.0), "E": ("A3", 1.0), "W": ("A1", 1.0)},
    "A3": {"N": ("P3", 2.0), "E": ("A4", 1.0), "W": ("A2", 1.0)},
    "A4": {"N": ("P4", 2.0), "E": ("A5", 1.0), "W": ("A3", 1.0)},
    "A5": {"N": None, "E": ("A6", 4.0), "S": ("B1", 1.0), "W": ("A4", 1.0)},
    "A6": {"N": None, "E": None, "S": ("B2", 1.5), "W": ("A5", 4.0)},

    # B row
    "B1": {"N": ("A5", 1.5), "E": ("B2", 4), "S": ("C2", 1.0)},
    "B2": {"N": ("A6", 1.5), "E": None, "S": ("C3", 1.0), "W": ("B1", 4.0)},

    # C row
    "C1": {"N": ("A1", 2.5), "E": ("C2", 4.0), "S": ("D1", 1.0)},
    "C2": {"N": ("B1", 1.0), "E": ("C3", 4.0), "S": ("D2", 1.0), "W": ("C1", 4.0)},
    "C3": {"N": ("B2", 1.0), "E": None, "S": ("E6", 2.5), "W": ("C2", 4.0)},

    # D row
    "D1": {"N": ("C1", 1.0), "E": ("D2", 4.0), "S": ("E1", 1.5)},
    "D2": {"N": ("C2", 1.0), "E": None, "S": ("E2", 1.5), "W": ("D1", 4.0)},

    # E row
    "E1": {"N": ("D1", 1.5), "E": ("E2", 4.0), "S": None, "W": None},
    "E2": {"N": ("D2", 1.5), "E": ("E3", 1.0), "S": None, "W": ("E1", 4.0)},
    "E3": {"S": ("P5", 2.0), "E": ("E4", 1.0), "N": None, "W": ("E2", 1.0)},
    "E4": {"S": ("P6", 2.0), "E": ("E5", 1.0), "N": None, "W": ("E3", 1.0)},
    "E5": {"S": ("P7", 2.0), "E": ("E6", 1.0), "N": None, "W": ("E4", 1.0)},
    "E6": {"N": ("C3", 2.5), "E": None, "S": ("P8", 2.0), "W": ("E5", 1.0)},
}

# --- Dijkstra's Algorithm (from Obstakeldetectie_try.py) ---

def find_path_dijkstra(start_node, goal_node, grid_map_weighted, blocked_nodes=None):
    """
    Find shortest path between nodes using Dijkstra's algorithm with weighted edges.
    """
    if blocked_nodes is None:
        blocked_nodes = set()

    if start_node not in grid_map_weighted or goal_node not in grid_map_weighted:
        print("Error: Invalid nodes - Start: {}, Goal: {}".format(start_node, goal_node))
        return []

    distances = {node: float('infinity') for node in grid_map_weighted}
    distances[start_node] = 0

    priority_queue = [(0, start_node, [start_node])]

    while priority_queue:
        current_distance, current_node_pq, path_pq = heapq.heappop(priority_queue)

        if current_distance > distances[current_node_pq]:
            continue

        if current_node_pq == goal_node:
            return path_pq

        for direction, neighbor_info in grid_map_weighted[current_node_pq].items():
            if neighbor_info:
                neighbor, weight = neighbor_info
                if neighbor not in blocked_nodes:
                    distance = current_distance + weight
                    if distance < distances[neighbor]:
                        distances[neighbor] = distance
                        new_path = path_pq + [neighbor]
                        heapq.heappush(priority_queue, (distance, neighbor, new_path))

    print("No path found from {} to {}".format(start_node, goal_node))
    return []

def get_direction_to_next_node(current_node, next_node, grid_map_weighted):
    """
    Get the direction (N, E, S, W) to move from current_node to next_node
    """
    if current_node not in grid_map_weighted:
        print("ERROR: get_direction_to_next_node: current_node '{}' not in grid_map_weighted.".format(current_node))
        return None

    for direction, neighbor_info in grid_map_weighted[current_node].items():
        if neighbor_info and neighbor_info[0] == next_node:
            return direction

    print("ERROR: get_direction_to_next_node: No direct connection found from '{}' to '{}'.".format(current_node, next_node))
    return None

def normalize_angle(angle):
    """Normalize angle to [-π, π] (using float values for pi)"""
    pi_val = 3.14159265359
    while angle > pi_val:
        angle -= 2 * pi_val
    while angle < -pi_val:
        angle += 2 * pi_val
    return angle

def direction_to_robot_action(direction, current_heading):
    """
    Convert grid direction to robot action based on current heading
    Returns: (action_string, angle_difference_needed)
    """
    direction_angles = {
        'N': 1.57079632679, # np.pi / 2
        'E': 0.0,
        'S': -1.57079632679, # -np.pi / 2
        'W': 3.14159265359 # np.pi
    }

    if direction not in direction_angles:
        print("WARNING: Unknown direction '{}'. Defaulting to 'forward'.".format(direction))
        return 'forward', 0.0

    target_angle = direction_angles[direction]

    current_heading_normalized = normalize_angle(current_heading)
    target_angle_normalized = normalize_angle(target_angle)

    # Calculate the shortest angle difference
    angle_diff = target_angle_normalized - current_heading_normalized
    angle_diff = normalize_angle(angle_diff)

    # --- DEBUGGING PRINTS FOR ANGLE CALCULATION ---
    print("DEBUG_DIR_ACTION: Current Heading (norm): {:.3f}".format(current_heading_normalized))
    print("DEBUG_DIR_ACTION: Target Angle (norm): {:.3f}".format(target_angle_normalized))
    print("DEBUG_DIR_ACTION: Calculated Angle Diff: {:.3f}".format(angle_diff))
    # --- END DEBUGGING PRINTS ---

    # Determine required action based on angle difference
    # Priority: Forward -> Turn Around -> Left/Right
    if abs(angle_diff) < 0.1: # Within approx 5.7 degrees - go forward
        return 'forward', angle_diff
    # Check for approximate 180 degree turns (turn around)
    # This condition is evaluated *after* the forward condition, ensuring forward is always prioritized for small diffs.
    elif abs(angle_diff - 3.14159265359) < 0.2 or abs(angle_diff + 3.14159265359) < 0.2: # Close to 180 degrees (pi or -pi)
        return 'turn_around', angle_diff
    elif angle_diff > 0: # Need to turn left (positive angle diff)
        return 'turn_left', angle_diff
    else: # Need to turn right (negative angle diff)
        return 'turn_right', angle_diff

# --- Helper Functions (adapted for MicroPython and serial communication) ---

def get_wheels_speed(encoderValues, oldEncoderValues, delta_t):
    """Calculates wheel angular speeds from encoder readings."""
    wl = (encoderValues[0] - oldEncoderValues[0]) / delta_t
    wr = (encoderValues[1] - oldEncoderValues[1]) / delta_t
    return wl, wr

def get_robot_speeds(wl, wr, R, D):
    """Calculates robot linear and angular speeds."""
    u = R / 2.0 * (wr + wl)
    w = R / D * (wr - wl)
    return u, w

def update_heading(w, phi, delta_t):
    """Updates robot heading based on angular velocity."""
    phi += w * delta_t
    return normalize_angle(phi)

def detect_intersection(gsValues):
    """
    Enhanced intersection detection with better filtering
    """
    line_right = gsValues[0] < line_threshold
    line_center = gsValues[1] < line_threshold
    line_left = gsValues[2] < line_threshold

    strong_intersection = line_right and line_center and line_left
    adjacent_detection = (line_left and line_center) or (line_center and line_right)

    return strong_intersection or adjacent_detection

def calculate_line_error(gsValues):
    """
    Calculate line following error using weighted sensor positions
    Returns error value: negative = line to left, positive = line to right, 0 = centered
    """
    # Normalize sensor values (invert so line gives high values)
    right_val = max(0, line_threshold - gsValues[0]) / line_threshold
    center_val = max(0, line_threshold - gsValues[1]) / line_threshold
    left_val = max(0, line_threshold - gsValues[2]) / line_threshold

    total_activation = right_val + center_val + left_val

    if total_activation < 0.1:  # No line detected
        return None

    error = (right_val * 1.0 + center_val * 0.0 + left_val * (-1.0)) / total_activation

    return error

def advanced_line_following_control(gsValues):
    """
    Advanced PID-based line following with smooth control
    """
    global previous_error, integral_error, error_history
    global consecutive_no_line, last_known_direction

    current_error = calculate_line_error(gsValues)

    if current_error is None:
        consecutive_no_line += 1
        if consecutive_no_line < max_no_line_steps:
            if last_known_direction == 'left':
                current_error = -0.6
            elif last_known_direction == 'right':
                current_error = 0.6
            else:
                current_error = 0.0
        else:
            return base_speed * 0.3, base_speed * 0.3
    else:
        consecutive_no_line = 0
        if current_error < -0.2:
            last_known_direction = 'left'
        elif current_error > 0.2:
            last_known_direction = 'right'
        else:
            last_known_direction = 'center'

    error_history.append(current_error)
    if len(error_history) > max_history_length:
        error_history.pop(0)

    proportional = current_error

    if consecutive_no_line == 0:
        integral_error += current_error
    else:
        integral_error *= 0.9
    integral_error = max(-2.0, min(2.0, integral_error))

    derivative = 0.0
    if len(error_history) >= 2:
        derivative = error_history[-1] - error_history[-2]

    pid_output = kp * proportional + ki * integral_error + kd * derivative
    pid_output = max(-1.0, min(1.0, pid_output))

    turn_adjustment = pid_output * max_turn_speed_diff

    leftSpeed = base_speed - turn_adjustment
    rightSpeed = base_speed + turn_adjustment

    leftSpeed = max(base_speed * 0.2, min(MAX_SPEED, leftSpeed))
    rightSpeed = max(base_speed * 0.2, min(MAX_SPEED, rightSpeed))

    previous_error = current_error

    return leftSpeed, rightSpeed

def execute_improved_turn(action):
    """
    IMPROVED: Execute a more reliable turn with better completion detection
    Uses closed-loop control to achieve target heading based on the pre-calculated expected_heading_change.
    Returns (leftSpeed, rightSpeed, phase_completed, turn_fully_completed)
    """
    global stop_counter, turn_counter, post_turn_counter
    global turn_start_heading, expected_heading_change, phi
    global heading_tolerance, turn_kp

    turn_speed_base = MAX_SPEED * 0.5

    # Phase 1: Stop completely
    if stop_counter < STOP_DURATION:
        if stop_counter == 0:
            turn_start_heading = phi
            print("TURNING_STATE: Starting turn sequence: {}".format(action))
            print("TURNING_STATE: Initial heading for turn: {:.3f} rad".format(phi)) # Added for clarity
            print("TURNING_STATE: Expected change: {:.3f} rad".format(expected_heading_change))

        stop_counter += 1
        return 0.0, 0.0, False, False

    # Phase 2: Execute turn with closed-loop heading control
    target_heading = normalize_angle(turn_start_heading + expected_heading_change)
    current_heading_normalized = normalize_angle(phi)

    # Calculate heading error using basic subtraction and normalization
    heading_error = target_heading - current_heading_normalized
    heading_error = normalize_angle(heading_error)

    print("TURNING_STATE: Current Phi: {:.3f}, Target Heading: {:.3f}, Heading Error: {:.3f}".format(current_heading_normalized, target_heading, heading_error))

    if abs(heading_error) < heading_tolerance:
        print("TURNING_STATE: Turn completed based on precise heading!")
        if post_turn_counter < POST_TURN_PAUSE:
            post_turn_counter += 1
            return 0.0, 0.0, False, False
        else:
            print("TURNING_STATE: Turn sequence fully completed!")
            return 0.0, 0.0, True, True

    turn_adjustment = turn_kp * heading_error

    # Determine direction of turn and apply speeds
    if heading_error > 0:  # Need to turn left
        left_speed_turn = -turn_speed_base # Turn left wheel backward
        right_speed_turn = turn_speed_base # Turn right wheel forward
    else:  # Need to turn right
        left_speed_turn = turn_speed_base # Turn left wheel forward
        right_speed_turn = -turn_speed_base # Turn right wheel backward

    # Apply proportional adjustment based on error, but ensure minimum turn speed
    # This keeps the robot turning even with small errors, avoiding oscillations near target.
    # The magnitude of turn_adjustment determines how aggressively it tries to correct.
    left_speed_turn -= turn_adjustment
    right_speed_turn += turn_adjustment

    left_speed_turn = max(-MAX_SPEED, min(MAX_SPEED, left_speed_turn))
    right_speed_turn = max(-MAX_SPEED, min(MAX_SPEED, right_speed_turn))

    print("TURNING_STATE: L_speed: {:.2f}, R_speed: {:.2f}".format(left_speed_turn, right_speed_turn))
    turn_counter += 1
    return left_speed_turn, right_speed_turn, False, False

def search_for_line_after_turn(gsValues):
    """
    IMPROVED: Enhanced line search after turning
    """
    global line_search_counter

    line_search_counter += 1

    line_error = calculate_line_error(gsValues)

    if line_error is not None:
        print("LINE_SEARCH: Line found after turn! Error: {:.3f}".format(line_error))
        return True, advanced_line_following_control(gsValues)

    if line_search_counter < LINE_SEARCH_DURATION:
        search_speed = base_speed * 0.3
        # Implement a subtle oscillate to search for the line
        if (line_search_counter // 5) % 2 == 0:
            return False, (search_speed * 0.8, search_speed * 1.2)
        else:
            return False, (search_speed * 1.2, search_speed * 0.8)
    else:
        print("LINE_SEARCH: Line search timeout - proceeding with forward motion")
        return True, (base_speed * 0.5, base_speed * 0.5)

def reset_turn_counters():
    """Reset all turn-related counters"""
    global stop_counter, turn_counter, post_turn_counter, line_search_counter
    stop_counter = 0
    turn_counter = 0
    post_turn_counter = 0
    line_search_counter = 0

# --- Main Logic Setup ---
start_node = "P3"
goal_node = "P8"
current_node = start_node
next_node_index = 1

# Generate path using Dijkstra's algorithm
path = find_path_dijkstra(start_node, goal_node, corrected_weighted_grid)
print("Path from {} to {}: {}".format(start_node, goal_node, ' -> '.join(path)))

# Determine initial robot heading (phi) based on the first segment of the path
phi = 0.0  # Initialize phi to a default (e.g., East)
if len(path) >= 2:
    initial_direction = get_direction_to_next_node(path[0], path[1], corrected_weighted_grid)
    print("INITIAL_SETUP: Determined initial direction: {}".format(initial_direction)) # Added debug print

    direction_angles_mapping = {
        'N': 1.57079632679, # np.pi / 2
        'E': 0.0,
        'S': -1.57079632679, # -np.pi / 2
        'W': 3.14159265359 # np.pi
    }

    if initial_direction and initial_direction in direction_angles_mapping:
        phi = direction_angles_mapping[initial_direction]
        print("Automated initial heading set to {} ({:.3f} rad)".format(initial_direction, phi))
        # Ensure initial encoder values are captured for first phi update
        # They will be refreshed on the first serial read, but this ensures a clean start.
        oldEncoderValues = [0.0, 0.0] # Or initial known encoder values
    else:
        print("WARNING: Could not determine initial direction for path. Defaulting initial heading to East (0°).")
        phi = 0.0
        oldEncoderValues = [0.0, 0.0]
else:
    print("WARNING: Path is too short ({} nodes). Cannot determine initial direction. Defaulting initial heading to East (0°).".format(len(path)))
    phi = 0.0
    oldEncoderValues = [0.0, 0.0]

print("INITIAL_SETUP: Phi value after initial assignment: {:.3f} rad".format(phi)) # Added debug print

if len(path) < 2:
    print("ERROR: No valid path found! Robot will stop.")
    path = [start_node]
    next_node_index = 0
    robot_state = 'stopping'
else:
    print("First target node in path: {}".format(path[next_node_index]))

robot_states = ['line_following', 'at_intersection', 'stopping', 'turning', 'post_turn_line_search',
                'at_intersection_drive_through']
robot_state = 'line_following' if len(path) >= 2 else 'stopping'

# --- Main MicroPython Loop ---
print("ESP32 Robot Logic Controller Started. Waiting for Webots data...")
leftSpeed = 0.0
rightSpeed = 0.0

# Loop for controlling the robot
# Loop for controlling the robot
while True:
    led_board.value(not led_board.value()) # Blink onboard LED to show activity

    # 1. Receive sensor data from Webots
    # Data format: "gs0_val,gs1_val,gs2_val,enc0_val,enc1_val\n"
    if uart.any():
        try:
            msg_bytes = uart.readline()
            if msg_bytes:
                msg_str = msg_bytes.decode('utf-8').strip()
                parts = msg_str.split(',')
                if len(parts) == 5:
                    gsValues = [float(parts[0]), float(parts[1]), float(parts[2])]
                    new_enc0 = float(parts[3])
                    new_enc1 = float(parts[4])

                    # Controleer of dit de eerste ontvangen data is
                    if not first_data_received:
                        oldEncoderValues[0] = new_enc0
                        oldEncoderValues[1] = new_enc1
                        encoderValues[0] = new_enc0
                        encoderValues[1] = new_enc1
                        first_data_received = True
                        print("DEBUG: Eerste data ontvangen. Initialiseren van encoderwaarden.") # Optionele debug print
                    else:
                        # Sla de huidige encoderwaarden op voordat ze worden bijgewerkt naar nieuwe waarden
                        oldEncoderValues[0] = encoderValues[0]
                        oldEncoderValues[1] = encoderValues[1]
                        encoderValues[0] = new_enc0
                        encoderValues[1] = new_enc1

                    # 2. Update odometry
                    wl, wr = get_wheels_speed(encoderValues, oldEncoderValues, delta_t)
                    u, w = get_robot_speeds(wl, wr, R, D)
                    phi = update_heading(w, phi, delta_t)
                    # print("MAIN_LOOP: Current Robot State: {}, Phi: {:.3f}".format(robot_state, phi)) # This information will now be sent back to Webots

                    # 3. Process logic based on robot_state
                    if current_node == goal_node:
                        print("Mission completed! Reached goal node.")
                        leftSpeed = 0.0
                        rightSpeed = 0.0
                        robot_state = 'stopping' # Ensure it stays stopped

                    elif not path or next_node_index >= len(path):
                        print("Path exhausted or invalid. Stopping robot.")
                        leftSpeed = 0.0
                        rightSpeed = 0.0
                        robot_state = 'stopping'

                    else:
                        target_node = path[next_node_index]

                        if robot_state == 'line_following':
                            intersection_detected = detect_intersection(gsValues)
                            if intersection_detected:
                                cross_counter += 1
                                if cross_counter >= CROSS_COUNTER_MIN:
                                    print("Intersection detected at {}. Target: {}".format(current_node, target_node))
                                    robot_state = 'at_intersection_drive_through'
                                    intersection_drive_through_counter = 0
                                    cross_counter = 0
                                    integral_error = 0.0
                                    error_history = []
                            else:
                                cross_counter = 0
                            leftSpeed, rightSpeed = advanced_line_following_control(gsValues)

                        elif robot_state == 'at_intersection_drive_through':
                            if intersection_drive_through_counter < INTERSECTION_DRIVE_THROUGH_DURATION:
                                leftSpeed, rightSpeed = base_speed * 0.4, base_speed * 0.4
                                intersection_drive_through_counter += 1
                            else:
                                print("Finished driving through intersection. Now at {}".format(target_node))
                                robot_state = 'at_intersection'
                                leftSpeed = 0.0
                                rightSpeed = 0.0

                        elif robot_state == 'at_intersection':
                            print("Updating current_node from {} to {}.".format(current_node, target_node))
                            current_node = target_node

                            if current_node == goal_node:
                                print("Reached goal node {}. Stopping.".format(current_node))
                                robot_state = 'stopping'
                                leftSpeed, rightSpeed = 0.0, 0.0
                            elif next_node_index < len(path) - 1:
                                next_node_index += 1
                                new_target_node_for_action = path[next_node_index]
                                direction_needed = get_direction_to_next_node(current_node, new_target_node_for_action,
                                                                              corrected_weighted_grid)
                                if direction_needed:
                                    turn_action, expected_heading_change = direction_to_robot_action(direction_needed, phi)
                                    print("Next seg: {}->{}, Dir: {}, Act: {}".format(current_node, new_target_node_for_action, direction_needed, turn_action))
                                    if turn_action == 'forward':
                                        print("Decision: Continue straight (forward).")
                                        robot_state = 'line_following'
                                        integral_error = 0.0
                                        error_history = []
                                        consecutive_no_line = 0
                                        leftSpeed, rightSpeed = advanced_line_following_control(gsValues)
                                    else:
                                        print("Decision: Initiate turn. Action: {}".format(turn_action))
                                        robot_state = 'turning'
                                        reset_turn_counters()
                                        leftSpeed = 0.0
                                        rightSpeed = 0.0
                                else:
                                    print("ERROR: No valid next direction from {} to {}. Stopping.".format(current_node, new_target_node_for_action))
                                    robot_state = 'stopping'
                                    leftSpeed, rightSpeed = 0.0, 0.0
                            else:
                                print("Path end reached. Current node: {}. Stopping.".format(current_node))
                                robot_state = 'stopping'
                                leftSpeed, rightSpeed = 0.0, 0.0

                        elif robot_state == 'turning':
                            leftSpeed, rightSpeed, phase_completed, turn_fully_completed = execute_improved_turn(turn_action)
                            if turn_fully_completed:
                                print("Turn sequence completed! Transitioning to line search.")
                                robot_state = 'post_turn_line_search'
                                line_search_counter = 0
                                leftSpeed = 0.0
                                rightSpeed = 0.0

                        elif robot_state == 'post_turn_line_search':
                            line_found, (leftSpeed, rightSpeed) = search_for_line_after_turn(gsValues)
                            if line_found:
                                print("Line acquired after turn. Continuing line following.")
                                robot_state = 'line_following'
                                integral_error = 0.0
                                error_history = []
                                consecutive_no_line = 0
                                previous_error = 0.0
                                reset_turn_counters()

                        elif robot_state == 'stopping':
                            leftSpeed = 0.0
                            rightSpeed = 0.0

                    # 4. Send motor commands and robot state back to Webots
                    # Data format: "left_speed,right_speed,phi,robot_state\n"
                    response_msg = "{:.4f},{:.4f},{:.3f},{}\n".format(leftSpeed, rightSpeed, phi, robot_state)
                    uart.write(response_msg.encode('utf-8'))
                    # print("Sent: {}".format(response_msg.strip())) # Uncomment for detailed debug on ESP32

                else:
                    print("Received malformed message: {}".format(msg_str))
        except Exception as e:
            print("Error processing serial data: {}".format(e))
            # Potentially reset state or motor speeds to safe values
            leftSpeed = 0.0
            rightSpeed = 0.0
            uart.write("{:.4f},{:.4f},0.0,{}\n".format(leftSpeed, rightSpeed, robot_state).encode('utf-8')) # Fallback with default phi if error

    # Small delay to prevent busy-waiting if no data is received
    sleep(0.01) # This sleep value is critical; adjust as needed for responsiveness