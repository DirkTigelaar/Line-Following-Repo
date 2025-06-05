# robot_webots_hil.py - Webots controller for Hardware-in-the-Loop simulation.
# This code sends sensor data to an external microcontroller and receives motor commands.

from controller import Robot
import serial
import time # Using time module for delays if needed, for serial buffer

# --- Configuration ---
# IMPORTANT: Change the port parameter according to your system and ESP32 connection
# On Windows, it's usually COMx (e.g., 'COM11'). On Linux/macOS, it's /dev/ttyUSBx or /dev/tty.usbserial-xxxx
SERIAL_PORT = 'COM3' 
BAUDRATE = 115200 

try:
    ser = serial.Serial(port=SERIAL_PORT, baudrate=BAUDRATE, timeout=0.1) # Shorter timeout for faster checks
    print(f"Successfully connected to serial port {SERIAL_PORT}") #
except serial.SerialException as e:
    print(f"ERROR: Could not open serial port {SERIAL_PORT}. Please check if the ESP32 is connected and the port is correct.") #
    print(f"Error details: {e}") 
    # Exit Webots simulation gracefully or handle error appropriately
    raise SystemExit(e) # Exit the controller if serial fails

# --- Robot Controller Setup ---
MAX_SPEED = 6.28 

robot = Robot() 
timestep = int(robot.getBasicTimeStep()) 

print("Webots HIL Robot Controller Started.") 

# Initialize devices
ps = [robot.getDevice(f'ps{i}') for i in range(8)] 
for sensor in ps: 
    sensor.enable(timestep) 

gs = [robot.getDevice(f'gs{i}') for i in range(3)] 
for sensor in gs: 
    sensor.enable(timestep) 

encoder = [robot.getDevice(name) for name in ['left wheel sensor', 'right wheel sensor']] 
for enc in encoder: 
    enc.enable(timestep) 

leftMotor = robot.getDevice('left wheel motor') 
rightMotor = robot.getDevice('right wheel motor') 
leftMotor.setPosition(float('inf')) 
rightMotor.setPosition(float('inf')) 
leftMotor.setVelocity(0.0) 
rightMotor.setVelocity(0.0) 

# --- Main Control Loop ---
last_sent_time = time.time() 
send_interval = timestep / 1000.0 # Convert timestep to seconds for sending frequency

while robot.step(timestep) != -1: 
    current_time = time.time() 

    # 1. Read sensor data from Webots robot
    gsValues = [gs[i].getValue() for i in range(3)] 
    # psValues = [ps[i].getValue() for i in range(8)] # Proximity sensors not used yet by logic, but can be sent
    encoderValues = [encoder[i].getValue() for i in range(2)] 

    # 2. Send sensor data to Microcontroller
    # Format: "gs0_val,gs1_val,gs2_val,enc0_val,enc1_val\n"
    # Ensure data is sent frequently enough for responsive control
    # Using a fixed interval rather than every timestep to avoid flooding the serial buffer, if necessary
    # For a timestep of 64ms, this will send at almost every step.
    if current_time - last_sent_time >= send_interval: 
        message_to_send = f"{gsValues[0]:.0f},{gsValues[1]:.0f},{gsValues[2]:.0f},{encoderValues[0]:.4f},{encoderValues[1]:.4f}\n" #
        try:
            ser.write(message_to_send.encode('utf-8')) 
            # print(f"Webots Sent: {message_to_send.strip()}") # Uncomment for detailed debug
        except serial.SerialTimeoutException: 
            print("WARNING: Serial write timeout. Microcontroller might be busy or disconnected.") #
        except Exception as e: #
            print(f"ERROR: Failed to write to serial: {e}") 
        last_sent_time = current_time 

    # 3. Receive motor commands from Microcontroller
    leftSpeed = 0.0 
    rightSpeed = 0.0 
    received_phi = 0.0 # New variable for phi
    received_robot_state = "" # New variable for robot_state

    if ser.in_waiting: 
        try:
            received_bytes = ser.readline() 
            if received_bytes: 
                received_str = received_bytes.decode('utf-8').strip() 
                parts = received_str.split(',') 
                # EXPECTING 4 PARTS: leftSpeed, rightSpeed, phi, robot_state
                if len(parts) == 4: 
                    leftSpeed = float(parts[0]) 
                    rightSpeed = float(parts[1]) 
                    received_phi = float(parts[2]) # Parse phi
                    received_robot_state = parts[3] # Parse robot_state
                    # Print received phi and state to Webots console
                    print(f"Webots Received - L:{leftSpeed:.2f}, R:{rightSpeed:.2f}, Phi:{received_phi:.3f}, State:{received_robot_state}")
                elif len(parts) == 2: # Fallback for old format if needed, but should be 4
                    leftSpeed = float(parts[0]) 
                    rightSpeed = float(parts[1]) 
                    print(f"Webots Received (Old Format) - L:{leftSpeed:.2f}, R:{rightSpeed:.2f}")
                else:
                    print(f"WARNING: Received malformed motor command: {received_str}") #
            # Clear any remaining junk in the buffer
            ser.flushInput() #
        except serial.SerialException as e: 
            print(f"ERROR: Serial read error: {e}") 
        except UnicodeDecodeError: #
            print("WARNING: Could not decode received serial data (possibly incomplete or corrupted).") #
        except ValueError: #
            print(f"WARNING: Could not convert received data to float/string: {received_str}") #
    
    # 4. Act: Set motor speeds in Webots
    leftMotor.setVelocity(leftSpeed) 
    rightMotor.setVelocity(rightSpeed) 

# Close serial port when Webots simulation ends
ser.close() #
print("Webots controller stopped and serial port closed.") 