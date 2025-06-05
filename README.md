## README: Hardware-in-the-Loop Line Following Robot

This project implements a Hardware-in-the-Loop (HIL) simulation setup for a line-following robot. The robot's environment and sensors are simulated in Webots, while the core control logic (pathfinding, PID control, odometry) runs on an external ESP32 microcontroller. Communication between Webots and the ESP32 occurs via a serial connection.

### 1. Dependencies

To reproduce this experiment, you will need the following:

#### 1.1. Software Dependencies

* **Python 3.x**: The Webots controller script (`line_following_with_HIL.py`) is written in Python.
    * **`pyserial` library**: This Python package is required for serial communication between the Webots controller and the ESP32.
        ```bash
        pip install pyserial
        ```
* **Webots R2023b or later**: The simulation environment for the robot. You can download it from the official Webots website.
* **MicroPython firmware for ESP32**: The `main.py` script runs on the ESP32, which requires MicroPython.
    * **`esptool`**: Used to flash MicroPython firmware to the ESP32.
        ```bash
        pip install esptool
        ```
    * **`ampy` or `thonny` (recommended)**: Tools for uploading Python scripts to the ESP32. Thonny IDE has an integrated MicroPython environment and file uploader.

#### 1.2. Hardware Dependencies

* **ESP32 Development Board**: Any standard ESP32 development board (e.g., ESP32-DevKitC, NodeMCU-32S) will work.
* **USB Cable**: To connect the ESP32 to your computer for flashing firmware and serial communication.

### 2. How to Reproduce the Experiment

Follow these steps to set up and run the HIL simulation:

#### 2.1. Prepare the ESP32 Microcontroller

1.  **Install MicroPython Firmware**:
    * Download the latest stable MicroPython firmware for your ESP32 board from the [MicroPython downloads page](https://micropython.org/download/esp32/). Look for a `.bin` file.
    * Erase the flash memory of your ESP32 (replace `COMx` with your ESP32's serial port):
        ```bash
        esptool.py --port COMx erase_flash
        ```
    * Flash the MicroPython firmware to your ESP32:
        ```bash
        esptool.py --chip esp32 --port COMx write_flash -z 0x1000 esp32-YYYYMMDD-vX.X.bin
        ```
        (Replace `esp32-YYYYMMDD-vX.X.bin` with your downloaded firmware file).

2.  **Upload `main.py` to ESP32**:
    * Connect your ESP32 to your computer via USB.
    * Use Thonny IDE (recommended) or `ampy` to upload the `main.py` script to the ESP32's filesystem as `main.py`.
        * **Using Thonny**: Open `main.py` in Thonny, ensure your ESP32 interpreter is selected (`Run` -> `Select interpreter` -> `MicroPython (ESP32)`), then go to `File` -> `Save as...` -> `MicroPython device` and save it as `main.py`.
        * **Using `ampy`**:
            ```bash
            ampy --port COMx put main.py
            ```
    * **Note on `main.py` UART Configuration**:
        The `main.py` script uses `uart = UART(1, 115200, tx=1, rx=3)`. This configuration specifies `UART1` with `TX` on GPIO1 and `RX` on GPIO3. On many ESP32 development boards, `UART0` (pins GPIO1 (TX) and GPIO3 (RX)) is dedicated to USB-to-Serial communication. If you are communicating with Webots over the *same USB serial connection* that you use to program the ESP32, you should typically use `UART(0, ...)` instead of `UART(1, ...)` in `main.py`.

        **Before proceeding, verify and adjust the `uart` configuration in `main.py` based on how your ESP32 board exposes its UART pins for communication, especially if you are using the onboard USB for HIL.** If using USB-Serial for HIL, `UART(0, 115200)` is typically correct.

#### 2.2. Set Up the Webots Simulation

1.  **Open or Create Webots Project**:
    * You'll need a Webots world (.wbt) file that contains a robot model equipped with:
        * 3 ground sensors (`gs0`, `gs1`, `gs2`) for line detection.
        * 2 wheel encoders (`left wheel sensor`, `right wheel sensor`).
        * 2 rotational motors (`left wheel motor`, `right wheel motor`).
    * Ensure the robot's controller field points to `line_following_with_HIL.py`.

2.  **Configure `line_following_with_HIL.py`**:
    * Open `line_following_with_HIL.py` in a text editor.
    * **IMPORTANT**: Modify the `SERIAL_PORT` variable to match the serial port your ESP32 is connected to.
        * On Windows, this will be `COMx` (e.g., `COM3`, `COM11`).
        * On Linux/macOS, this will be `/dev/ttyUSBx` or `/dev/tty.usbserial-xxxx`.
    * The `BAUDRATE` should remain `115200` as it's configured identically in `main.py`.

#### 2.3. Run the HIL Simulation

1.  **Start Webots Simulation**:
    * Open your Webots project.
    * Click the "Run" button (the play icon) in Webots.
    * The Webots controller (`line_following_with_HIL.py`) will attempt to establish a serial connection with the ESP32. Check the Webots console for "Successfully connected to serial port..." or any error messages.

2.  **Monitor Communication**:
    * **Webots Console**: You should see "Webots Sent: ..." and "Webots Received - L:..., R:..., Phi:..., State:..." messages indicating data exchange.
    * **ESP32 Serial Monitor**: If you have a serial monitor open for your ESP32 (e.g., in Thonny), you can see "Sent: ..." and "Received malformed message: ..." (if any) from the ESP32 side, as well as odometry and state updates.

3.  **Observe Robot Behavior**:
    * The robot in the Webots simulation should start moving, following the line, detecting intersections, and executing turns as dictated by the logic running on the ESP32.
    * The robot is programmed to navigate a predefined grid map from `P3` to `P8`. You can change these nodes to whichever once you'd like.

### 3. Expected Result

Upon successful setup and execution, the robot in the Webots simulation will:

* **Initialize Heading**: Set its initial heading (`phi`) based on the first segment of the calculated path from `P3` to `P8` or any other nodes if so desired.
* **Line Follow**: Use its ground sensors to follow the white lines on the track.
* **Detect Intersections**: Identify intersections based on multiple ground sensors detecting the line simultaneously.
* **Path Navigation**: At intersections, the robot will update its `current_node` and determine the `direction_needed` to the `next_node` in the Dijkstra-calculated path.
* **Execute Turns**: If a turn is required, the robot will enter a `turning` state, stop briefly, execute a closed-loop turn to achieve the `expected_heading_change`, and then search for the line again.
* **Reach Goal**: The robot will navigate through the defined grid until it reaches the `goal_node` (`P8`), at which point it will stop.

The serial output in both Webots and the ESP32's console will show real-time updates of sensor data, motor commands, robot heading (`phi`), and the robot's current operational `state`.
