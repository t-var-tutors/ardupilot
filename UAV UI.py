import sys
import subprocess
import threading
import time
import os
import signal
from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit,
    QPushButton, QComboBox, QTableWidget, QTableWidgetItem, QHeaderView, QMessageBox,
    QGroupBox, QAbstractItemView, QTextEdit
)
from PySide6.QtCore import Qt, QTimer, QThread, Signal, QObject
from PySide6.QtGui import QFont, QColor
from pymavlink import mavutil, mavwp

class SITLWorker(QObject):
    log_signal = Signal(str)
    finished = Signal()

    def run(self):
        try:
            self.log_signal.emit("Starting SITL simulation...")
            sitl_command = [
                "./Tools/autotest/sim_vehicle.py",
                "-v", "ArduCopter",
                "-f", "quad",
                "--console",
                "--map"
            ]
            self.log_signal.emit(f"Executing command: {' '.join(sitl_command)}")
            self.process = subprocess.Popen(
                sitl_command,
                cwd=os.path.expanduser("~/ardupilot"),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True
            )
            self.log_signal.emit("SITL process started. Waiting for output...")
            for line in self.process.stdout:
                self.log_signal.emit(line.strip())
            self.finished.emit()
        except Exception as e:
            self.log_signal.emit(f"Failed to launch SITL: {str(e)}")
            self.log_signal.emit(f"Current working directory: {os.getcwd()}")
            self.log_signal.emit(f"ArduPilot path: {os.path.expanduser('~/ardupilot')}")
            self.finished.emit()

class MAVWorker(QThread):
    log_signal = Signal(str)

    def __init__(self, parent=None, window=None):
        super().__init__(parent)
        self.window = window

    def run(self):
        try:
            self.log("Attempting to connect to SITL...")
            self.log("Waiting for heartbeat...")
            
            # Try to connect multiple times
            max_attempts = 5
            for attempt in range(max_attempts):
                try:
                    self.window.master = mavutil.mavlink_connection("127.0.0.1:14550")
                    self.window.master.wait_heartbeat()
                    self.log(f"Connection successful on attempt {attempt + 1}")
                    break
                except Exception as e:
                    if attempt < max_attempts - 1:
                        self.log(f"Connection attempt {attempt + 1} failed: {str(e)}")
                        self.log("Retrying in 2 seconds...")
                        time.sleep(2)
                    else:
                        raise Exception(f"Failed to connect after {max_attempts} attempts: {str(e)}")
            
            self.log("Heartbeat received. MAVLink connection established.")
            self.log(f"Connected to system {self.window.master.target_system}, component {self.window.master.target_component}")
            
            # Wait for system to be ready
            self.log("Waiting for system to be ready...")
            time.sleep(5)  # Initial wait for system initialization
            
            # Wait for GPS to initialize
            self.log("Waiting for GPS to initialize...")
            gps_ready = False
            for attempt in range(10):  # Try up to 10 times
                gps_msg = self.window.master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=0.5)
                if gps_msg:
                    self.log(f"GPS status: fix={gps_msg.fix_type}, satellites={gps_msg.satellites_visible}")
                    if gps_msg.fix_type >= 3:  # 3D fix
                        gps_ready = True
                        self.log("GPS is ready with 3D fix")
                        break
                    else:
                        self.log(f"GPS not ready yet, attempt {attempt + 1}/10")
                time.sleep(4)  # Wait 4 seconds between checks
            
            if not gps_ready:
                raise Exception("GPS not ready after maximum attempts")
            
            self.upload_waypoints()
            self.start_mission()
        except Exception as e:
            self.log(f"MAVLink connection failed: {str(e)}")
            self.log("Please ensure SITL is running and accessible on port 14550")

    def upload_waypoints(self):
        try:
            self.log("Starting waypoint upload...")
            wp = mavwp.MAVWPLoader()
            
            # Log waypoint details
            self.log(f"Number of waypoints to upload: {self.window.waypoint_table.rowCount()}")
            
            for i in range(self.window.waypoint_table.rowCount()):
                lat = float(self.window.waypoint_table.item(i, 2).text())
                lon = float(self.window.waypoint_table.item(i, 3).text())
                alt = float(self.window.waypoint_table.item(i, 4).text())
                self.log(f"Processing waypoint {i+1}: lat={lat}, lon={lon}, alt={alt}")
                
                frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
                command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
                autocontinue = 1
                current = 1 if i == 0 else 0
                wp.add(mavutil.mavlink.MAVLink_mission_item_message(
                    self.window.master.target_system,
                    self.window.master.target_component,
                    i,
                    frame,
                    command,
                    current,
                    autocontinue,
                    0, 0, 0, 0,
                    lat, lon, alt
                ))

            self.log("Clearing existing waypoints...")
            self.window.master.waypoint_clear_all_send()
            time.sleep(1)
            
            self.log(f"Sending waypoint count: {wp.count()}")
            self.window.master.waypoint_count_send(wp.count())
            
            for i in range(wp.count()):
                self.log(f"Waiting for waypoint request {i+1}...")
                msg = self.window.master.recv_match(type=['MISSION_REQUEST'], blocking=True)
                self.log(f"Sending waypoint {i+1}...")
                self.window.master.mav.send(wp.wp(msg.seq))
            
            self.log("Waypoints uploaded successfully.")
        except Exception as e:
            self.log(f"Failed to upload waypoints: {str(e)}")
            raise

    def start_mission(self):
        try:
            # Define constants
            max_mode_checks = 10
            max_arm_attempts = 10
            max_alt_checks = 10
            
            # Wait for system to be fully initialized
            self.log("Waiting for system initialization...")
            time.sleep(5)
            
            # Check current mode
            current_mode = self.window.master.flightmode
            self.log(f"Current flight mode: {current_mode}")
            
            # Get available modes
            self.log("Getting available modes...")
            mode_mapping = self.window.master.mode_mapping()
            self.log(f"Available modes: {mode_mapping}")
            
            # Check system status
            self.log("Checking system status...")
            msg = self.window.master.recv_match(type='SYS_STATUS', blocking=True)
            if msg:
                self.log(f"System status: {msg}")
            
            # Check GPS status
            self.log("Checking GPS status...")
            msg = self.window.master.recv_match(type='GPS_RAW_INT', blocking=True)
            if msg:
                self.log(f"GPS status: fix={msg.fix_type}, satellites={msg.satellites_visible}")
                if msg.fix_type < 3:  # Need at least 3D fix
                    raise Exception(f"GPS not ready. Fix type: {msg.fix_type}")
            
            # Wait for EKF to be ready
            self.log("Waiting for EKF to initialize...")
            max_ekf_attempts = 10
            for attempt in range(max_ekf_attempts):
                msg = self.window.master.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=1)
                if msg:
                    self.log(f"EKF status: flags={msg.flags}")
                    if msg.flags & 0x01:  # EKF has a good position estimate
                        self.log("EKF has good position estimate")
                        break
                if attempt < max_ekf_attempts - 1:
                    self.log(f"EKF not ready yet, attempt {attempt + 1}/{max_ekf_attempts}")
                    time.sleep(2)
                else:
                    raise Exception("EKF failed to initialize after maximum attempts")
            
            # Set home position to match our waypoints
            home_lat = -35.3632621
            home_lon = 149.1652374
            home_alt = 0.0
            
            self.log(f"Setting home position to lat={home_lat}, lon={home_lon}, alt={home_alt}")
            self.window.master.mav.command_long_send(
                self.window.master.target_system,
                self.window.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                0,  # Confirmation
                0,  # Use specified location
                0, 0, 0, 0,
                home_lat,
                home_lon,
                home_alt
            )
            
            # Wait for command acknowledgment
            self.log("Waiting for home position command acknowledgment...")
            msg = self.window.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
            if msg:
                self.log(f"Home position command acknowledgment: {msg}")
            
            # Wait for position estimate to be ready and match our home position
            self.log("Waiting for position estimate to be ready...")
            max_pos_attempts = 10
            for attempt in range(max_pos_attempts):
                msg = self.window.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
                if msg:
                    lat = msg.lat / 1e7  # Convert to degrees
                    lon = msg.lon / 1e7
                    alt = msg.alt / 1000.0  # Convert to meters
                    self.log(f"Position: lat={lat}, lon={lon}, alt={alt}")
                    
                    # Check if position is close to our home position
                    lat_diff = abs(lat - home_lat)
                    lon_diff = abs(lon - home_lon)
                    if lat_diff < 0.0001 and lon_diff < 0.0001:  # About 10 meters
                        self.log("Position estimate matches home position")
                        break
                
                if attempt < max_pos_attempts - 1:
                    self.log(f"Position not ready yet, attempt {attempt + 1}/{max_pos_attempts}")
                    time.sleep(2)
                else:
                    raise Exception("Position estimate failed to match home position")
            
            # Check GPS status
            gps = self.window.master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
            if gps:
                self.log(f"GPS status before takeoff: fix={gps.fix_type}, satellites={gps.satellites_visible}")
            
            # Check EKF status
            ekf = self.window.master.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=5)
            if ekf:
                self.log(f"EKF status before takeoff: {ekf}")
            
            # Set mode to GUIDED
            self.log("Setting mode to GUIDED...")
            self.window.master.set_mode('GUIDED')
            
            # Wait for mode change
            time.sleep(2)
            
            # Verify mode change
            for _ in range(10):  # Wait up to 5 seconds
                mode_msg = self.window.master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
                if mode_msg and mode_msg.custom_mode == 4:  # 4 is GUIDED mode
                    self.log("Successfully set to GUIDED mode")
                    break
                time.sleep(0.5)
            else:
                raise Exception("Failed to verify GUIDED mode")
            
            # Check if vehicle is armed
            if not self.window.master.motors_armed():
                self.log("Vehicle is not armed, attempting to arm...")
                
                # Wait for pre-arm checks to pass
                self.log("Waiting for pre-arm checks to pass...")
                max_prearm_attempts = 20
                prearm_ready = False
                ekf_ready = False
                
                for attempt in range(max_prearm_attempts):
                    # Check vehicle state
                    vehicle_state = self.window.master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
                    if vehicle_state:
                        self.log(f"Vehicle state: {vehicle_state}")
                        # Check if vehicle is ready to arm
                        if vehicle_state.system_status == 3:  # MAV_STATE_STANDBY
                            if ekf_ready:
                                prearm_ready = True
                                self.log("Vehicle is ready to arm")
                                break
                    
                    # Check for status messages for logging
                    status_msg = self.window.master.recv_match(type='STATUSTEXT', blocking=True, timeout=0.5)
                    if status_msg:
                        self.log(f"Status message: {status_msg.text}")
                    
                    # Check EKF status
                    ekf_msg = self.window.master.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=0.5)
                    if ekf_msg:
                        self.log(f"EKF status: flags={ekf_msg.flags}")
                        if ekf_msg.flags == 831:  # This is the magic number we're looking for
                            ekf_ready = True
                            self.log("EKF is ready with good position estimate")
                    
                    if attempt < max_prearm_attempts - 1:
                        self.log(f"Waiting for pre-arm checks... attempt {attempt + 1}/{max_prearm_attempts}")
                        time.sleep(1)
                    else:
                        raise Exception("Pre-arm checks did not pass after maximum attempts")
                
                if not prearm_ready:
                    raise Exception("Pre-arm checks did not pass")
                
                # Additional wait after pre-arm good
                self.log("Waiting additional time for systems to stabilize...")
                time.sleep(5)
                
                # Send arm command
                self.window.master.mav.command_long_send(
                    self.window.master.target_system,
                    self.window.master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 1, 0, 0, 0, 0, 0, 0
                )
                
                # Wait for command acknowledgment
                msg = self.window.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
                if msg:
                    self.log(f"Arm command acknowledgment: {msg}")
                
                # Wait for arming to complete and verify armed state
                self.log("Waiting for arming to complete...")
                armed = False
                disarm_detected = False
                
                # First check for immediate arming
                for _ in range(5):  # Check quickly for first 5 seconds
                    if self.window.master.motors_armed():
                        armed = True
                        self.log("Vehicle successfully armed")
                        break
                    time.sleep(0.5)  # Check more frequently
                
                # If not armed yet, check for any status messages
                if not armed:
                    for _ in range(10):  # Check for 5 more seconds
                        # Check armed state
                        if self.window.master.motors_armed():
                            armed = True
                            self.log("Vehicle successfully armed")
                            break
                        
                        # Check for status messages
                        status_msg = self.window.master.recv_match(type='STATUSTEXT', blocking=True, timeout=0.5)
                        if status_msg:
                            self.log(f"Status message: {status_msg.text}")
                            if "Disarming" in status_msg.text:
                                disarm_detected = True
                                self.log("Disarm detected, checking system status...")
                                # Check system status
                                sys_status = self.window.master.recv_match(type='SYS_STATUS', blocking=True, timeout=0.5)
                                if sys_status:
                                    self.log(f"System status: {sys_status}")
                                break
                        
                        time.sleep(0.5)
                
                if disarm_detected:
                    raise Exception("Vehicle disarmed immediately after arming attempt - possible safety check failure")
                elif not armed:
                    raise Exception("Failed to arm vehicle for takeoff - arming timeout")
            
            # Double check armed state before proceeding
            if not self.window.master.motors_armed():
                raise Exception("Vehicle is not armed after arming attempt")
            
            self.log("Vehicle is armed and ready for takeoff")
            
            # Get current position
            current_pos = self.window.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
            if not current_pos:
                raise Exception("Failed to get current position")
            
            # Convert altitude from millimeters to meters
            current_alt = current_pos.alt / 1000.0
            self.log(f"Current position: lat={current_pos.lat/1e7}, lon={current_pos.lon/1e7}, alt={current_alt}m")
            
            # Calculate target altitude (current + 10 meters)
            target_alt = current_alt + 10.0
            self.log(f"Sending takeoff command to altitude {target_alt}m...")
            
            # Send takeoff command
            self.window.master.mav.command_long_send(
                self.window.master.target_system,
                self.window.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,  # Confirmation
                0,  # Minimum pitch
                0,  # Empty
                0,  # Empty
                0,  # Empty
                0,  # Empty
                0,  # Empty
                target_alt  # Target altitude
            )
            
            # Wait for takeoff command acknowledgment
            self.log("Waiting for takeoff command acknowledgment...")
            msg = self.window.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            if msg:
                self.log(f"Takeoff command acknowledgment: {msg}")
                if msg.result != 0:
                    raise Exception(f"Takeoff command failed with result: {msg.result} ({mavutil.mavlink.enums['MAV_RESULT'][msg.result].name})")
            else:
                raise Exception("No acknowledgment received for takeoff command")
            
            # Wait for takeoff to start
            self.log("Waiting for takeoff to start...")
            takeoff_started = False
            for _ in range(20):  # Wait up to 10 seconds
                # Check for status messages
                status_msg = self.window.master.recv_match(type='STATUSTEXT', blocking=True, timeout=0.5)
                if status_msg:
                    self.log(f"Status message: {status_msg.text}")
                    if "Taking off" in status_msg.text:
                        takeoff_started = True
                        break
                
                # Check if we're climbing
                pos_msg = self.window.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=0.5)
                if pos_msg:
                    current_alt = pos_msg.alt / 1000.0  # Convert to meters
                    if current_alt > 0.5:  # If we've climbed more than 0.5m
                        takeoff_started = True
                        self.log(f"Takeoff detected - current altitude: {current_alt}m")
                        break
                
                time.sleep(0.5)
            
            if not takeoff_started:
                raise Exception("Takeoff did not start within timeout period")
            
            self.log("Takeoff started successfully")
            
            # Wait for takeoff to complete
            self.log("Waiting for takeoff to complete...")
            time.sleep(10)
            
            # Verify altitude with multiple checks
            for check in range(max_alt_checks):
                msg = self.window.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
                if msg:
                    # Use alt instead of relative_alt to get altitude above ground level
                    current_alt = msg.alt / 1000.0  # Convert to meters
                    self.log(f"Current altitude: {current_alt}m")
                    if current_alt >= target_alt * 0.9:  # Allow for 10% error
                        self.log("Takeoff successful - reached target altitude")
                        break
                    elif check < max_alt_checks - 1:
                        self.log(f"Waiting for altitude... attempt {check + 1}/{max_alt_checks}")
                        time.sleep(2)
                    else:
                        raise Exception(f"Takeoff failed. Current altitude {current_alt}m is below target {target_alt}m")
            
            # Only proceed with AUTO mode if takeoff was successful
            self.log("Switching to AUTO mode to start mission...")
            auto_mode = None
            for mode_name, mode_number in mode_mapping.items():
                if mode_name.upper() == "AUTO":
                    auto_mode = mode_number
                    break
            
            if auto_mode is None:
                raise Exception("AUTO mode not found in available modes")
            
            self.log(f"AUTO mode number: {auto_mode}")
            
            # Use command protocol to switch to AUTO mode
            self.window.master.mav.command_long_send(
                self.window.master.target_system,
                self.window.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                auto_mode,
                0, 0, 0, 0, 0
            )
            
            # Wait for command acknowledgment
            self.log("Waiting for command acknowledgment...")
            msg = self.window.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            if msg:
                self.log(f"Command acknowledgment: {msg}")
                if msg.result != 0:  # Check if command was successful
                    raise Exception(f"Failed to switch to AUTO mode: {msg.result}")
            
            # Wait for mode change
            time.sleep(5)
            
            # Verify mode change with multiple checks
            for check in range(max_mode_checks):
                # Get the current mode using STATUSTEXT message
                msg = self.window.master.recv_match(type='STATUSTEXT', blocking=True)
                if msg:
                    self.log(f"Status message: {msg.text}")
                    if "Mode AUTO" in msg.text:
                        self.log("Successfully verified AUTO mode")
                        break
                    
                if check < max_mode_checks - 1:
                    time.sleep(2)
                else:
                    # Instead of raising an exception, just log a warning and continue
                    self.log("Warning: Could not verify AUTO mode, but continuing with mission")
            
            self.log("Mission started successfully.")
        except Exception as e:
            self.log(f"Failed to start mission: {str(e)}")
            raise

    def log(self, message):
        self.log_signal.emit(message)

class UAVSimulatorUI(QWidget):
    def __init__(self):
        try:
            print("Initializing UAV Simulator UI...")
            super().__init__()
            self.setWindowTitle("UAV Simulator")
            self.setGeometry(100, 100, 800, 600)
            self.layout = QVBoxLayout()
            self.master = None
            self.is_paused = False
            self.is_initializing = True  # Add initialization flag
            self.original_value = None  # Add storage for original value
            
            # Create Logs directory if it doesn't exist
            self.logs_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Logs")
            if not os.path.exists(self.logs_dir):
                os.makedirs(self.logs_dir)
            
            # Create log file with timestamp
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            self.log_file_path = os.path.join(self.logs_dir, f"uav_simulation_{timestamp}.log")
            
            # Initialize logging
            self.log("=== UAV Simulation Started ===")
            self.log(f"Log file: {self.log_file_path}")
            self.log(f"Time: {time.strftime('%Y-%m-%d %H:%M:%S')}")
            self.log("============================")
            
            self.sitl_process = None

            self.log("Initializing UI components...")
            self.init_start_params()
            self.init_waypoints_table()
            
            # Starting position (home)
            start_lat = "-35.3632621"
            start_lon = "149.1652374"
            start_alt = "584.0"
            
            # Waypoint 1 position
            wp1_lat = "-35.3568921"
            wp1_lon = "149.1574474"
            
            self.log("Adding waypoints...")
            # Add waypoints
            self.add_waypoint(1, "Takeoff", start_lat, start_lon, start_alt, "0")
            self.add_waypoint(2, "Waypoint 1", wp1_lat, wp1_lon, "450", "100")
            self.add_waypoint(3, "Waypoint 2", start_lat, start_lon, "450", "100")
            self.add_waypoint(4, "Landing", start_lat, start_lon, "0", "100")
            
            self.log("Initializing controls...")
            self.init_controls()
            self.setLayout(self.layout)
            self.log("UI initialization complete.")
            
            # Set initialization flag to False after everything is set up
            self.is_initializing = False
            
        except Exception as e:
            print(f"Error initializing UI: {str(e)}")
            import traceback
            traceback.print_exc()
            raise

    def init_start_params(self):
        header = QLabel("Start parameters")
        header.setAlignment(Qt.AlignCenter)
        header.setStyleSheet("background: #b0b0b0; color: #111; font-weight: 600; font-size: 20px; padding: 10px 0; border-top-left-radius: 6px; border-top-right-radius: 6px; margin-bottom: 0px;")
        self.layout.addWidget(header)

        group = QGroupBox()
        group.setStyleSheet("QGroupBox { border: 1.5px solid #0074D9; border-radius: 6px; margin-top: 0px; padding: 8px; }")
        group_layout = QVBoxLayout()
        form_layout = QHBoxLayout()
        labels_layout = QVBoxLayout()
        fields_layout = QVBoxLayout()

        field_width = 200

        labels_layout.addWidget(QLabel("Model"))
        self.model_box = QComboBox()
        self.model_box.addItems(["DJI Mavic 3", "QuadCopter", "Plane"])
        self.model_box.setFixedWidth(field_width)
        self.model_box.setEditable(True)  # Make combo box editable
        fields_layout.addWidget(self.model_box)

        labels_layout.addWidget(QLabel("Latitude"))
        self.lat_input = QLineEdit("-35.3632621")
        self.lat_input.setFixedWidth(field_width)
        self.lat_input.setPlaceholderText("Enter latitude (-90 to 90)")
        self.lat_input.textChanged.connect(self.validate_lat)
        fields_layout.addWidget(self.lat_input)

        labels_layout.addWidget(QLabel("Longitude"))
        self.lon_input = QLineEdit("149.1652374")
        self.lon_input.setFixedWidth(field_width)
        self.lon_input.setPlaceholderText("Enter longitude (-180 to 180)")
        self.lon_input.textChanged.connect(self.validate_lon)
        fields_layout.addWidget(self.lon_input)

        labels_layout.addWidget(QLabel("Altitude"))
        self.alt_input = QLineEdit("584.0")
        self.alt_input.setFixedWidth(field_width)
        self.alt_input.setPlaceholderText("Enter altitude (0 to 10000m)")
        self.alt_input.textChanged.connect(self.validate_alt)
        fields_layout.addWidget(self.alt_input)

        labels_layout.addWidget(QLabel("Yaw"))
        self.yaw_input = QLineEdit("0")
        self.yaw_input.setFixedWidth(field_width)
        fields_layout.addWidget(self.yaw_input)

        labels_layout.addWidget(QLabel("Speed"))
        self.speed_input = QLineEdit("0")
        self.speed_input.setFixedWidth(field_width)
        self.speed_input.setPlaceholderText("Enter speed (0 to 100 m/s)")
        self.speed_input.textChanged.connect(self.validate_speed)
        fields_layout.addWidget(self.speed_input)

        form_layout.addLayout(labels_layout)
        form_layout.addLayout(fields_layout)
        form_layout.addStretch()
        group_layout.addLayout(form_layout)
        group.setLayout(group_layout)
        self.layout.addWidget(group)

    def validate_waypoint_coordinates(self, item):
        if self.is_initializing:
            return
            
        try:
            # Get the column of the changed item
            col = item.column()
            
            # Get the text value
            text = item.text().strip()
            
            # Skip validation if empty
            if not text:
                item.setBackground(QColor("white"))
                item.setToolTip("")
                return

            # Try to convert to float
            value = float(text)
            
            # Validate based on column
            if col == 2:  # Latitude
                if value < -90 or value > 90:
                    QMessageBox.warning(self, "Invalid Latitude", 
                        "Latitude must be between -90 and 90 degrees.")
                    item.setText(self.original_value)
                    return
                # Format to 7 decimal places
                formatted = f"{value:.7f}"
                if formatted != text:
                    item.setText(formatted)
            elif col == 3:  # Longitude
                if value < -180 or value > 180:
                    QMessageBox.warning(self, "Invalid Longitude", 
                        "Longitude must be between -180 and 180 degrees.")
                    item.setText(self.original_value)
                    return
                # Format to 7 decimal places
                formatted = f"{value:.7f}"
                if formatted != text:
                    item.setText(formatted)
            elif col == 4:  # Altitude
                if value < 0 or value > 10000:
                    QMessageBox.warning(self, "Invalid Altitude", 
                        "Altitude must be between 0 and 10000 meters.")
                    item.setText(self.original_value)
                    return
                # Format to integer
                formatted = f"{int(value)}"
                if formatted != text:
                    item.setText(formatted)
            elif col == 5:  # Speed
                if value < 0 or value > 100:
                    QMessageBox.warning(self, "Invalid Speed", 
                        "Speed must be between 0 and 100 meters per second.")
                    item.setText(self.original_value)
                    return
                # Format to integer
                formatted = f"{int(value)}"
                if formatted != text:
                    item.setText(formatted)
            
            # If we get here, the value is valid
            item.setBackground(QColor("white"))
            item.setToolTip("")
            
        except ValueError:
            # Only show error if this is not during initialization
            if not self.is_initializing:
                error_msg = "Please enter a valid number.\n"
                if col == 2:
                    error_msg += "Latitude must be between -90 and 90 degrees."
                elif col == 3:
                    error_msg += "Longitude must be between -180 and 180 degrees."
                elif col == 4:
                    error_msg += "Altitude must be between 0 and 10000 meters."
                elif col == 5:
                    error_msg += "Speed must be between 0 and 100 meters per second."
                
                QMessageBox.warning(self, "Invalid Input", error_msg)
                # Reset to original value after showing the error message
                item.setText(self.original_value)
                item.setToolTip(error_msg)
                # Force the table to update
                self.waypoint_table.viewport().update()

    def validate_lat(self, text):
        if self.is_initializing:
            return
            
        try:
            # Store the original value
            original_value = self.lat_input.text()
            
            # Remove any whitespace
            text = text.strip()
            
            # Check if empty
            if not text:
                self.lat_input.setStyleSheet("")
                self.lat_input.setToolTip("")
                return
            
            # Try to convert to float
            value = float(text)
            
            # Check range
            if value < -90 or value > 90:
                QMessageBox.warning(self, "Invalid Latitude", "Latitude must be between -90 and 90 degrees")
                self.lat_input.setText(original_value)
                return
            
            # Format to 7 decimal places
            formatted = f"{value:.7f}"
            if formatted != text:
                # Only update if different to avoid cursor jumping
                self.lat_input.setText(formatted)
            
            self.lat_input.setStyleSheet("")
            self.lat_input.setToolTip("")
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", "Please enter a valid number")
            self.lat_input.setText(original_value)
            self.lat_input.setToolTip("Latitude must be a number between -90 and 90 degrees")

    def validate_lon(self, text):
        if self.is_initializing:
            return
            
        try:
            # Store the original value
            original_value = self.lon_input.text()
            
            # Remove any whitespace
            text = text.strip()
            
            # Check if empty
            if not text:
                self.lon_input.setStyleSheet("")
                self.lon_input.setToolTip("")
                return
            
            # Try to convert to float
            value = float(text)
            
            # Check range
            if value < -180 or value > 180:
                QMessageBox.warning(self, "Invalid Longitude", "Longitude must be between -180 and 180 degrees")
                self.lon_input.setText(original_value)
                return
            
            # Format to 7 decimal places
            formatted = f"{value:.7f}"
            if formatted != text:
                # Only update if different to avoid cursor jumping
                self.lon_input.setText(formatted)
            
            self.lon_input.setStyleSheet("")
            self.lon_input.setToolTip("")
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", "Please enter a valid number")
            self.lon_input.setText(original_value)
            self.lon_input.setToolTip("Longitude must be a number between -180 and 180 degrees")

    def validate_alt(self, text):
        if self.is_initializing:
            return
            
        try:
            # Store the original value
            original_value = self.alt_input.text()
            
            # Remove any whitespace
            text = text.strip()
            
            # Check if empty
            if not text:
                self.alt_input.setStyleSheet("")
                self.alt_input.setToolTip("")
                return
            
            # Try to convert to float
            value = float(text)
            
            # Check range
            if value < 0 or value > 10000:
                QMessageBox.warning(self, "Invalid Altitude", "Altitude must be between 0 and 10000 meters")
                self.alt_input.setText(original_value)
                return
            
            # Format to integer
            formatted = f"{int(value)}"
            if formatted != text:
                # Only update if different to avoid cursor jumping
                self.alt_input.setText(formatted)
            
            self.alt_input.setStyleSheet("")
            self.alt_input.setToolTip("")
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", "Please enter a valid number")
            self.alt_input.setText(original_value)
            self.alt_input.setToolTip("Altitude must be a number between 0 and 10000 meters")

    def validate_speed(self, text):
        if self.is_initializing:
            return
            
        try:
            # Store the original value
            original_value = self.speed_input.text()
            
            # Remove any whitespace
            text = text.strip()
            
            # Check if empty
            if not text:
                self.speed_input.setStyleSheet("")
                self.speed_input.setToolTip("")
                return
            
            # Try to convert to float
            value = float(text)
            
            # Check range
            if value < 0 or value > 100:
                QMessageBox.warning(self, "Invalid Speed", "Speed must be between 0 and 100 m/s")
                self.speed_input.setText(original_value)
                return
            
            # Format to integer
            formatted = f"{int(value)}"
            if formatted != text:
                # Only update if different to avoid cursor jumping
                self.speed_input.setText(formatted)
            
            self.speed_input.setStyleSheet("")
            self.speed_input.setToolTip("")
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", "Please enter a valid number")
            self.speed_input.setText(original_value)
            self.speed_input.setToolTip("Speed must be a number between 0 and 100 m/s")

    def init_waypoints_table(self):
        header = QLabel("Waypoints")
        header.setAlignment(Qt.AlignCenter)
        header.setStyleSheet("background: #b0b0b0; color: #111; font-weight: 600; font-size: 20px; padding: 10px 0; border-top-left-radius: 6px; border-top-right-radius: 6px; margin-bottom: 0px;")
        self.layout.addWidget(header)

        group = QGroupBox()
        group.setStyleSheet("QGroupBox { border: 1.5px solid #0074D9; border-radius: 6px; margin-top: 0px; padding: 8px; }")
        group_layout = QVBoxLayout()

        self.waypoint_table = QTableWidget(0, 7)  # Keep 7 columns internally
        self.waypoint_table.setHorizontalHeaderLabels(["#", "Name", "Latitude", "Longitude", "Altitude (m)", "Speed (m/s)", "Actions"])
        header = self.waypoint_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.Fixed)
        header.setSectionResizeMode(1, QHeaderView.Fixed)
        header.setSectionResizeMode(2, QHeaderView.Fixed)
        header.setSectionResizeMode(3, QHeaderView.Fixed)
        header.setSectionResizeMode(4, QHeaderView.Fixed)
        header.setSectionResizeMode(5, QHeaderView.Fixed)
        header.setSectionResizeMode(6, QHeaderView.Stretch)
        
        # Hide the first column
        self.waypoint_table.hideColumn(0)
        
        # Adjust column widths for visible columns
        self.waypoint_table.setColumnWidth(1, 120)  # Name (wider)
        self.waypoint_table.setColumnWidth(2, 100)  # Latitude
        self.waypoint_table.setColumnWidth(3, 100)  # Longitude
        self.waypoint_table.setColumnWidth(4, 100)  # Altitude (m) (wider)
        self.waypoint_table.setColumnWidth(5, 100)  # Speed (m/s) (wider)
        self.waypoint_table.setColumnWidth(6, 180)
        
        # Make all columns except Actions editable with single click
        self.waypoint_table.setEditTriggers(
            QAbstractItemView.CurrentChanged |  # Start editing when cell is selected
            QAbstractItemView.AnyKeyPressed |   # Start editing when any key is pressed
            QAbstractItemView.EditKeyPressed    # Start editing when edit key (F2) is pressed
        )
        
        # Connect cell change event
        self.waypoint_table.itemChanged.connect(self.validate_waypoint_coordinates)
        
        # Connect cell click event for actions
        self.waypoint_table.cellClicked.connect(self.handle_waypoint_action)
        
        # Connect cell editing started event
        self.waypoint_table.itemSelectionChanged.connect(self.store_original_value)
        
        group_layout.addWidget(self.waypoint_table)
        
        # Add button layout below the table
        button_layout = QHBoxLayout()
        
        # Create Add Waypoint button and store reference
        self.add_waypoint_button = QPushButton("Add Waypoint")
        self.add_waypoint_button.setStyleSheet("""
            QPushButton {
                background-color: #2ECC40;
                color: white;
                font-weight: bold;
                border-radius: 4px;
                padding: 6px 18px;
                min-width: 120px;
            }
            QPushButton:hover {
                background-color: #27AE60;
            }
        """)
        self.add_waypoint_button.clicked.connect(self.add_new_waypoint)
        button_layout.addWidget(self.add_waypoint_button)
        button_layout.addStretch()
        
        group_layout.addLayout(button_layout)
        group.setLayout(group_layout)
        self.layout.addWidget(group)

    def store_original_value(self):
        # Store the original value when editing starts
        selected_items = self.waypoint_table.selectedItems()
        if selected_items:
            item = selected_items[0]
            self.original_value = item.text()

    def handle_waypoint_action(self, row, column):
        if column == 6:  # Actions column
            item = self.waypoint_table.item(row, column)
            if item is None:
                return
                
            # Get the click position relative to the item
            cursor_pos = self.waypoint_table.viewport().mapFromGlobal(self.waypoint_table.cursor().pos())
            item_rect = self.waypoint_table.visualItemRect(item)
            relative_x = cursor_pos.x() - item_rect.x()
            
            # Calculate the width of each action (Delete and Download)
            total_width = item_rect.width()
            action_width = total_width / 2
            
            if relative_x < action_width:
                # Delete action
                self.delete_waypoint(row)
            else:
                # Download action
                self.download_waypoint(row)

    def delete_waypoint(self, row):
        try:
            # Temporarily block signals to prevent validation
            self.waypoint_table.blockSignals(True)
            
            # Remove the row
            self.waypoint_table.removeRow(row)
            
            # Update the waypoint numbers in the hidden column
            for i in range(row, self.waypoint_table.rowCount()):
                number_item = self.waypoint_table.item(i, 0)
                if number_item:
                    number_item.setText(str(i + 1))
            
            # Update waypoint names
            total_waypoints = self.waypoint_table.rowCount()
            for i in range(total_waypoints):
                name_item = self.waypoint_table.item(i, 1)
                if name_item:
                    if i == 0:
                        name_item.setText("Takeoff")
                    elif i == total_waypoints - 1:
                        name_item.setText("Landing")
                    else:
                        name_item.setText(f"Waypoint {i}")
            
            # Re-enable signals
            self.waypoint_table.blockSignals(False)
            
            self.log(f"Waypoint {row + 1} deleted")
        except Exception as e:
            # Make sure to re-enable signals even if there's an error
            self.waypoint_table.blockSignals(False)
            self.log(f"Error deleting waypoint: {str(e)}")

    def download_waypoint(self, row):
        # Placeholder for future download functionality
        self.log(f"Download functionality for waypoint {row + 1} will be implemented later")

    def add_waypoint(self, number, name, lat, lon, alt, speed):
        row = self.waypoint_table.rowCount()
        self.waypoint_table.insertRow(row)

        def create_item(text):
            item = QTableWidgetItem(text)
            item.setTextAlignment(Qt.AlignCenter)
            font = QFont()
            font.setUnderline(False)
            item.setFont(font)
            return item

        def create_action_item():
            # Create a custom widget for the actions
            widget = QWidget()
            layout = QHBoxLayout()
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setSpacing(20)  # Space between the links
            
            # Create Delete link
            delete_label = QLabel("Delete")
            delete_label.setStyleSheet("color: #0074D9; text-decoration: underline;")
            delete_label.setCursor(Qt.PointingHandCursor)
            delete_label.setAlignment(Qt.AlignCenter)
            delete_label.mousePressEvent = lambda e: self.delete_waypoint(row)
            
            # Create Download link
            download_label = QLabel("Download")
            download_label.setStyleSheet("color: #0074D9; text-decoration: underline;")
            download_label.setCursor(Qt.PointingHandCursor)
            download_label.setAlignment(Qt.AlignCenter)
            download_label.mousePressEvent = lambda e: self.download_waypoint(row)
            
            # Add stretch to center the labels
            layout.addStretch()
            layout.addWidget(delete_label)
            layout.addWidget(download_label)
            layout.addStretch()
            
            widget.setLayout(layout)
            return widget

        # Temporarily disconnect the validation signal
        self.waypoint_table.blockSignals(True)

        # Add all items including the hidden number column
        self.waypoint_table.setItem(row, 0, create_item(str(number)))
        self.waypoint_table.setItem(row, 1, create_item(name))
        self.waypoint_table.setItem(row, 2, create_item(lat))
        self.waypoint_table.setItem(row, 3, create_item(lon))
        self.waypoint_table.setItem(row, 4, create_item(alt))
        self.waypoint_table.setItem(row, 5, create_item(speed))
        self.waypoint_table.setCellWidget(row, 6, create_action_item())

        # Reconnect the validation signal
        self.waypoint_table.blockSignals(False)

    def init_controls(self):
        control_layout = QHBoxLayout()
        control_layout.addStretch()

        self.start_button = QPushButton("Start simulation")
        self.start_button.setStyleSheet("background-color: #2ECC40; color: white; font-weight: bold; border-radius: 4px; padding: 6px 18px;")
        self.start_button.clicked.connect(self.launch_sitl)
        control_layout.addWidget(self.start_button)

        self.pause_button = QPushButton("Pause simulation")
        self.pause_button.setEnabled(False)
        self.pause_button.setStyleSheet("background-color: lightgray; color: black; font-weight: normal; border-radius: 4px; padding: 6px 18px;")
        self.pause_button.clicked.connect(self.pause_simulation)
        control_layout.addWidget(self.pause_button)

        self.stop_button = QPushButton("End simulation")
        self.stop_button.setEnabled(False)
        self.stop_button.setStyleSheet("background-color: lightgray; color: black; font-weight: normal; border-radius: 4px; padding: 6px 18px;")
        self.stop_button.clicked.connect(self.end_simulation)  # Connect the stop button
        control_layout.addWidget(self.stop_button)

        control_layout.addStretch()
        self.layout.addLayout(control_layout)

    def end_simulation(self):
        try:
            self.log("=== Ending Simulation ===")
            
            # Re-enable waypoint table and Add Waypoint button
            self.waypoint_table.setEnabled(True)
            self.add_waypoint_button.setEnabled(True)
            self.add_waypoint_button.setStyleSheet("""
                QPushButton {
                    background-color: #2ECC40;
                    color: white;
                    font-weight: bold;
                    border-radius: 4px;
                    padding: 6px 18px;
                    min-width: 120px;
                }
                QPushButton:hover {
                    background-color: #27AE60;
                }
            """)
            
            # Disarm the vehicle if connected
            if self.master:
                try:
                    self.log("Disarming vehicle...")
                    self.master.mav.command_long_send(
                        self.master.target_system,
                        self.master.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                        0, 0, 0, 0, 0, 0, 0, 0
                    )
                    time.sleep(1)
                except Exception as e:
                    self.log(f"Error while disarming: {str(e)}")

            # Kill all ArduPilot related processes
            self.log("Terminating all ArduPilot processes...")
            try:
                # Kill the main SITL process
                if hasattr(self, 'sitl_worker') and hasattr(self.sitl_worker, 'process'):
                    self.sitl_worker.process.terminate()
                    self.sitl_worker.process.wait(timeout=5)
                
                # Kill all related processes
                subprocess.run(['pkill', '-f', 'sim_vehicle.py'], timeout=5)
                subprocess.run(['pkill', '-f', 'arducopter'], timeout=5)
                subprocess.run(['pkill', '-f', 'mavproxy.py'], timeout=5)
                
                # Additional cleanup for any remaining processes
                subprocess.run(['killall', '-9', 'sim_vehicle.py'], timeout=5)
                subprocess.run(['killall', '-9', 'arducopter'], timeout=5)
                subprocess.run(['killall', '-9', 'mavproxy.py'], timeout=5)
                
                self.log("All ArduPilot processes terminated.")
            except Exception as e:
                self.log(f"Error terminating processes: {str(e)}")
                # Try force kill as last resort
                try:
                    subprocess.run(['pkill', '-9', '-f', 'sim_vehicle.py'])
                    subprocess.run(['pkill', '-9', '-f', 'arducopter'])
                    subprocess.run(['pkill', '-9', '-f', 'mavproxy.py'])
                except:
                    pass

            # Reset UI state
            self.start_button.setEnabled(True)
            self.start_button.setStyleSheet("background-color: #2ECC40; color: white; font-weight: bold; border-radius: 4px; padding: 6px 18px;")
            self.pause_button.setEnabled(False)
            self.pause_button.setStyleSheet("background-color: lightgray; color: black; font-weight: normal; border-radius: 4px; padding: 6px 18px;")
            self.stop_button.setEnabled(False)
            self.stop_button.setStyleSheet("background-color: lightgray; color: black; font-weight: normal; border-radius: 4px; padding: 6px 18px;")
            
            self.log("=== Simulation Ended Successfully ===")
            
        except Exception as e:
            self.log(f"Error ending simulation: {str(e)}")
            # Force reset UI state even if there's an error
            self.start_button.setEnabled(True)
            self.start_button.setStyleSheet("background-color: #2ECC40; color: white; font-weight: bold; border-radius: 4px; padding: 6px 18px;")
            self.pause_button.setEnabled(False)
            self.pause_button.setStyleSheet("background-color: lightgray; color: black; font-weight: normal; border-radius: 4px; padding: 6px 18px;")
            self.stop_button.setEnabled(False)
            self.stop_button.setStyleSheet("background-color: lightgray; color: black; font-weight: normal; border-radius: 4px; padding: 6px 18px;")
            # Make sure waypoint table and Add Waypoint button are re-enabled even if there's an error
            self.waypoint_table.setEnabled(True)
            self.add_waypoint_button.setEnabled(True)
            self.add_waypoint_button.setStyleSheet("""
                QPushButton {
                    background-color: #2ECC40;
                    color: white;
                    font-weight: bold;
                    border-radius: 4px;
                    padding: 6px 18px;
                    min-width: 120px;
                }
                QPushButton:hover {
                    background-color: #27AE60;
                }
            """)

    def launch_sitl(self):
        try:
            print("Launching SITL...")
            self.start_button.setEnabled(False)
            self.start_button.setStyleSheet("background-color: lightgray; color: black; font-weight: normal; border-radius: 4px; padding: 6px 18px;")
            self.pause_button.setEnabled(True)
            self.pause_button.setStyleSheet("background-color: orange; color: black; font-weight: normal; border-radius: 4px; padding: 6px 18px;")
            self.stop_button.setEnabled(True)
            self.stop_button.setStyleSheet("background-color: red; color: black; font-weight: normal; border-radius: 4px; padding: 6px 18px;")

            # Disable waypoint table and Add Waypoint button
            self.waypoint_table.setEnabled(False)
            self.add_waypoint_button.setEnabled(False)
            self.add_waypoint_button.setStyleSheet("""
                QPushButton {
                    background-color: lightgray;
                    color: black;
                    font-weight: normal;
                    border-radius: 4px;
                    padding: 6px 18px;
                    min-width: 120px;
                }
            """)

            print("Creating SITL worker...")
            self.sitl_worker = SITLWorker()
            self.sitl_thread = QThread()
            self.sitl_worker.moveToThread(self.sitl_thread)
            self.sitl_thread.started.connect(self.sitl_worker.run)
            self.sitl_worker.log_signal.connect(self.log)
            self.sitl_worker.finished.connect(self.sitl_thread.quit)
            print("Starting SITL thread...")
            self.sitl_thread.start()

            print("Creating MAVLink worker...")
            self.mav_worker = MAVWorker(window=self)
            self.mav_worker.log_signal.connect(self.log)
            print("Starting MAVLink worker...")
            self.mav_worker.start()
            print("SITL launch complete.")
            
        except Exception as e:
            print(f"Error launching SITL: {str(e)}")
            import traceback
            traceback.print_exc()
            self.log(f"Failed to launch SITL: {str(e)}")
            # Re-enable waypoint table and Add Waypoint button if there's an error
            self.waypoint_table.setEnabled(True)
            self.add_waypoint_button.setEnabled(True)
            self.add_waypoint_button.setStyleSheet("""
                QPushButton {
                    background-color: #2ECC40;
                    color: white;
                    font-weight: bold;
                    border-radius: 4px;
                    padding: 6px 18px;
                    min-width: 120px;
                }
                QPushButton:hover {
                    background-color: #27AE60;
                }
            """)

    def log(self, message):
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        log_message = f"[{timestamp}] {message}"
        print(log_message)
        try:
            with open(self.log_file_path, "a") as f:
                f.write(log_message + "\n")
        except Exception as e:
            print(f"Error writing to log file: {str(e)}")

    def add_new_waypoint(self):
        try:
            # Check if we've reached the maximum number of waypoints
            if self.waypoint_table.rowCount() >= 10:
                QMessageBox.warning(self, "Maximum Waypoints Reached", 
                    "You have reached the maximum limit of 10 waypoints.\n"
                    "Please delete some waypoints before adding new ones.")
                return

            # Set initialization flag to prevent validation errors
            self.is_initializing = True
            
            # Get the last waypoint's position as a starting point
            last_row = self.waypoint_table.rowCount() - 1
            if last_row >= 0:
                lat = self.waypoint_table.item(last_row, 2).text()
                lon = self.waypoint_table.item(last_row, 3).text()
                alt = self.waypoint_table.item(last_row, 4).text()
            else:
                # Use default values if no waypoints exist
                lat = "-35.3632621"
                lon = "149.1652374"
                alt = "584.0"
            
            # Add new waypoint
            number = self.waypoint_table.rowCount() + 1
            
            # Determine the name based on position
            if number == 1:
                name = "Takeoff"
            elif number == 2:
                name = "Waypoint 1"
            else:
                name = f"Waypoint {number-1}"
            
            self.add_waypoint(number, name, lat, lon, alt, "100")
            self.log(f"Added new waypoint {number}")
            
            # If this is the second waypoint, rename the last one to "Landing"
            if number == 2:
                self.waypoint_table.item(0, 1).setText("Takeoff")
            elif number > 2:
                # Rename the previous waypoint to "Waypoint X"
                prev_row = number - 2
                self.waypoint_table.item(prev_row, 1).setText(f"Waypoint {prev_row}")
                # Rename the last waypoint to "Landing"
                self.waypoint_table.item(number-1, 1).setText("Landing")
            
            # Reset initialization flag after all operations are complete
            self.is_initializing = False
            
        except Exception as e:
            self.is_initializing = False  # Make sure to reset the flag even if there's an error
            self.log(f"Error adding new waypoint: {str(e)}")

    def create_mission(self):
        """Create a mission with the current waypoints"""
        try:
            self.log("Starting mission creation...")
            
            # Debug connection
            self.log("Checking connection...")
            try:
                self.window.master.wait_heartbeat()
                self.log("Heartbeat received")
            except Exception as e:
                self.log(f"Connection error: {str(e)}")
                return False
            
            # Check initial state
            mode = self.window.master.flightmode
            armed = self.window.master.motors_armed()
            self.log(f"Initial state check - Mode: {mode}, Armed: {armed}")
            
            if not armed:
                self.log("Warning: Vehicle not armed!")
                return False
            
            # Get current position and altitude
            self.log("Getting current position...")
            current_pos = self.window.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
            if not current_pos:
                raise Exception("Failed to get current position")
            
            current_alt = current_pos.alt / 1000.0
            current_lat = current_pos.lat / 1e7
            current_lon = current_pos.lon / 1e7
            
            self.log(f"Initial position: lat={current_lat}, lon={current_lon}, alt={current_alt}m")
            
            # Get current velocity
            vel = self.window.master.recv_match(type='VFR_HUD', blocking=True, timeout=0.5)
            if vel:
                self.log(f"Initial velocity: {vel.airspeed}m/s")
            
            # Calculate target altitude (current + 20m)
            target_alt = current_alt + 20.0
            self.log(f"Setting target altitude to {target_alt}m")
            
            # Send direct takeoff command
            self.log("Sending takeoff command...")
            self.window.master.mav.command_long_send(
                self.window.master.target_system,
                self.window.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,  # Confirmation
                0,  # Minimum pitch
                0,  # Empty
                0,  # Empty
                0,  # Empty
                0,  # Empty
                0,  # Empty
                target_alt  # Target altitude
            )
            
            # Wait for command acknowledgment
            msg = self.window.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            if msg:
                self.log(f"Takeoff command acknowledgment: {msg}")
                if msg.result != 0:
                    raise Exception(f"Takeoff command failed with result: {msg.result}")
            else:
                raise Exception("No acknowledgment received for takeoff command")
            
            self.log("Takeoff command sent successfully")
            return True
            
        except Exception as e:
            self.log(f"Error creating mission: {str(e)}")
            return False

    def start_mission(self):
        """Start the mission"""
        try:
            # Debug connection
            self.log("Checking connection...")
            try:
                self.window.master.wait_heartbeat()
                self.log("Heartbeat received")
            except Exception as e:
                self.log(f"Connection error: {str(e)}")
                return False
            
            # Check initial state before starting
            mode = self.window.master.flightmode
            armed = self.window.master.motors_armed()
            self.log(f"Pre-mission state check - Mode: {mode}, Armed: {armed}")
            
            if not armed:
                self.log("Warning: Vehicle not armed before mission start!")
                return False
            
            # Switch to GUIDED mode for direct control
            self.log("Switching to GUIDED mode...")
            self.window.master.set_mode('GUIDED')
            
            # Wait for command acknowledgment
            self.log("Waiting for command acknowledgment...")
            msg = self.window.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            if msg:
                self.log(f"Command acknowledgment: {msg}")
                if msg.result != 0:
                    raise Exception(f"Mode change failed with result: {msg.result}")
            else:
                raise Exception("No acknowledgment received for mode change")
            
            # Verify mode change
            time.sleep(1)  # Give time for mode change to take effect
            new_mode = self.window.master.flightmode
            self.log(f"Mode after change: {new_mode}")
            
            if new_mode != 'GUIDED':
                raise Exception(f"Failed to change to GUIDED mode, current mode: {new_mode}")
            
            # Send takeoff command in GUIDED mode
            self.log("Sending takeoff command in GUIDED mode...")
            self.window.master.mav.command_long_send(
                self.window.master.target_system,
                self.window.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,  # Confirmation
                0,  # Minimum pitch
                0,  # Empty
                0,  # Empty
                0,  # Empty
                0,  # Empty
                0,  # Empty
                20.0  # Target altitude (20m)
            )
            
            # Wait for command acknowledgment
            msg = self.window.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            if msg:
                self.log(f"Takeoff command acknowledgment: {msg}")
                if msg.result != 0:
                    raise Exception(f"Takeoff command failed with result: {msg.result}")
            else:
                raise Exception("No acknowledgment received for takeoff command")
            
            # Monitor mission progress
            start_time = time.time()
            last_alt = None
            min_alt = None
            recovery_attempts = 0
            max_recovery_attempts = 3
            last_mode = None
            last_armed = None
            
            self.log("Starting mission monitoring...")
            
            while time.time() - start_time < 300:  # Monitor for up to 5 minutes
                # Get current mode and armed state
                current_mode = self.window.master.flightmode
                current_armed = self.window.master.motors_armed()
                
                # Log mode and armed state changes
                if current_mode != last_mode or current_armed != last_armed:
                    self.log(f"State change: mode={current_mode}, armed={current_armed}")
                    last_mode = current_mode
                    last_armed = current_armed
                
                # Check current altitude and velocity
                pos = self.window.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=0.5)
                vel = self.window.master.recv_match(type='VFR_HUD', blocking=True, timeout=0.5)
                
                if pos:
                    current_alt = pos.alt / 1000.0
                    current_vel = vel.airspeed if vel else 0
                    
                    # Log detailed position and velocity
                    self.log(f"Position: alt={current_alt:.2f}m, vel={current_vel:.2f}m/s")
                    
                    # Track minimum altitude
                    if min_alt is None or current_alt < min_alt:
                        min_alt = current_alt
                        self.log(f"New minimum altitude: {min_alt:.2f}m")
                    
                    # Check for altitude decrease
                    if last_alt is not None:
                        alt_diff = current_alt - last_alt
                        if alt_diff < -0.5:  # If altitude drops more than 0.5m
                            self.log(f"Warning: Altitude decreasing by {abs(alt_diff):.2f}m (from {last_alt:.2f}m to {current_alt:.2f}m)")
                            # If altitude drops too much, try to recover
                            if alt_diff < -5.0 and recovery_attempts < max_recovery_attempts:
                                self.log(f"Attempting to recover altitude (attempt {recovery_attempts + 1}/{max_recovery_attempts})...")
                                
                                # Send takeoff command to regain altitude
                                target_recovery_alt = last_alt + 20.0
                                self.log(f"Sending takeoff command to {target_recovery_alt:.2f}m")
                                self.window.master.mav.command_long_send(
                                    self.window.master.target_system,
                                    self.window.master.target_component,
                                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                    0,  # Confirmation
                                    0,  # Minimum pitch
                                    0,  # Empty
                                    0,  # Empty
                                    0,  # Empty
                                    0,  # Empty
                                    0,  # Empty
                                    target_recovery_alt
                                )
                                
                                # Wait for command acknowledgment
                                msg = self.window.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
                                if msg:
                                    self.log(f"Recovery command acknowledgment: {msg}")
                                
                                recovery_attempts += 1
                    
                    last_alt = current_alt
                
                # Check for status messages
                status_msg = self.window.master.recv_match(type='STATUSTEXT', blocking=True, timeout=0.5)
                if status_msg:
                    self.log(f"Status message: {status_msg.text}")
                    if "Hit ground" in status_msg.text:
                        # Log final state before crash
                        self.log(f"CRASH DETECTED - Final state: mode={current_mode}, armed={current_armed}, alt={current_alt:.2f}m, vel={current_vel:.2f}m/s")
                        raise Exception("Vehicle hit ground during mission")
                
                time.sleep(0.5)  # Check more frequently
            
            self.log("Mission completed successfully")
            return True
            
        except Exception as e:
            self.log(f"Error during mission: {str(e)}")
            return False

    def pause_simulation(self):
        try:
            print("Pause button clicked")  # Debug log
            if not hasattr(self, 'is_paused'):
                self.is_paused = False
            
            print(f"Current is_paused state: {self.is_paused}")  # Debug log
            
            if not self.is_paused:
                self.log("Pausing simulation...")
                print("Attempting to pause simulation")  # Debug log
                # Send pause command by switching to HOLD mode
                if hasattr(self, 'master'):
                    print("Sending HOLD mode command")  # Debug log
                    # Send the command to switch to HOLD mode
                    self.master.mav.command_long_send(
                        self.master.target_system,
                        self.master.target_component,
                        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                        0,  # Confirmation
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # Base mode
                        4,  # Custom mode (HOLD)
                        0,  # Empty
                        0,  # Empty
                        0,  # Empty
                        0,  # Empty
                        0   # Empty
                    )
                    
                    # Wait for command acknowledgment
                    ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=2)
                    if ack:
                        print(f"Command acknowledgment received: {ack}")  # Debug log
                        if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                            self.log("Simulation paused successfully")
                            self.is_paused = True
                            self.pause_button.setText("Resume simulation")
                            self.pause_button.setStyleSheet("background-color: #2ECC40; color: white; font-weight: bold; border-radius: 4px; padding: 6px 18px;")
                        else:
                            self.log(f"Failed to pause simulation: {ack.result}")
                    else:
                        self.log("No command acknowledgment received")
                else:
                    print("No MAVLink connection found")  # Debug log
                    self.log("Cannot pause simulation - no MAVLink connection")
            else:
                self.log("Resuming simulation...")
                print("Attempting to resume simulation")  # Debug log
                # Send resume command by switching back to AUTO mode
                if hasattr(self, 'master'):
                    print("Sending AUTO mode command")  # Debug log
                    # Send the command to switch back to AUTO mode
                    self.master.mav.command_long_send(
                        self.master.target_system,
                        self.master.target_component,
                        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                        0,  # Confirmation
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # Base mode
                        3,  # Custom mode (AUTO)
                        0,  # Empty
                        0,  # Empty
                        0,  # Empty
                        0,  # Empty
                        0   # Empty
                    )
                    
                    # Wait for command acknowledgment
                    ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=2)
                    if ack:
                        print(f"Command acknowledgment received: {ack}")  # Debug log
                        if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                            self.log("Simulation resumed successfully")
                            self.is_paused = False
                            self.pause_button.setText("Pause simulation")
                            self.pause_button.setStyleSheet("background-color: orange; color: black; font-weight: normal; border-radius: 4px; padding: 6px 18px;")
                        else:
                            self.log(f"Failed to resume simulation: {ack.result}")
                    else:
                        self.log("No command acknowledgment received")
                else:
                    print("No MAVLink connection found")  # Debug log
                    self.log("Cannot resume simulation - no MAVLink connection")
            
        except Exception as e:
            print(f"Error in pause_simulation: {str(e)}")  # Debug log
            import traceback
            traceback.print_exc()  # Print full stack trace
            self.log(f"Error pausing/resuming simulation: {str(e)}")
            # Reset button state if there's an error
            self.is_paused = False
            self.pause_button.setText("Pause simulation")
            self.pause_button.setStyleSheet("background-color: orange; color: black; font-weight: normal; border-radius: 4px; padding: 6px 18px;")

if __name__ == "__main__":
    try:
        print("Starting UAV Simulator UI...")
        app = QApplication(sys.argv)
        
        # Check if ArduPilot is installed
        ardupilot_path = os.path.expanduser("~/ardupilot")
        if not os.path.exists(ardupilot_path):
            print(f"Error: ArduPilot not found at {ardupilot_path}")
            print("Please install ArduPilot first.")
            sys.exit(1)
            
        # Check if SITL script exists
        sitl_script = os.path.join(ardupilot_path, "Tools/autotest/sim_vehicle.py")
        if not os.path.exists(sitl_script):
            print(f"Error: SITL script not found at {sitl_script}")
            print("Please ensure ArduPilot is properly installed.")
            sys.exit(1)
            
        print("Creating main window...")
        window = UAVSimulatorUI()
        print("Showing window...")
        window.show()
        print("Starting application event loop...")
        sys.exit(app.exec())
    except Exception as e:
        print(f"Fatal error: {str(e)}")
        import traceback
        traceback.print_exc()
        sys.exit(1)