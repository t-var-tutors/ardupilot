import os
import logging
import time
from datetime import datetime
from pymavlink import mavutil, mavwp

class UAV:
    def __init__(self):
        # Initialize connection to the SITL simulator
        # Set up logging
        log_dir = "logs"
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        log_file = os.path.join(log_dir, f"uav_connection_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log")
        logging.basicConfig(
            level=logging.DEBUG,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(log_file),
                logging.StreamHandler()
            ]
        )
        self.logger = logging.getLogger(__name__)
        
        # Default position values (in degrees)
        self.default_lat = 30.234242  # Start position latitude
        self.default_lon = 50.12321   # Start position longitude
        self.default_alt = 0.0        # Start position altitude
        
        try:
            # Check if simulator is running
            import subprocess
            result = subprocess.run(['pgrep', 'arducopter'], capture_output=True, text=True)
            if not result.stdout.strip():
                self.logger.error("SITL simulator is not running. Please start it first.")
                raise ConnectionError("SITL simulator is not running. Please start it first.")
            
            # Connect to SITL simulator on default port
            self.master = mavutil.mavlink_connection('udpin:127.0.0.1:14550')
            self.master.wait_heartbeat()
            self.logger.info("Connected to SITL simulator")
            self.logger.info("Heartbeat from system (system %u component %u)" % 
                  (self.master.target_system, self.master.target_component))
            
            # Wait for system to be ready
            self.master.wait_gps_fix()
            self.logger.info("GPS fix obtained")
            
            # Wait for AHRS to be ready
            while not self.master.motors_armed():
                self.master.wait_heartbeat()
                if self.master.messages.get('HEARTBEAT', None):
                    if self.master.messages['HEARTBEAT'].autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
                        break
            self.logger.info("AHRS ready")
            
            # Don't try to configure mission parameters as we don't have permission
            self.logger.info("Skipping mission parameter configuration (permission denied)")
            
            # Store waypoints for later use
            self.waypoints = None
            
        except Exception as e:
            self.logger.error(f"Failed to connect to SITL simulator: {str(e)}")
            raise ConnectionError(f"Failed to connect to SITL simulator: {str(e)}")

    def start_mission(self, waypoints=None):
        # Start the mission with the given waypoints
        try:
            # Use default waypoints if none provided
            if waypoints is None:
                waypoints = [
                    (self.default_lat, self.default_lon, self.default_alt),     # Start position
                    (self.default_lat + 0.0045, self.default_lon, 50.0),        # Move north
                    (self.default_lat, self.default_lon, 50.0),                 # Return to start
                    (self.default_lat, self.default_lon, self.default_alt)      # Land
                ]
                self.logger.info(f"Using default waypoints starting at lat={self.default_lat}, lon={self.default_lon}, alt={self.default_alt}")
            
            # Upload waypoints
            self.upload_waypoints(waypoints)
            
            # Set home position
            self.set_home_position()
            
            # Take off
            self.takeoff()
            
            # Start mission
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_MISSION_START,
                0,  # Confirmation
                0,  # Param 1
                0,  # Param 2
                0,  # Param 3
                0,  # Param 4
                0,  # Param 5
                0,  # Param 6
                0   # Param 7
            )
            
            self.logger.info("Mission started")
            
        except Exception as e:
            raise ConnectionError(f"Failed to start mission: {str(e)}")

    def wait_for_gps(self, timeout=30):
        # Wait for GPS to be ready with a fix
        self.logger.info("Waiting for GPS to be ready...")
        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = self.master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=0.1)
            if msg:
                fix_type = msg.fix_type
                satellites = msg.satellites_visible
                self.logger.info(f"GPS status: fix={fix_type}, satellites={satellites}")
                if fix_type >= 3 and satellites >= 5:  # 3D fix with enough satellites
                    self.logger.info("GPS is ready with 3D fix")
                    return True
            time.sleep(0.1)
        raise ConnectionError("Timeout waiting for GPS to be ready")

    def set_home_position(self):
        # Set the home position
        try:
            # Wait for GPS to be ready first
            self.wait_for_gps()
            
            # Convert default position to integer coordinates
            lat_int = int(self.default_lat * 1e7)
            lon_int = int(self.default_lon * 1e7)
            alt_int = int(self.default_alt * 1000)
            
            self.logger.info(f"Setting home position - Raw values: lat_int={lat_int}, lon_int={lon_int}, alt_int={alt_int}")
            self.logger.info(f"Setting home position - Converted values: lat={self.default_lat}, lon={self.default_lon}, alt={self.default_alt}")
            
            # Set home position using integer coordinates
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                0,  # Confirmation
                1,  # Param 1: 1 to use current location
                0,  # Param 2
                0,  # Param 3
                0,  # Param 4
                lat_int,  # Param 5: Latitude (integer)
                lon_int,  # Param 6: Longitude (integer)
                alt_int   # Param 7: Altitude (integer)
            )
            
            # Wait for command acknowledgment
            start_time = time.time()
            while time.time() - start_time < 5.0:  # 5 second timeout
                ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=0.1)
                if ack is not None:
                    if ack.command == mavutil.mavlink.MAV_CMD_DO_SET_HOME:
                        if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                            self.logger.info(f"Home position set to: lat={self.default_lat}, lon={self.default_lon}, alt={self.default_alt}")
                            return
                        else:
                            raise ConnectionError(f"Failed to set home position: {ack.result}")
            raise ConnectionError("Timeout waiting for home position acknowledgment")
                
        except Exception as e:
            raise ConnectionError(f"Failed to set home position: {str(e)}")

    def takeoff(self, target_altitude=10):
        # Take off to the specified altitude
        try:
            # Wait for GPS to be ready first
            self.wait_for_gps()
            
            # Convert default position to integer coordinates
            lat_int = int(self.default_lat * 1e7)
            lon_int = int(self.default_lon * 1e7)
            alt_int = int(self.default_alt * 1000)
            
            self.logger.info(f"Takeoff position - Raw values: lat_int={lat_int}, lon_int={lon_int}, alt_int={alt_int}")
            self.logger.info(f"Takeoff position - Converted values: lat={self.default_lat}, lon={self.default_lon}, alt={self.default_alt}")
            
            # Send takeoff command
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,  # Confirmation
                0,  # Param 1: Minimum pitch
                0,  # Param 2: Empty
                0,  # Param 3: Empty
                0,  # Param 4: Yaw angle
                lat_int,  # Param 5: Latitude
                lon_int,  # Param 6: Longitude
                alt_int + int(target_altitude * 1000)  # Param 7: Altitude (in mm)
            )
            
            # Wait for takeoff to complete
            self.logger.info(f"Taking off to {target_altitude}m")
            start_time = time.time()
            while time.time() - start_time < 30:  # 30 second timeout
                # Check if we've reached target altitude
                msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=0.1)
                if msg:
                    current_alt = float(msg.alt) / 1000.0
                    self.logger.debug(f"Current altitude: {current_alt}m")
                    if current_alt >= target_altitude * 0.95:  # Allow 5% margin
                        self.logger.info(f"Reached target altitude of {target_altitude}m")
                        return
                time.sleep(0.1)
            raise ConnectionError("Timeout waiting for takeoff to complete")
        except Exception as e:
            raise ConnectionError(f"Takeoff failed: {str(e)}")

    def upload_waypoints(self, waypoints):
        # Upload waypoints to the vehicle
        try:
            # Clear existing waypoints
            self.logger.info("Clearing existing waypoints...")
            self.master.waypoint_clear_all_send()
            self.master.waypoint_count_send(0)
            
            # Wait for acknowledgment
            start_time = time.time()
            while time.time() - start_time < 5.0:
                ack = self.master.recv_match(type='MISSION_ACK', blocking=True, timeout=0.1)
                if ack is not None:
                    if ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                        break
                    else:
                        raise ConnectionError(f"Failed to clear waypoints: {ack.type}")
            
            # Get current position for waypoint 0 (home)
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5.0)
            if msg is None:
                raise ConnectionError("Failed to get current position for waypoint 0")
            
            # Create home waypoint (waypoint 0)
            home_waypoint = mavutil.mavlink.MAVLink_mission_item_message(
                self.master.target_system,
                self.master.target_component,
                0,  # Sequence
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,  # Current
                0,  # Autocontinue
                0,  # Param 1
                0,  # Param 2
                0,  # Param 3
                0,  # Param 4
                0,  # Param 5
                0,  # Param 6
                float(msg.alt) / 1000.0  # Param 7 (altitude in meters)
            )
            
            # Add home waypoint to the beginning of the list
            waypoints.insert(0, home_waypoint)
            
            # Send waypoint count
            self.master.waypoint_count_send(len(waypoints))
            self.logger.info(f"Sending waypoint count: {len(waypoints)}")
            
            # Wait for waypoint requests and send each waypoint
            for i, waypoint in enumerate(waypoints):
                self.logger.info(f"Waiting for waypoint request {i}...")
                msg = self.master.recv_match(type='WAYPOINT_REQUEST', blocking=True, timeout=5.0)
                if msg is None:
                    raise ConnectionError(f"Timeout waiting for waypoint request {i}")
                
                # Create waypoint message
                self.master.mav.send(waypoint)
                self.logger.info(f"Sending waypoint {i}...")
            
            # Wait for final acknowledgment
            start_time = time.time()
            while time.time() - start_time < 5.0:
                ack = self.master.recv_match(type='MISSION_ACK', blocking=True, timeout=0.1)
                if ack is not None:
                    if ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                        self.logger.info("Waypoints uploaded successfully.")
                        return
                    else:
                        raise ConnectionError(f"Failed to upload waypoints: {ack.type}")
            
            raise ConnectionError("Timeout waiting for waypoint upload acknowledgment")
            
        except Exception as e:
            raise ConnectionError(f"Failed to upload waypoints: {str(e)}") 