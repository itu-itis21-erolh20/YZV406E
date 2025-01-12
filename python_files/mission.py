from dronekit import Command, connect, VehicleMode, LocationGlobalRelative
import time
from pymavlink import mavutil
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import math
from geometry_msgs.msg import PoseStamped
import subprocess
import os

# Connect to the drone
iha = connect("127.0.0.1:14550", wait_ready=True)

class FireDetectionNode(Node):
    def __init__(self):
        super().__init__('fire_detection_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.fire_detected = False
        self.fire_coords = None
        self.image_center = None  # Image center of the camera
        self.fire_center = None  # Fire center in the image
        self.x_gazebo = None
        self.y_gazebo = None

        self.fire_pose_publisher = self.create_publisher(PoseStamped, '/fire_position', 10)
        self.MIN_FIRE_AREA = 400  # Minimum fire area in pixels

    def listener_callback(self, msg):
        try:
            # Convert the image from the camera to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("Camera Feed", cv_image)

            # Detect fire using color filtering
            fire_mask = self.detect_fire(cv_image)
            height, width, _ = cv_image.shape
            self.image_center = (width // 2, height // 2)

            # Find the center of the fire in the image
            contours, _ = cv2.findContours(fire_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest_contour) > self.MIN_FIRE_AREA:  # Area threshold check
                    M = cv2.moments(largest_contour)
                    if M["m00"] > 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        self.fire_center = (cX, cY)
                        cv2.circle(cv_image, self.fire_center, 5, (0, 255, 0), -1)

                        # If fire is detected
                        if not self.fire_detected:
                            self.fire_detected = True
                            self.get_logger().info("Fire detected!")
                            self.fire_coords = self.get_fire_coordinates()
                            self.get_logger().info(f"Fire GPS coordinates: {self.fire_coords}")

                            pose_msg = PoseStamped()
                            pose_msg.header.frame_id = "map"  # Reference frame for RViz
                            pose_msg.header.stamp = self.get_clock().now().to_msg()
                            pose_msg.pose.position.x = self.fire_coords[0]
                            pose_msg.pose.position.y = self.fire_coords[1]
                            pose_msg.pose.position.z = self.fire_coords[2]
                            self.fire_pose_publisher.publish(pose_msg)

                    else:
                        self.fire_detected = False
                else:
                    self.fire_detected = False

                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error while processing the image: {str(e)}")

    def detect_fire(self, image):
        """Detect fire based on color range (filters red tones)."""
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 120, 120])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 120, 120])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        return cv2.bitwise_or(mask1, mask2)

    def get_fire_coordinates(self):
        """Retrieve GPS information of the drone and detect the fire."""
        current_location = iha.location.global_relative_frame
        lat_ref = -35.363262
        lon_ref = 149.165237
        fire_lat = current_location.lat
        fire_lon = current_location.lon
        fire_alt = 10
        print("Coordinates normal", fire_lat, fire_lon)
        self.x_gazebo, self.y_gazebo = self.gps_to_gazebo(fire_lat, fire_lon, lat_ref, lon_ref, -15, 0)
        print("Gazebo", self.x_gazebo, self.y_gazebo)
        return fire_lat, fire_lon, fire_alt

    def gps_to_gazebo(self, lat, lon, lat_ref, lon_ref, x_ref, y_ref):
        # Constants
        meters_per_deg_lat = 111320  # Approximate meters per degree latitude
        meters_per_deg_lon = 111320 * math.cos(math.radians(lat_ref))  # Adjust for longitude

        # Calculate offsets in meters
        delta_lat = lat - lat_ref
        delta_lon = lon - lon_ref
        y_offset = delta_lat * meters_per_deg_lat
        x_offset = delta_lon * meters_per_deg_lon

        # Translate to Gazebo coordinates
        x_gazebo = x_ref + x_offset
        y_gazebo = y_ref + y_offset
        x_gazebo = x_gazebo + 21
        y_gazebo = y_gazebo - 3
        return x_gazebo, y_gazebo

def takeoff(altitude):
    while not iha.is_armable:
        print("Drone is not armable.")
        time.sleep(1)

    print("Drone is armable.")
    iha.mode = VehicleMode("GUIDED")
    iha.armed = True

    while not iha.armed:
        print("Arming the drone...")
        time.sleep(0.5)

    print("Drone armed.")
    iha.simple_takeoff(altitude)

    while iha.location.global_relative_frame.alt < altitude * 0.9:
        print("Drone ascending to target altitude.")
        time.sleep(1)

def add_mission():
    global komut
    komut = iha.commands
    komut.clear()
    time.sleep(1)

    home_location = iha.location.global_relative_frame
    home_lat = home_location.lat
    home_lon = home_location.lon
    takeoff_alt = 10

    waypoint_1_lat = home_lat + 0.00045
    waypoint_1_lon = home_lon
    waypoint_2_lat = waypoint_1_lat
    waypoint_2_lon = home_lon + 0.00027
    waypoint_3_lat = waypoint_2_lat - 0.00045
    waypoint_3_lon = waypoint_2_lon

    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0,
                      home_lat, home_lon, takeoff_alt))

    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                      waypoint_1_lat, waypoint_1_lon, takeoff_alt))

    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                      waypoint_2_lat, waypoint_2_lon, takeoff_alt))

    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                      waypoint_3_lat, waypoint_3_lon, takeoff_alt))

    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0,
                      0, 0, 0))
    komut.upload()
    print("Mission uploaded.")

# Function to start the simulation
def start_simulation():
    try:
        # Source ROS 2 environment
        ros_setup_path = "/opt/ros/foxy/setup.bash"  # Path to ROS 2 setup
        workspace_setup_path = "/home/erdem/ros2_ws/install/setup.bash"  # Path to workspace setup

        # Run the launch command
        process = subprocess.Popen(
            f"source {ros_setup_path} && source {workspace_setup_path} && ros2 launch turtlebot3_gazebo project.py x_pose:=-7.216780 y_pose:=-9.611480",
            shell=True,
            executable="/bin/bash",
        )

        return process  # Return the subprocess

    except Exception as e:
        print(f"Error occurred while starting the simulation: {str(e)}")
        return None

def publish_goal_pose(x_gazebo, y_gazebo):
    # Calculate Gazebo x and y coordinates using gps_to_gazebo
    # Create the ros2 topic pub command
    command = f"ros2 topic pub /goal_pose geometry_msgs/PoseStamped \"{{header: {{frame_id: 'map'}}, pose: {{position: {{x: {x_gazebo}, y: {y_gazebo}, z: 0.0}}, orientation: {{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}}}\""

    try:
        # Run the command using subprocess
        process = subprocess.Popen(command, shell=True, executable="/bin/bash")
        process.wait()  # Wait for the command to complete
        print(f"/goal_pose position {x_gazebo}, {y_gazebo} sent.")
    except Exception as e:
        print(f"Error occurred while running the command: {str(e)}")

def publish_initial_pose():
    command = """
    ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped "
    {
      header: {
        frame_id: 'map'
      },
      pose: {
        pose: {
          position: {
            x: -7.216780,
            y: -9.611480,
            z: 0.0
          },
          orientation: {
            x: 0.0,
            y: 0.0,
            z: 0.0023734764283772684,
            w: 0.9999971833008551
          }
        },
        covariance: [
          0.20894151592677424, -0.004840539988737848, 0.0, 0.0, 0.0, 0.0,
          -0.004840539988737848, 0.208603739297144, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0574483219202385
        ]
      }
    }"
    """
    try:
        process = subprocess.Popen(command, shell=True, executable="/bin/bash")
        process.wait()
        print("Initial pose successfully published.")
    except Exception as e:
        print(f"Error occurred while publishing initial pose: {str(e)}")

def main():
    # Start the simulation
    sim_process = start_simulation()
    if not sim_process:
        print("Simulation could not be started.")
        return
    time.sleep(5)  # Wait for Gazebo to load

    # Publish the initial pose
    publish_initial_pose()

    rclpy.init()
    fire_node = FireDetectionNode()

    # Run the ROS node in a thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(fire_node,))
    ros_thread.start()

    takeoff(10)
    add_mission()

    komut.next = 0
    iha.mode = VehicleMode("AUTO")

    # Set speed in m/s
    iha.airspeed = 5  # Forward speed
    iha.groundspeed = 5  # Ground speed

    while True:
        next_waypoint = komut.next
        print(f"Next command: {next_waypoint}")

        # Break the waypoint loop if fire is detected
        if fire_node.fire_detected:
            print("Fire detected, drone is hovering at 10 meters altitude.")
            fire_lat, fire_lon, fire_alt = fire_node.fire_coords

            # Parameters
            delta_x = 3  # meters
            latitude = fire_lat  # degrees
            R = 6371000  # Earth's radius in meters

            # Calculate
            delta_longitude = (delta_x / (R * math.cos(math.radians(latitude)))) * (180 / math.pi)
            iha.mode = VehicleMode("GUIDED")
            fire_location = LocationGlobalRelative(fire_lat + delta_longitude, fire_lon, fire_alt)
            iha.simple_goto(fire_location)

            # Hover over the fire at 10 meters altitude
            while True:
                current_location = iha.location.global_relative_frame
                if abs(current_location.alt - fire_alt) < 0.5:
                    print("Drone is hovering over the fire.")
                    time.sleep(1)
                    publish_goal_pose(fire_node.x_gazebo, fire_node.y_gazebo)

                else:
                    break

            break

        if next_waypoint == 5:
            print("Mission completed.")
            break

        time.sleep(1)

    rclpy.shutdown()
    ros_thread.join()
    cv2.destroyAllWindows()

    # Terminate the simulation process
    sim_process.terminate()
    print("Simulation terminated.")

if __name__ == '__main__':
    main()
