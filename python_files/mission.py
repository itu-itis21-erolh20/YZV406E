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
import math
import subprocess
import threading
import os

# Drone bağlantısı
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
        self.image_center = None  # Kameranın görüntü merkezi
        self.fire_center = None  # Ateşin görüntüdeki merkezi
        self.x_gazebo = None
        self.y_gazebo = None

        self.fire_pose_publisher = self.create_publisher(PoseStamped, '/fire_position', 10)
        self.MIN_FIRE_AREA = 400  # Minimum ateş alanı (piksel cinsinden)


    def listener_callback(self, msg):
        try:
            # Kameradan gelen görüntüyü OpenCV formatına çevir
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Ateşi tespit etmek için renk filtreleme
            fire_mask = self.detect_fire(cv_image)
            height, width, _ = cv_image.shape
            self.image_center = (width // 2, height // 2)
            # Ateşin görüntüdeki merkezi
            contours, _ = cv2.findContours(fire_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest_contour) > self.MIN_FIRE_AREA:  # Alan eşik kontrolü
                    M = cv2.moments(largest_contour)
                    if M["m00"] > 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        self.fire_center = (cX, cY)
                        cv2.circle(cv_image, self.fire_center, 5, (0, 255, 0), -1)

                        # Eğer ateş tespit edilmişse
                        if not self.fire_detected:
                            self.fire_detected = True
                            self.get_logger().info("Ateş tespit edildi!")
                            self.fire_coords = self.get_fire_coordinates()
                            self.get_logger().info(f"Ateşin GPS koordinatları: {self.fire_coords}")

                            pose_msg = PoseStamped()
                            pose_msg.header.frame_id = "map"  # RViz için referans çerçeve
                            pose_msg.header.stamp = self.get_clock().now().to_msg()
                            pose_msg.pose.position.x = self.fire_coords[0]
                            pose_msg.pose.position.y = self.fire_coords[1]
                            pose_msg.pose.position.z = self.fire_coords[2]
                            self.fire_pose_publisher.publish(pose_msg)

                    else:
                        self.fire_detected = False
                else:
                    self.fire_detected = False
                # Görüntüyü ekranda göster
                cv2.imshow("Camera Feed", cv_image)
                cv2.imshow("Fire Mask", fire_mask)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Görüntü işlenirken hata oluştu: {str(e)}")

    def detect_fire(self, image):
        """Renk aralığına göre ateş tespiti (kırmızı tonlarını filtreler)."""
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 120, 120])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 120, 120])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        return cv2.bitwise_or(mask1, mask2)

    def get_fire_coordinates(self):
        """Drone'un GPS bilgilerini al ve ateşi tespit et."""
        current_location = iha.location.global_relative_frame
        lat_ref = -35.363262
        lon_ref = 149.165237
        fire_lat = current_location.lat
        fire_lon = current_location.lon
        fire_alt = 10
        print("kordinat normal",  fire_lat, fire_lon)
        self.x_gazebo, self.y_gazebo = self.gps_to_gazebo(fire_lat, fire_lon, lat_ref, lon_ref, -15, 0)
        print("gazebo", self.x_gazebo, self.y_gazebo)
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
    

def takeoff(irtifa):
    while not iha.is_armable:
        print("İHA arm edilebilir durumda değil.")
        time.sleep(1)

    print("İHA arm edilebilir.")
    iha.mode = VehicleMode("GUIDED")
    iha.armed = True

    while not iha.armed:
        print("İHA arm ediliyor...")
        time.sleep(0.5)

    print("İHA arm edildi.")
    iha.simple_takeoff(irtifa)

    while iha.location.global_relative_frame.alt < irtifa * 0.9:
        print("İHA hedefe yükseliyor.")
        time.sleep(1)


def gorev_ekle():
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
    print("Görev yüklendi.")

# Simülasyon Başlatma Fonksiyonu
def start_simulation():
    try:
        # ROS 2 ortamını kaynakla
        ros_setup_path = "/opt/ros/foxy/setup.bash"  # ROS 2'nin kurulu olduğu yol
        workspace_setup_path = "/home/erdem/ros2_ws/install/setup.bash"  # Çalışma alanınızın setup dosyası
        
        # Launch komutunu çalıştır
        process = subprocess.Popen(
            f"source {ros_setup_path} && source {workspace_setup_path} && ros2 launch turtlebot3_gazebo project.py",
            shell=True,
            executable="/bin/bash",
        )
        
        return process  # Alt süreci döndür

    except Exception as e:
        print(f"Simülasyon başlatılırken bir hata oluştu: {str(e)}")
        return None
    
def publish_goal_pose(x_gazebo, y_gazebo):
    # gps_to_gazebo fonksiyonu çağrılarak Gazebo x ve y hesaplanır
    # ros2 topic pub komutunu oluştur
    command = f"ros2 topic pub /goal_pose geometry_msgs/PoseStamped \"{{header: {{frame_id: 'map'}}, pose: {{position: {{x: {x_gazebo}, y: {y_gazebo}, z: 0.0}}, orientation: {{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}}}\""

    try:
        # Subprocess ile komutu çalıştır
        process = subprocess.Popen(command, shell=True, executable="/bin/bash")
        process.wait()  # Komutun tamamlanmasını bekle
        print(f"/goal_pose konumuna {x_gazebo}, {y_gazebo} gönderildi.")
    except Exception as e:
        print(f"Komut çalıştırılırken bir hata oluştu: {str(e)}")

def main():
        # Simülasyonu başlat
    sim_process = start_simulation()
    if not sim_process:
        print("Simülasyon başlatılamadı.")
        return
    rclpy.init()
    fire_node = FireDetectionNode()

    # ROS düğümünü bir thread içinde çalıştır
    ros_thread = threading.Thread(target=rclpy.spin, args=(fire_node,))
    ros_thread.start()

    takeoff(10)
    gorev_ekle()

    komut.next = 0
    iha.mode = VehicleMode("AUTO")
    # Hız ayarını belirle (m/s cinsinden)
    iha.airspeed = 5  # İleri hız
    iha.groundspeed = 5  # Yere göre hız
    while True:
        next_waypoint = komut.next
        print(f"Sıradaki komut {next_waypoint}")

        # Ateş tespit edildiyse waypoint döngüsünü kır
        if fire_node.fire_detected:
            print("Ateş tespit edildi, drone 10 metre irtifada bekliyor.")
            fire_lat, fire_lon, fire_alt = fire_node.fire_coords
            # Parametreler
            delta_x = 3  # metre
            latitude = fire_lat  # derece
            R = 6371000  # Dünya yarıçapı (metre)
            # Hesaplama
            delta_longitude = (delta_x / (R * math.cos(math.radians(latitude)))) * (180 / math.pi)
            iha.mode = VehicleMode("GUIDED")
            fire_location = LocationGlobalRelative(fire_lat+delta_longitude ,  fire_lon , fire_alt)
            iha.simple_goto(fire_location)

            # 10 metre irtifada ateşin üzerinde bekle
            while True:
                current_location = iha.location.global_relative_frame
                if abs(current_location.alt - fire_alt) < 0.5:
                    print("Drone ateşin üzerinde bekliyor.")
                    time.sleep(1)
                    publish_goal_pose(fire_node.x_gazebo, fire_node.y_gazebo)

                else:
                    break

            break

        if next_waypoint == 5:
            print("Görev tamamlandı.")
            break
                

        time.sleep(1)

    rclpy.shutdown()
    ros_thread.join()
    cv2.destroyAllWindows()

    # Simülasyon sürecini sonlandır
    sim_process.terminate()
    print("Simülasyon sonlandırıldı.")


if __name__ == '__main__':
    main()
