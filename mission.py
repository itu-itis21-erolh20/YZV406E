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
        fire_lat = current_location.lat
        fire_lon = current_location.lon
        fire_alt = 10
        return fire_lat, fire_lon, fire_alt

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

def main():
    rclpy.init()
    fire_node = FireDetectionNode()

    # ROS düğümünü bir thread içinde çalıştır
    ros_thread = threading.Thread(target=rclpy.spin, args=(fire_node,))
    ros_thread.start()

    takeoff(10)
    gorev_ekle()

    komut.next = 0
    iha.mode = VehicleMode("AUTO")

    while True:
        next_waypoint = komut.next
        print(f"Sıradaki komut {next_waypoint}")

        # Ateş tespit edildiyse waypoint döngüsünü kır
        if fire_node.fire_detected:
            print("Ateş tespit edildi, drone 10 metre irtifada bekliyor.")
            fire_lat, fire_lon, fire_alt = fire_node.fire_coords
            iha.mode = VehicleMode("GUIDED")
            fire_location = LocationGlobalRelative(fire_lat, fire_lon, fire_alt)
            iha.simple_goto(fire_location)

            # 10 metre irtifada ateşin üzerinde bekle
            while True:
                current_location = iha.location.global_relative_frame
                if abs(current_location.alt - fire_alt) < 0.5:
                    print("Drone ateşin üzerinde bekliyor.")
                    time.sleep(1)
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



if __name__ == '__main__':
    main()
