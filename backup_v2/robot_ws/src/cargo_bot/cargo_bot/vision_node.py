"""Vision node (USB camera): line following + ArUco station markers.

  publishes  line/error                std_msgs/Float32   -1 (line far left) .. +1 (far right), NaN = no line
             station_marker            std_msgs/Int32     id of the biggest ArUco marker in view, -1 = none
             cmd_vel                   geometry_msgs/Twist  ONLY while line following is enabled
             vision/debug/compressed   sensor_msgs/CompressedImage  annotated JPEG for RViz / rqt
  service    line_follow/enable        std_srvs/SetBool
"""
import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, Int32
from std_srvs.srv import SetBool


def make_aruco_detector(dict_name):
    """Returns detect(gray) -> (corners, ids), working with both old and new OpenCV APIs; None if no aruco."""
    aruco = getattr(cv2, "aruco", None)
    if aruco is None:
        return None
    dictionary = aruco.getPredefinedDictionary(getattr(aruco, dict_name))
    if hasattr(aruco, "ArucoDetector"):                       # OpenCV >= 4.7
        det = aruco.ArucoDetector(dictionary, aruco.DetectorParameters())
        return lambda gray: det.detectMarkers(gray)[:2]
    params = aruco.DetectorParameters_create()                # OpenCV 4.6 (Ubuntu 24.04)
    return lambda gray: aruco.detectMarkers(gray, dictionary, parameters=params)[:2]


class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")
        self.declare_parameters("", [
            ("device", 0), ("width", 640), ("height", 480), ("rate", 15.0),
            ("dark_line", True),          # True = black tape on a light floor
            ("roi_top", 0.55),            # only look at the image below this fraction of its height
            ("min_area", 600),            # px, smaller blobs are noise
            ("follow_speed", 0.12),       # m/s while following
            ("k_turn", 1.5),              # rad/s per unit of line error
            ("lost_timeout", 0.5),        # s without a line -> stop
            ("aruco_dict", "DICT_4X4_50"),
            ("debug_image", True),
        ])
        p = lambda name: self.get_parameter(name).value
        self.dark, self.roi_top, self.min_area = p("dark_line"), p("roi_top"), p("min_area")
        self.speed, self.k_turn, self.lost_timeout = p("follow_speed"), p("k_turn"), p("lost_timeout")
        self.debug = p("debug_image")

        self.cap = cv2.VideoCapture(p("device"), cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, p("width"))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, p("height"))
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            self.get_logger().error(f"cannot open camera /dev/video{p('device')}")
        self.detect_markers = make_aruco_detector(p("aruco_dict"))
        if self.detect_markers is None:
            self.get_logger().warn("OpenCV has no aruco module - station markers disabled")

        self.following = False
        self.last_seen = self.get_clock().now()
        self.frame_count = 0
        self.err_pub = self.create_publisher(Float32, "line/error", 10)
        self.marker_pub = self.create_publisher(Int32, "station_marker", 10)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.dbg_pub = self.create_publisher(CompressedImage, "vision/debug/compressed", 2)
        self.create_service(SetBool, "line_follow/enable", self.on_enable)
        self.create_timer(1.0 / p("rate"), self.step)

    def on_enable(self, request, response):
        self.following = request.data
        self.last_seen = self.get_clock().now()
        if not self.following:
            self.cmd_pub.publish(Twist())
        response.success = True
        response.message = "line following " + ("ON" if self.following else "OFF")
        self.get_logger().info(response.message)
        return response

    def find_line(self, gray):
        """Returns (error -1..1 or None, mask, contour, roi_y0)."""
        h, w = gray.shape
        y0 = int(h * self.roi_top)
        roi = cv2.GaussianBlur(gray[y0:, :], (7, 7), 0)
        mode = cv2.THRESH_BINARY_INV if self.dark else cv2.THRESH_BINARY
        _, mask = cv2.threshold(roi, 0, 255, mode + cv2.THRESH_OTSU)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, mask, None, y0
        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)
        if area < self.min_area or area > 0.6 * mask.size:   # too small = noise, too big = no real line
            return None, mask, None, y0
        m = cv2.moments(c)
        cx = m["m10"] / m["m00"]
        return (cx - w / 2.0) / (w / 2.0), mask, c, y0

    def step(self):
        ok, frame = self.cap.read()
        if not ok:
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        now = self.get_clock().now()

        error, _, contour, y0 = self.find_line(gray)
        self.err_pub.publish(Float32(data=float("nan") if error is None else float(error)))

        marker_id, corners = -1, None
        if self.detect_markers is not None:
            corners, ids = self.detect_markers(gray)
            if ids is not None and len(ids):
                sizes = [cv2.contourArea(c.reshape(-1, 2).astype(np.float32)) for c in corners]
                marker_id = int(ids[int(np.argmax(sizes))][0])
        self.marker_pub.publish(Int32(data=marker_id))

        if self.following:
            cmd = Twist()
            if error is not None:
                self.last_seen = now
                cmd.linear.x = self.speed * (1.0 - 0.5 * min(abs(error), 1.0))
                cmd.angular.z = -self.k_turn * error
            elif (now - self.last_seen).nanoseconds * 1e-9 < self.lost_timeout:
                return                                   # brief dropout: keep last command
            self.cmd_pub.publish(cmd)                    # line lost -> zero Twist = stop

        self.frame_count += 1
        if self.debug and self.frame_count % 3 == 0:
            self.publish_debug(frame, error, contour, y0, corners, marker_id)

    def publish_debug(self, frame, error, contour, y0, corners, marker_id):
        img = frame.copy()
        h, w = img.shape[:2]
        cv2.line(img, (0, y0), (w, y0), (255, 255, 0), 1)
        cv2.line(img, (w // 2, y0), (w // 2, h), (255, 0, 0), 1)
        if contour is not None:
            cv2.drawContours(img, [contour + np.array([[0, y0]])], -1, (0, 255, 0), 2)
        if corners is not None and len(corners):
            for c in corners:
                cv2.polylines(img, [c.reshape(-1, 2).astype(np.int32)], True, (0, 0, 255), 2)
        txt = f"err={'none' if error is None else f'{error:+.2f}'} marker={marker_id} follow={self.following}"
        cv2.putText(img, txt, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        msg.format = "jpeg"
        msg.data = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 60])[1].tobytes()
        self.dbg_pub.publish(msg)

    def shutdown(self):
        if self.following:
            self.cmd_pub.publish(Twist())
        self.cap.release()


def main():
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
