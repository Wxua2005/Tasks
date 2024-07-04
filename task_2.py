import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
import cv2 as cv
import numpy as np

class BallTracker(Node):
    def __init__(self):
        super().__init__('ball_tracker')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.publisher_2 = self.create_publisher(Point, 'coords', 10)
        self.subscription = self.create_subscription(Point, 'coords', self.coords_callback, 10)
        self.cap = cv.VideoCapture(r"WIN_20240704_19_12_21_Pro.mp4")
        self.previous_x = 0
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, image = self.cap.read()

        if not ret:
            self.get_logger().info("No image data available")
            return

        img2 = cv.cvtColor(image, cv.COLOR_BGR2HSV)

        lower_yellow = np.array([16, 102, 0])
        upper_yellow = np.array([35, 255, 255])

        mask = cv.inRange(img2, lower_yellow, upper_yellow)

        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if cv.contourArea(contour) > 500:
                x, y, w, h = cv.boundingRect(contour)
                middle = (int(x + w / 2), int(y + h / 2))
                cv.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv.circle(image, middle, 5, (0, 0, 255), 2, cv.LINE_AA)
                point = Point()
                point.x = float(x)
                point.y = float(y)
                self.publisher_2.publish(point)

                twist = Twist()
    
                if middle[0] - self.previous_x < 0:
                    twist.linear.x = -0.5
                    self.get_logger().info("Moving Left")

                elif middle[0] - self.previous_x > 0:
                    twist.linear.x = 0.5
                    self.get_logger().info("Moving Right")

                else:
                    self.get_logger().info("No movement")
                    
                self.previous_x = middle[0]
                self.publisher_.publish(twist)
                break

        cv.imshow("Image", image)

        if cv.waitKey(1) == ord('q'):
            self.cap.release()
            cv.destroyAllWindows()
            rclpy.shutdown()

    def coords_callback(self, msg):
        self.get_logger().info(f"Ball center: ({msg.x, msg.y})")

def main(args=None):
    rclpy.init(args=args)
    ball_tracker = BallTracker()
    rclpy.spin(ball_tracker)
    ball_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
