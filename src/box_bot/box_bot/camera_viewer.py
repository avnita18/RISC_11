
import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')

        self.bridge = CvBridge()

        # Keep your existing ROS topics.
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Movement settings: start slowly in simulation.
        self.forward_speed = 0.50
        self.curve_speed = 0.012
        self.Kp = 0.003
        self.max_turn = 0.15

        # Black track becomes white in the binary mask
        self.threshold_value = 100

        # Ignore small noise contours.
        self.min_contour_area = 100

        # Look mainly at the part of the image near the robot.
        self.roi_top_ratio = 0.55
        self.roi_bottom_ratio = 0.95

        self.get_logger().info(
            'Contour-based camera line follower started'
        )

    def publish_velocity(self, linear=0.0, angular=0.0):
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.cmd_pub.publish(cmd)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as exc:
            self.get_logger().error(f'Image conversion failed: {exc}')
            self.publish_velocity()
            return

        height, width = frame.shape[:2]
        print("Frame mean brightness:", frame.mean())
        print("Frame timestamp:", msg.header.stamp.sec, msg.header.stamp.nanosec)
        center_x = width / 2.0

        # 1. Convert the image to grayscale.
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 2. Invert thresholding:
        #    black track -> white pixels
        #    white floor -> black pixels
        _, mask = cv2.threshold(
            gray,
            self.threshold_value,
            255,
            cv2.THRESH_BINARY_INV
        )

        # 3. Keep the lower part of the image.
        y_start = int(height * self.roi_top_ratio)
        y_end = int(height * self.roi_bottom_ratio)

        roi = mask[y_start:y_end, :].copy()

        # 4. Find connected white contours in the ROI.
        contours, _ = cv2.findContours(
            roi,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        # Ignore contours that are too small.
        valid_contours = [
            contour for contour in contours
            if cv2.contourArea(contour) >= self.min_contour_area
        ]

        output = frame.copy()
        cv2.rectangle(
            output,
            (0, y_start),
            (width - 1, y_end),
            (0, 0, 255),
            2
        )

        # Draw the image center.
        cv2.line(
            output,
            (int(center_x), 0),
            (int(center_x), height - 1),
            (255, 0, 0),
            2
        )

        if not valid_contours:
            # Do not steer using an old error when the line disappears.
            self.publish_velocity()

            cv2.putText(
                output,
                'LINE LOST - STOPPED',
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

        else:
            # 5. Select the largest contour in the near ROI.
            contour = max(valid_contours, key=cv2.contourArea)
            area = cv2.contourArea(contour)

            moments = cv2.moments(contour)

            if moments['m00'] == 0:
                self.publish_velocity()
            else:
                # Centroid coordinates are relative to the cropped ROI.
                local_x = moments['m10'] / moments['m00']
                local_y = moments['m01'] / moments['m00']

                # Convert the centroid back to full-image coordinates.
                line_x = int(local_x)
                line_y = int(local_y + y_start)

                # 6. Calculate the horizontal error.
                error = line_x - center_x

                # 7. Proportional steering, limited to a safe range.
                angular = max(
                    -self.max_turn,
                    min(self.max_turn, -self.Kp * error)
                )

                # Slow down when the detected line is far from center.
                if abs(error) > width * 0.12:
                    speed = self.curve_speed
                else:
                    speed = self.forward_speed

                self.publish_velocity(speed, angular)

                # Draw the selected contour and its centroid.
                shifted_contour = contour.copy()
                shifted_contour[:, 0, 1] += y_start

                cv2.drawContours(
                    output,
                    [shifted_contour],
                    -1,
                    (0, 255, 255),
                    2
                )

                cv2.circle(
                    output,
                    (line_x, line_y),
                    7,
                    (0, 0, 255),
                    -1
                )

                cv2.putText(
                    output,
                    f'Error: {error:.1f}  Area: {area:.0f}',
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 180, 0),
                    2
                )

                cv2.putText(
                    output,
                    f'Angular: {angular:.3f}  Speed: {speed:.3f}',
                    (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 180, 0),
                    2
                )

        cv2.imshow('Gazebo Camera', output)
        cv2.imshow('Threshold Mask', mask)
        cv2.imshow('Raw Camera Feed', frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_velocity()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

