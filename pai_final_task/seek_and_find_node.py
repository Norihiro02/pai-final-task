#!/usr/bin/env python3
"""Vision-guided "scavenger hunt" controller for TurtleBot3 in Gazebo.

The robot has no map and no pre-programmed coordinates for the targets.
Every decision (where to look, when to turn, when to stop) is driven by
open-vocabulary object detection (YOLO-World, pretrained, zero-shot) run
on the live camera feed. The robot visits three targets in a fixed order,
purely by seeing them.
"""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge
from ultralytics import YOLO

# Order matters: this is the "treasure hunt" sequence the robot must follow.
# Picked empirically (see README "Prompt tuning"): plain word choices like
# "box"/"cylinder" score far lower against YOLO-World's pretrained weights
# than everyday object words like "ball"/"pillar" for the same geometry.
MISSION_TARGETS = ['red ball', 'green pillar', 'blue ball']

ARRIVAL_AREA_RATIO = 0.12   # bbox area / image area considered "arrived"
FORWARD_SPEED = 0.18        # m/s cruise speed while approaching
ANGULAR_GAIN = 1.4          # rad/s per unit of normalized horizontal error
SEARCH_SPIN_SPEED = 0.5     # rad/s while rotating to search
SEARCH_ROTATE_DURATION = 6.0   # seconds of spin before trying a forward nudge
SEARCH_ADVANCE_DURATION = 1.5  # seconds of forward nudge between spins
CONF_THRESHOLD = 0.10
PROCESS_EVERY_N_FRAMES = 2  # throttle inference to roughly half the camera rate


class SeekAndFindNode(Node):

    def __init__(self):
        super().__init__('seek_and_find_node')

        self.bridge = CvBridge()
        self.get_logger().info('Loading YOLO-World (pretrained, zero-shot)...')
        self.model = YOLO('yolov8s-world.pt')
        self.model.set_classes(MISSION_TARGETS)
        self.get_logger().info(f'Mission order: {MISSION_TARGETS}')

        self.target_index = 0
        self.frame_count = 0
        self.mission_done = False

        self.search_phase = 'rotate'
        self.search_phase_start = time.monotonic()

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

    @property
    def current_target(self):
        if self.target_index >= len(MISSION_TARGETS):
            return None
        return MISSION_TARGETS[self.target_index]

    def publish_cmd(self, linear_x: float, angular_z: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def reset_search_phase(self):
        self.search_phase = 'rotate'
        self.search_phase_start = time.monotonic()

    def do_search_step(self):
        """Rotate to scan the arena; periodically nudge forward to reveal
        new viewpoints instead of spinning on a single spot forever."""
        elapsed = time.monotonic() - self.search_phase_start
        if self.search_phase == 'rotate':
            self.publish_cmd(0.0, SEARCH_SPIN_SPEED)
            if elapsed > SEARCH_ROTATE_DURATION:
                self.search_phase = 'advance'
                self.search_phase_start = time.monotonic()
        else:
            self.publish_cmd(FORWARD_SPEED, 0.0)
            if elapsed > SEARCH_ADVANCE_DURATION:
                self.search_phase = 'rotate'
                self.search_phase_start = time.monotonic()

    def image_callback(self, msg: Image):
        if self.current_target is None:
            if not self.mission_done:
                self.mission_done = True
                self.publish_cmd(0.0, 0.0)
                self.get_logger().info('All targets found. Mission complete!')
            return

        self.frame_count += 1
        if self.frame_count % PROCESS_EVERY_N_FRAMES != 0:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]

        results = self.model.predict(frame, verbose=False, conf=CONF_THRESHOLD)

        best_box = None
        best_conf = 0.0
        for box in results[0].boxes:
            cls_name = self.model.names[int(box.cls[0])]
            conf = float(box.conf[0])
            if cls_name == self.current_target and conf > best_conf:
                best_conf = conf
                best_box = box.xyxy[0].tolist()

        if best_box is None:
            self.do_search_step()
            return

        x1, y1, x2, y2 = best_box
        cx = (x1 + x2) / 2.0
        area_ratio = (x2 - x1) * (y2 - y1) / (w * h)
        err = (cx - w / 2.0) / (w / 2.0)  # -1 (left) .. +1 (right)

        if area_ratio >= ARRIVAL_AREA_RATIO:
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info(
                f'Arrived at "{self.current_target}" '
                f'(conf={best_conf:.2f}, area_ratio={area_ratio:.3f})')
            self.target_index += 1
            self.reset_search_phase()
            if self.current_target is not None:
                self.get_logger().info(f'Next target: "{self.current_target}"')
            return

        angular = -ANGULAR_GAIN * err
        linear = FORWARD_SPEED * max(0.0, 1.0 - abs(err))
        self.publish_cmd(linear, angular)


def main(args=None):
    rclpy.init(args=args)
    node = SeekAndFindNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.publish_cmd(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
