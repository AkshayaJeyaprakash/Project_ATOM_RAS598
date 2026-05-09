import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
import json
import math
import os
import time

from atom_bringup.config import MEMORY_FILE as _MEMORY_FILE
import os
MEMORY_FILE = os.path.expanduser(_MEMORY_FILE)


class MemoryMapper(Node):
    def __init__(self):
        super().__init__('memory_mapper')
        self._robot_x   = 0.0
        self._robot_y   = 0.0
        self._robot_yaw = 0.0
        self._pose_received = False
        self._memory = self._load_memory()

        qos_transient = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        self.create_subscription(String, '/atom/detections', self.object_spotted_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/pose', self.amcl_pose_callback, 10)
        self.create_timer(5.0, self._save_memory)

        self.get_logger().info(
            f'MemoryMapper started | saving to: {MEMORY_FILE}\n'
            f'Current memory: {list(self._memory.keys())}'
        )

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._robot_yaw = math.atan2(siny, cosy)
        self._pose_received = True

    def object_spotted_callback(self, msg: String):
        if not self._pose_received:
            self.get_logger().warn('No pose yet — skipping detection')
            return
        try:
            detections = json.loads(msg.data)
            if not isinstance(detections, list):
                detections = [detections]

            for data in detections:
                obj_class  = data.get('class', '').lower().strip()
                confidence = float(data.get('confidence', 0.0))
                bbox       = data.get('bbox', None)
                if not obj_class:
                    continue
                entry = {
                    'x':          round(self._robot_x, 3),
                    'y':          round(self._robot_y, 3),
                    'yaw':        round(self._robot_yaw, 4),
                    'confidence': round(confidence, 3),
                    'bbox':       bbox,
                    'timestamp':  time.time()
                }

                if obj_class not in self._memory:
                    self._memory[obj_class] = []

                self._memory[obj_class].append(entry)
                self._memory[obj_class].sort(key=lambda e: e['confidence'], reverse=True)

                self.get_logger().info(
                    f'Stored: {obj_class} | conf: {confidence:.2f} | '
                    f'pose: ({self._robot_x:.2f}, {self._robot_y:.2f}, '
                    f'{math.degrees(self._robot_yaw):.1f}°) | '
                    f'total entries for {obj_class}: {len(self._memory[obj_class])}'
                )
        except Exception as e:
            self.get_logger().error(f'object_spotted_callback failed: {e}')

    def _load_memory(self) -> dict:
        if os.path.exists(MEMORY_FILE):
            try:
                with open(MEMORY_FILE, 'r') as f:
                    data = json.load(f)
                self.get_logger().info(
                    f'Loaded existing memory: {MEMORY_FILE} | '
                    f'objects: {list(data.keys())}'
                )
                return data
            except Exception as e:
                self.get_logger().warn(f'Failed to load memory: {e} — starting fresh')
        return {}

    def _save_memory(self):
        try:
            os.makedirs(os.path.dirname(MEMORY_FILE), exist_ok=True)
            with open(MEMORY_FILE, 'w') as f:
                json.dump(self._memory, f, indent=2)
            total = sum(len(v) for v in self._memory.values())
            self.get_logger().info(
                f'Memory saved | objects: {list(self._memory.keys())} | '
                f'total entries: {total}'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to save memory: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MemoryMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down — saving final memory...')
        node._save_memory()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()