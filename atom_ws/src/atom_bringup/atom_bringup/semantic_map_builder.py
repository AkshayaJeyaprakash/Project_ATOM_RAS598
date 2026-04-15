import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math
import tf2_ros
from geometry_msgs.msg import PointStamped
import rclpy.duration

class SemanticMapBuilder(Node):
    def __init__(self):
        super().__init__('semantic_map_builder')
        self.object_map = []
        self.next_id = 1
        self.declare_parameter('dedup_threshold', 0.5)
        self.dedup_threshold = self.get_parameter('dedup_threshold').value
        self.create_subscription(String, '/atom/detections', self.detections_callback, 10)
        self.map_pub = self.create_publisher(String, '/atom/semantic_map', 10)
        self.get_logger().info('Semantic Map Builder started')
        self.create_subscription(String, '/task_command', self.task_callback, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def task_callback(self, msg):
        self.get_logger().info('Task received — re-publishing current map...')
        self.publish_map()

    def detections_callback(self, msg):
        detections = json.loads(msg.data)
        for d in detections:
            if 'bbox' in d and 'depth' in d:
                bbox = d['bbox']
                pixel_x = (bbox[0] + bbox[2]) / 2
                pixel_y = (bbox[1] + bbox[3]) / 2
                depth = d['depth']
                x, y = self.get_map_coordinates(depth, pixel_x, pixel_y)
            else:
                x, y = 0.0, 0.0
            self.update_map(d, x, y)
        self.publish_map()

    def update_map(self, detection, x=0.0, y=0.0):
        obj_class = detection['class']
        color = detection['avg_color_rgb']
        confidence = detection['confidence']

        existing = self._find_existing(obj_class, x, y)

        if existing:
            existing['x'] = (existing['x'] + x) / 2
            existing['y'] = (existing['y'] + y) / 2
            existing['confidence'] = max(existing['confidence'], confidence)
            existing['seen_count'] += 1
            self.get_logger().info(
                f"Updated: {obj_class} (seen {existing['seen_count']}x) "
                f"at ({existing['x']:.2f}, {existing['y']:.2f})"
            )
        else:
            obj_id = f"{obj_class}_{self.next_id:03d}"
            self.next_id += 1
            self.object_map.append({'id': obj_id, 'class': obj_class, 'color_rgb': color, 'x': x, 'y': y, 'confidence': confidence, 'seen_count': 1, 'status': 'present'})
            self.get_logger().info(f"New object: {obj_id} at ({x:.2f}, {y:.2f})")

    def get_map_coordinates(self, depth, pixel_x, pixel_y, image_width=416, image_height=416):
        try:
            fx = 421.0
            fy = 421.0
            cx = image_width / 2
            cy = image_height / 2

            x_cam = (pixel_x - cx) * depth / fx
            y_cam = (pixel_y - cy) * depth / fy
            z_cam = depth

            point_cam = PointStamped()
            point_cam.header.frame_id = 'oakd_rgb_camera_optical_frame'
            point_cam.header.stamp = rclpy.time.Time().to_msg()
            point_cam.point.x = x_cam
            point_cam.point.y = y_cam
            point_cam.point.z = z_cam

            point_map = self.tf_buffer.transform(
                point_cam, 'map', timeout=rclpy.duration.Duration(seconds=0.5)
            )
            return point_map.point.x, point_map.point.y

        except Exception as e:
            self.get_logger().warn(f'TF2 transform failed: {e}')
            return 0.0, 0.0

    def _find_existing(self, obj_class, x, y):
        for obj in self.object_map:
            if obj['class'] == obj_class:
                dist = math.sqrt((obj['x'] - x) ** 2 + (obj['y'] - y) ** 2)
                if dist < self.dedup_threshold:
                    return obj
        return None

    def lookup(self, description):
        for obj in self.object_map:
            if description.lower() in obj['class'].lower() and obj['status'] == 'present':
                return obj
        return None

    def mark_missing(self, obj_id):
        for obj in self.object_map:
            if obj['id'] == obj_id:
                obj['status'] = 'missing'
                self.get_logger().warn(f"Object {obj_id} marked as missing")
                return

    def publish_map(self):
        msg = String()
        msg.data = json.dumps(self.object_map)
        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapBuilder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()