import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np
import json


class DepthReader(Node):
    def __init__(self):
        super().__init__('depth_reader')

        self.latest_depth  = None
        self.latest_bbox   = None
        self.depth_width   = 640
        self.depth_height  = 400
        self.yolo_width    = 320
        self.yolo_height   = 240
        self.scale_x       = self.depth_width  / self.yolo_width
        self.scale_y       = self.depth_height / self.yolo_height
        self.min_depth_mm  = 200
        self.max_depth_mm  = 8000

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.create_subscription(
            Image, '/oakd/stereo/image_raw', self.depth_callback, qos)
        self.create_subscription(
            String, '/atom/depth_bbox', self.bbox_callback, 10)

        self.create_service(
            Trigger, '/atom/get_depth', self.get_depth_callback)

        self.depth_frame_count = 0
        self.get_logger().info(
            'DepthReader started | '
            'service: /atom/get_depth | '
            'method: IQR + histogram peak + min of cluster'
        )

    def depth_callback(self, msg: Image):
        try:
            self.latest_depth = np.frombuffer(
                msg.data, dtype=np.uint16
            ).reshape(msg.height, msg.width)
            self.depth_width  = msg.width
            self.depth_height = msg.height
            self.scale_x = self.depth_width  / self.yolo_width
            self.scale_y = self.depth_height / self.yolo_height
            self.depth_frame_count += 1
            if self.depth_frame_count % 50 == 0:
                self.get_logger().info(
                    f'Depth OK | frames: {self.depth_frame_count} | '
                    f'{msg.width}x{msg.height}'
                )
        except Exception as e:
            self.get_logger().error(f'depth_callback failed: {e}')

    def bbox_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.latest_bbox = data.get('bbox', None)
            self.get_logger().info(
                f'Bbox received: {self.latest_bbox} '
                f'class: {data.get("class", "unknown")}'
            )
        except Exception as e:
            self.get_logger().error(f'bbox_callback failed: {e}')

    def get_depth_callback(self, request, response):
        if self.latest_depth is None:
            response.success = False
            response.message = 'No depth frame available yet'
            return response

        if self.latest_bbox is None:
            response.success = False
            response.message = 'No bbox received yet'
            return response

        try:
            x1, y1, x2, y2 = self.latest_bbox

            dx1 = int(np.clip(x1 * self.scale_x, 0, self.depth_width  - 1))
            dy1 = int(np.clip(y1 * self.scale_y, 0, self.depth_height - 1))
            dx2 = int(np.clip(x2 * self.scale_x, 0, self.depth_width  - 1))
            dy2 = int(np.clip(y2 * self.scale_y, 0, self.depth_height - 1))

            if dx2 <= dx1 or dy2 <= dy1:
                response.success = False
                response.message = 'Invalid bbox dimensions'
                return response

            roi  = self.latest_depth[dy1:dy2, dx1:dx2].astype(np.float32)
            flat = roi.flatten()

            valid = flat[(flat >= self.min_depth_mm) & (flat <= self.max_depth_mm)]

            if len(valid) < 5:
                response.success = False
                response.message = 'Too few valid depth pixels in bbox'
                self.get_logger().warn(
                    f'Only {len(valid)} valid pixels in bbox '
                    f'({dx1},{dy1})→({dx2},{dy2})'
                )
                return response

            Q1  = np.percentile(valid, 25)
            Q3  = np.percentile(valid, 75)
            IQR = Q3 - Q1
            iqr_filtered = valid[
                (valid >= Q1 - 1.5 * IQR) &
                (valid <= Q3 + 1.5 * IQR)
            ]

            if len(iqr_filtered) < 3:
                iqr_filtered = valid

            bin_size   = 50.0
            val_min    = float(np.min(iqr_filtered))
            val_max    = float(np.max(iqr_filtered))
            n_bins     = max(int((val_max - val_min) / bin_size) + 1, 1)
            bins       = np.linspace(val_min, val_max, n_bins + 1)
            hist, edges = np.histogram(iqr_filtered, bins=bins)

            if len(hist) >= 5:
                kernel     = np.array([1, 2, 3, 2, 1], dtype=float)
                kernel    /= kernel.sum()
                hist_smooth = np.convolve(hist, kernel, mode='same')
            else:
                hist_smooth = hist.astype(float)

            threshold  = 0.05 * hist_smooth.max()
            peak_idx   = 0
            for i in range(len(hist_smooth)):
                if hist_smooth[i] > threshold:
                    peak_idx = i
                    break

            peak_center = float(edges[peak_idx] + bin_size / 2.0)

            peak_cluster = iqr_filtered[
                (iqr_filtered >= peak_center - 150) &
                (iqr_filtered <= peak_center + 150)
            ]

            if len(peak_cluster) == 0:
                distance_mm = float(np.min(iqr_filtered))
            else:
                distance_mm = float(np.min(peak_cluster))

            distance_m = distance_mm / 1000.0

            self.get_logger().info(
                f'Depth OK | bbox: ({dx1},{dy1})→({dx2},{dy2}) | '
                f'total: {len(flat)} | valid: {len(valid)} | '
                f'iqr: {len(iqr_filtered)} | cluster: {len(peak_cluster)} | '
                f'peak: {peak_center/1000:.2f}m | '
                f'distance: {distance_m:.3f}m'
            )

            response.success = True
            response.message = f'{distance_m:.3f}'
            return response

        except Exception as e:
            self.get_logger().error(f'get_depth_callback failed: {e}')
            response.success = False
            response.message = f'Error: {str(e)}'
            return response


def main():
    rclpy.init()
    node = DepthReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()