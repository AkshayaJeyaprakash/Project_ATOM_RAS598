import numpy as np
from nav_msgs.msg import OccupancyGrid
import cv2


def generate_scan_points(map_msg: OccupancyGrid, num_points: int = 4) -> list:
    width = map_msg.info.width
    height = map_msg.info.height
    resolution = map_msg.info.resolution
    origin_x = map_msg.info.origin.position.x
    origin_y = map_msg.info.origin.position.y

    grid = np.array(map_msg.data, dtype=np.int8).reshape((height, width))
    free_space = (grid == 0).astype(np.uint8)
    kernel = np.ones((5, 5), np.uint8)
    free_space_eroded = cv2.erode(free_space, kernel, iterations=3)
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
        free_space_eroded, connectivity=8
    )

    if num_labels < 2:
        return []
    components = []
    for i in range(1, num_labels):
        area = stats[i, cv2.CC_STAT_AREA]
        cx = centroids[i][0]
        cy = centroids[i][1]
        components.append((area, cx, cy))
    components.sort(key=lambda x: x[0], reverse=True)
    scan_points = []
    for area, cx, cy in components[:num_points]:
        map_x = origin_x + cx * resolution
        map_y = origin_y + cy * resolution
        scan_points.append((round(map_x, 2), round(map_y, 2)))

    return scan_points

def get_closest_scan_point(scan_points: list, robot_x: float, robot_y: float) -> tuple:
    if not scan_points:
        return (0.0, 0.0)

    distances = [
        ((x - robot_x) ** 2 + (y - robot_y) ** 2) ** 0.5
        for x, y in scan_points
    ]
    closest_idx = np.argmin(distances)
    return scan_points[closest_idx]