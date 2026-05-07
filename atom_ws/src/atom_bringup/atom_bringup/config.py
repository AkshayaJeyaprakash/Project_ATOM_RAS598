# ── Server [exploration_coordinator.py, object_detector.py] ──────────────────
SERVER_URL = 'http://192.168.50.16:5000'   # Flask inference server on host laptop

# ── Object detector [object_detector.py] ──────────────────────────────────────
CONFIDENCE_THRESHOLD  = 0.3    # minimum YOLO confidence to consider a detection
DETECTION_INTERVAL    = 0.5    # seconds between YOLO inference calls
SPOTTED_COOLDOWN      = 0.5    # seconds between /atom/object_spotted publishes
TARGET_LOST_TIMEOUT   = 5.0    # seconds before declaring target lost

# ── Goal publisher [goal_publisher.py] ────────────────────────────────────────
STOP_DISTANCE_THRESHOLD = 0.6    # distance threshold for object detection stop (meters)
COSTMAP_SETTLE_DELAY    = 1.5    # seconds to wait after costmap clear before sending goal

# ── Memory [exploration_coordinator.py, memory_mapper.py] ────────────────────
MEMORY_FILE = '~/maps/memory.json'   # path to memory JSON file

# ── Safety monitor [safety_monitor.py] ────────────────────────────────────────
LOW_BATTERY_THRESHOLD = 0.05     # battery percentage to trigger autodock (0.0-1.0 = 20%)
BATTERY_CHECK_INTERVAL = 30.0   # seconds between battery checks