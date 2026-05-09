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

# ── Validation [exploration_coordinator.py] ───────────────────────────────────
CLIP_THRESHOLD = 0.24    # ViT-B-32 laion2b cosine similarity threshold
                         # strong match >0.28, moderate 0.22-0.28
VOTES_REQUIRED = 2       # minimum votes out of 3 (YOLO + CLIP + LLaVA) to confirm

# ── Scanning rotation [exploration_coordinator.py] ────────────────────────────
SCAN_TOTAL_DEG = 420.0   # total degrees commanded (>360 to compensate deceleration)
SCAN_STEP_DEG  = 20.0    # degrees per rotation step
SCAN_PAUSE_S   = 2.0     # pause duration between steps (seconds)
SCAN_SPEED     = 0.5     # rotation speed (rad/s)

# ── Scan point generation [exploration_coordinator.py] ────────────────────────
SCAN_SPACING_M   = 1.0   # distance between scan points in boustrophedon grid (meters)
WALL_CLEARANCE_M = 0.20  # minimum clearance from walls for scan points (meters)

# ── Approach / driving [exploration_coordinator.py] ───────────────────────────
DRIVE_SPEED          = 0.15   # forward drive speed during approach (m/s)
STOP_DISTANCE_M      = 0.45   # stop when object is this close (meters)
STEREO_RELIABLE_M    = 1.5    # stereo depth reliable below this distance (meters)
DRIVE_1M_DIST        = 1.0    # distance per driving step when far from object (meters)
DEPTH_CHECK_INTERVAL = 0.5    # how often to check depth during approach (seconds)

# ── Centering [exploration_coordinator.py] ────────────────────────────────────
FRAME_W          = 320.0   # camera frame width (pixels) — OAK-D Lite 320x240
FRAME_CENTER_X   = FRAME_W / 2.0   # horizontal center of frame
CENTER_THRESHOLD = 20.0    # pixel error within which robot is considered centered
DRIFT_THRESHOLD  = 50.0    # pixel drift during approach that triggers re-centering
CONFIRM_COUNT    = 3       # number of consecutive centered readings required
KP               = 0.003   # proportional gain for centering rotation
MAX_ROT_SPEED    = 0.3     # maximum rotation speed during centering (rad/s)
MIN_ROT_SPEED    = 0.05    # minimum rotation speed during centering (rad/s)

# ── Memory [exploration_coordinator.py, memory_mapper.py] ────────────────────
MEMORY_FILE = '~/maps/memory.json'   # path to memory JSON file