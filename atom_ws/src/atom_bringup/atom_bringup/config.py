# ── Server [exploration_coordinator.py, object_detector.py] ──────────────────
SERVER_URL = 'http://192.168.50.16:5000'   # Flask inference server on host laptop

# ── Object detector [object_detector.py] ──────────────────────────────────────
CONFIDENCE_THRESHOLD  = 0.3    # minimum YOLO confidence to consider a detection
DETECTION_INTERVAL    = 0.5    # seconds between YOLO inference calls
SPOTTED_COOLDOWN      = 0.5    # seconds between /atom/object_spotted publishes
TARGET_LOST_TIMEOUT   = 5.0    # seconds before declaring target lost


