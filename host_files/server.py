from flask import Flask, request, jsonify
from ultralytics import YOLO
import torch
from PIL import Image
import io
import base64
import numpy as np
import requests

app = Flask(__name__)

yolo_model = YOLO("yolov8n.pt")

@app.route("/health", methods=["GET"])
def health():
    return jsonify({"status": "ok", "gpu": torch.cuda.get_device_name(0)})

@app.route("/detect", methods=["POST"])
def detect():
    """
    Receive base64 image, return YOLO detections.
    Returns: list of detections with class, confidence, bbox, avg_color_rgb, depth
    """
    data = request.get_json()
    img_bytes = base64.b64decode(data["image"])
    img = Image.open(io.BytesIO(img_bytes)).convert("RGB")
    results = yolo_model(img, verbose=False)
    detections = []
    img_np = np.array(img)

    for r in results:
        for box in r.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            crop = img_np[y1:y2, x1:x2]
            avg_color = crop.mean(axis=(0, 1)).tolist() if crop.size > 0 else [0, 0, 0]

            # Estimate depth from bbox size
            bbox_area = (x2 - x1) * (y2 - y1)
            img_area = img_np.shape[0] * img_np.shape[1]
            estimated_depth = max(0.5, 3.0 * (1.0 - bbox_area / img_area))

            detections.append({
                "class": yolo_model.names[int(box.cls)],
                "confidence": round(float(box.conf), 3),
                "bbox": [x1, y1, x2, y2],
                "avg_color_rgb": avg_color,
                "depth": round(estimated_depth, 2),
            })

    return jsonify({"detections": detections})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)