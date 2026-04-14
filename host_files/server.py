from flask import Flask, request, jsonify
from ultralytics import YOLO
import open_clip
import torch
from PIL import Image
import io
import base64
import numpy as np
import requests

app = Flask(__name__)

yolo_model = YOLO("yolov8n.pt")
clip_model, _, clip_preprocess = open_clip.create_model_and_transforms("ViT-B-32", pretrained="laion2b_s34b_b79k")
clip_model = clip_model.cuda().eval()
clip_tokenizer = open_clip.get_tokenizer("ViT-B-32")

print("All models loaded.")

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

@app.route("/clip_score", methods=["POST"])
def clip_score():
    """Score how well an image matches a text description."""
    data = request.get_json()
    img_bytes = base64.b64decode(data["image"])
    img = Image.open(io.BytesIO(img_bytes)).convert("RGB")
    text = data["text"]

    img_tensor = clip_preprocess(img).unsqueeze(0).cuda()
    text_tokens = clip_tokenizer([text]).cuda()

    with torch.no_grad():
        img_features = clip_model.encode_image(img_tensor)
        text_features = clip_model.encode_text(text_tokens)
        img_features /= img_features.norm(dim=-1, keepdim=True)
        text_features /= text_features.norm(dim=-1, keepdim=True)
        score = (img_features @ text_features.T).item()

    return jsonify({"score": round(score, 4), "text": text})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)