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

print("Loading YOLO...")
yolo_model = YOLO("yolov8n.pt")

print("Loading CLIP...")
clip_model, _, clip_preprocess = open_clip.create_model_and_transforms(
    "ViT-B-32", pretrained="laion2b_s34b_b79k"
)
clip_model = clip_model.cuda().eval()
clip_tokenizer = open_clip.get_tokenizer("ViT-B-32")

print("All models loaded.")

OLLAMA_URL = "http://localhost:11434/api/generate"

YOLO_CLASSES = (
    "person, bicycle, car, motorcycle, airplane, bus, train, truck, boat, "
    "traffic light, fire hydrant, stop sign, parking meter, bench, bird, cat, "
    "dog, horse, sheep, cow, elephant, bear, zebra, giraffe, backpack, umbrella, "
    "handbag, tie, suitcase, frisbee, skis, snowboard, sports ball, kite, "
    "baseball bat, baseball glove, skateboard, surfboard, tennis racket, bottle, "
    "wine glass, cup, fork, knife, spoon, bowl, banana, apple, sandwich, orange, "
    "broccoli, carrot, hot dog, pizza, donut, cake, chair, couch, potted plant, "
    "bed, dining table, toilet, tv, laptop, mouse, remote, keyboard, cell phone, "
    "microwave, oven, toaster, sink, refrigerator, book, clock, vase, scissors, "
    "teddy bear, hair drier, toothbrush"
)

VALID_YOLO_CLASSES = [c.strip() for c in YOLO_CLASSES.split(",")]

SYSTEM_PROMPT = f"""You are an AI assistant for a mobile robot navigation system called ATOM.

Your role is to help the robot find objects in a real indoor environment by:
1. Identifying which YOLO detection class best matches what the user wants to find
2. Reasoning about which objects in the robot's memory are spatially near the target
3. Extracting a short 2-4 word description of the target for visual validation
4. Verifying if a detected object in a camera image matches the user's request

The robot's object detector (YOLO) can detect exactly these 80 classes:
{YOLO_CLASSES}

CRITICAL RULES:
- YOLO_CLASS must be EXACTLY one of the 80 class names above — no variations, no extra words
- If target is not in YOLO classes, pick the closest match (e.g. "thermos" → "bottle", "sofa" → "couch")
- MEMORY_OBJECT must be EXACTLY one of the object names from the memory list provided, or "none"
- DESCRIPTION must be 2-4 words only — the most important visual features of the target

Always be concise. Follow the exact output format requested in each prompt.
Never add extra explanation unless asked."""


def call_llava(prompt: str, image_b64: str = None, timeout: int = 20) -> str:
    payload = {
        "model": "llava",
        "system": SYSTEM_PROMPT,
        "prompt": prompt,
        "stream": False
    }
    if image_b64:
        payload["images"] = [image_b64]
    r = requests.post(OLLAMA_URL, json=payload, timeout=timeout)
    r.raise_for_status()
    return r.json()["response"].strip()


def _find_closest_yolo_class(raw: str) -> str:
    raw = raw.lower().strip()
    if raw in VALID_YOLO_CLASSES:
        return raw
    for cls in VALID_YOLO_CLASSES:
        if raw in cls or cls in raw:
            return cls
    raw_words = set(raw.split())
    for cls in VALID_YOLO_CLASSES:
        cls_words = set(cls.split())
        if raw_words & cls_words:
            return cls
    return "none"


@app.route("/health", methods=["GET"])
def health():
    return jsonify({"status": "ok", "gpu": torch.cuda.get_device_name(0)})


@app.route("/detect", methods=["POST"])
def detect():
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


@app.route("/llava_parse_task", methods=["POST"])
def llava_parse_task():
    data = request.get_json()
    task = data.get("task", "").strip()
    memory_list = data.get("memory_list", "memory is empty")

    if not task:
        return jsonify({
            "yolo_class": "none",
            "memory_object": "none",
            "description": "none",
            "reasoning": "Empty task provided"
        })

    valid_memory_objects = []
    if memory_list != "memory is empty":
        valid_memory_objects = [o.strip().lower() for o in memory_list.split(",")]
    memory_objects_str = ", ".join(valid_memory_objects) if valid_memory_objects else "none"

    prompt = (
        f"User task: '{task}'\n"
        f"Robot memory contains ONLY these objects: {memory_objects_str}\n\n"
        f"Answer in this EXACT format with no extra text:\n"
        f"YOLO_CLASS: <one exact class from the 80 YOLO classes>\n"
        f"MEMORY_OBJECT: <one exact name from the memory list above>\n"
        f"DESCRIPTION: <2-4 words>\n"
        f"REASON: <one sentence>\n\n"
        f"STRICT RULES:\n"
        f"- YOLO_CLASS: EXACTLY one of the 80 YOLO class names — copy exactly\n"
        f"- MEMORY_OBJECT rules — follow in order:\n"
        f"  RULE 1: If YOLO_CLASS exactly matches a name in the memory list → use that name\n"
        f"  RULE 2: If YOLO_CLASS NOT in memory list → pick the memory object that is most likely to be in the SAME ROOM or NEAR the target object in a real home\n"
        f"  RULE 3: NEVER return 'none' unless the memory list is completely empty\n"
        f"  RULE 4: MEMORY_OBJECT must be EXACTLY one name from the memory list — no other words\n"
        f"- DESCRIPTION rules:\n"
        f"  If user mentioned color/size/material → include those (e.g. 'red bottle', 'small cup')\n"
        f"  If NO visual features mentioned → use ONLY the object name (e.g. 'cup', 'bottle')\n"
        f"  NEVER invent colors or features not mentioned by user\n\n"
        f"Examples (memory='bottle, oven, chair, bed'):\n"
        f"  task='find the red bottle' → YOLO_CLASS: bottle | MEMORY_OBJECT: bottle | DESCRIPTION: red bottle\n"
        f"  task='find the cup' → YOLO_CLASS: cup | MEMORY_OBJECT: bottle | DESCRIPTION: cup\n"
        f"  task='find the bed' → YOLO_CLASS: bed | MEMORY_OBJECT: bed | DESCRIPTION: bed\n"
        f"  task='find coffee mug' → YOLO_CLASS: cup | MEMORY_OBJECT: bottle | DESCRIPTION: coffee mug\n"
        f"  task='I am thirsty find my water bottle please' → YOLO_CLASS: bottle | MEMORY_OBJECT: bottle | DESCRIPTION: water bottle"
    )

    for attempt in range(2):
        try:
            response_text = call_llava(prompt)

            yolo_class = "none"
            memory_object = "none"
            description = "none"
            reasoning = ""

            for line in response_text.split("\n"):
                line = line.strip().replace("\\_", "_")
                if line.startswith("YOLO_CLASS:"):
                    yolo_class = line.split("YOLO_CLASS:")[-1].strip().lower()
                elif line.startswith("MEMORY_OBJECT:"):
                    memory_object = line.split("MEMORY_OBJECT:")[-1].strip().lower()
                elif line.startswith("DESCRIPTION:"):
                    description = line.split("DESCRIPTION:")[-1].strip().lower()
                elif line.startswith("REASON:"):
                    reasoning = line.split("REASON:")[-1].strip()

            if yolo_class not in VALID_YOLO_CLASSES:
                yolo_class = _find_closest_yolo_class(yolo_class)

            if memory_list != "memory is empty":
                valid_memory_objects_check = [o.strip().lower() for o in memory_list.split(",")]
                if memory_object not in valid_memory_objects_check:
                    matched = False
                    for obj in valid_memory_objects_check:
                        if memory_object in obj or obj in memory_object:
                            memory_object = obj
                            matched = True
                            break
                    if not matched:
                        memory_object = valid_memory_objects_check[0] if valid_memory_objects_check else "none"
            else:
                memory_object = "none"

            if not description or description == "none":
                description = yolo_class

            return jsonify({
                "yolo_class": yolo_class,
                "memory_object": memory_object,
                "description": description,
                "reasoning": reasoning,
                "raw": response_text
            })

        except Exception as e:
            if attempt == 0:
                continue
            fallback_class = _find_closest_yolo_class(task)
            return jsonify({
                "yolo_class": fallback_class,
                "memory_object": "none",
                "description": fallback_class,
                "reasoning": f"LLaVA failed after 2 attempts: {str(e)}",
                "raw": ""
            })


@app.route("/llava_reason", methods=["POST"])
def llava_reason():
    data = request.get_json()
    img_b64 = data["image"]
    target = data["target"]

    prompt = (
        f"Look at this image carefully. Is there a '{target}' visible?\n"
        f"Answer in this EXACT format:\n"
        f"PRESENT: yes/no\n"
        f"CONFIDENCE: high/medium/low\n"
        f"REASON: one sentence explanation"
    )

    try:
        response_text = call_llava(prompt, image_b64=img_b64)

        present = False
        confidence = "low"
        reasoning = response_text

        for line in response_text.split("\n"):
            line = line.strip().replace("\\_", "_")
            if line.startswith("PRESENT:"):
                present = "yes" in line.split("PRESENT:")[-1].lower()
            elif line.startswith("CONFIDENCE:"):
                conf = line.split("CONFIDENCE:")[-1].strip().lower()
                if "high" in conf:
                    confidence = "high"
                elif "medium" in conf:
                    confidence = "medium"
            elif line.startswith("REASON:"):
                reasoning = line.split("REASON:")[-1].strip()

        return jsonify({
            "present": present,
            "confidence": confidence,
            "reasoning": reasoning,
            "raw": response_text
        })

    except Exception as e:
        return jsonify({
            "present": False,
            "confidence": "low",
            "reasoning": f"LLaVA failed: {str(e)}",
            "raw": ""
        })


@app.route("/llava_proximity", methods=["POST"])
def llava_proximity():
    data = request.get_json()
    target = data.get("target", "")
    memory_list = data.get("memory_list", "memory is empty")

    prompt = (
        f"I am looking for a '{target}'.\n"
        f"My memory contains: {memory_list}\n\n"
        f"Which object in my memory is most likely to be physically near a '{target}'?\n"
        f"Answer in this EXACT format:\n"
        f"NEAREST: <exact object name from memory or 'none'>\n"
        f"CONFIDENCE: high/medium/low\n"
        f"REASON: one sentence explanation"
    )

    try:
        response_text = call_llava(prompt)

        nearest_object = "none"
        confidence = "low"
        reasoning = response_text

        for line in response_text.split("\n"):
            line = line.strip()
            if line.startswith("NEAREST:"):
                nearest_object = line.split("NEAREST:")[-1].strip().lower()
            elif line.startswith("CONFIDENCE:"):
                conf = line.split("CONFIDENCE:")[-1].strip().lower()
                if "high" in conf:
                    confidence = "high"
                elif "medium" in conf:
                    confidence = "medium"
            elif line.startswith("REASON:"):
                reasoning = line.split("REASON:")[-1].strip()

        return jsonify({
            "nearest_object": nearest_object,
            "confidence": confidence,
            "reasoning": reasoning,
            "raw": response_text
        })

    except Exception as e:
        return jsonify({
            "nearest_object": "none",
            "confidence": "low",
            "reasoning": f"LLaVA failed: {str(e)}",
            "raw": ""
        })


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)