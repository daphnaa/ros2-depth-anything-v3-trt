#!/usr/bin/env python3
import json
import os
import requests

URL = "http://127.0.0.1:5070/bbox_depth"

img_path = "/home/daphnaa/Pictures/ExpoTLV/living_room/20260212_173804220_iOS_4K_Wide.jpg"
js_path  = "/home/daphnaa/Pictures/ExpoTLV/living_room/20260212_173804220_iOS_4K_Wide.json"

with open(js_path, "r") as f:
    j = json.load(f)

dets = j["nanoowl"]["result"]["detections"]

# Build a payload your server already accepts
payload = {
    "nanoowl": {
        "result": {
            "detections": dets,
            "image": {"width": 3840, "height": 2160},  # matches your camera_yaml style
        }
    }
}

with open(img_path, "rb") as f_img:
    files = {
        "image": (os.path.basename(img_path), f_img, "image/jpeg"),
        "detections": ("detections.json", json.dumps(payload), "application/json"),
    }
    data = {"image_dir": os.path.dirname(img_path)}
    r = requests.post(URL, files=files, data=data, timeout=60)

print("status:", r.status_code)
print(r.text)
