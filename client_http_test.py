#!/usr/bin/env python3
import json
import requests

URL = "http://127.0.0.1:5070/bbox_depth"

img_path = "/home/user/jetson-containers/data/R1/2026_02_11___15_35_42/frame_2026_02_11___15_33_31_tile_0_0.jpg"
# det_path = "/home/user/jetson-containers/data/R1/2026_02_11___15_35_42/frame_2026_02_11___15_33_31_tile_0_0.json"  # nanoowl sidecar or detections array

# files = {
#     "image": (img_path.split("/")[-1], open(img_path, "rb"), "image/jpeg"),
#     "detections": (det_path.split("/")[-1], open(det_path, "rb"), "application/json"),
# }

nanoowl_dict = {
      "detections": [
        {
          "bbox": [
            52,
            614,
            447,
            1028
          ],
          "label": "a microwave",
          "score": 0.46083617210388184
        },
        {
          "bbox": [
            257,
            542,
            2236,
            801
          ],
          "label": "a shelf",
          "score": 0.24989540874958038
        },
        {
          "bbox": [
            2088,
            521,
            2241,
            718
          ],
          "label": "a plant",
          "score": 0.22322598099708557
        },
        {
          "bbox": [
            185,
            297,
            327,
            537
          ],
          "label": "a plant",
          "score": 0.20964086055755615
        },
        {
          "bbox": [
            1815,
            421,
            2004,
            689
          ],
          "label": "a plant",
          "score": 0.2043023556470871
        },
        {
          "bbox": [
            181,
            509,
            316,
            609
          ],
          "label": "a pot",
          "score": 0.20766690373420715
        },
        {
          "bbox": [
            856,
            749,
            975,
            1005
          ],
          "label": "a faucet",
          "score": 0.27345994114875793
        },
        {
          "bbox": [
            9,
            957,
            2299,
            1364
          ],
          "label": "a counter",
          "score": 0.24294091761112213
        }
      ],
      "image": {
        "height": 1296,
        "width": 2304
      },
      "latency_sec": 0.0978,
      "prompts": [
        "a microwave",
        "a coffee maker",
        "a sink",
        "a chair",
        "a shelf",
        "a plant",
        "a pot",
        "a bottle",
        "a jar",
        "a container",
        "a refrigerator",
        "a faucet",
        "a soap dispenser",
        "a counter",
        "a stool"
      ]
    }


files = {
    "image": (
        img_path.split("/")[-1],
        open(img_path, "rb"),
        "image/jpeg",
    ),
    "detections": (
        "detections.json",
        json.dumps(nanoowl_dict),
        "application/json",
    ),
    
}
import urllib.parse
url = URL + "?image_path=" + urllib.parse.quote(img_path, safe="")
r = requests.post(url, files=files)

print(r.status_code)
print(json.dumps(r.json(), indent=2))
