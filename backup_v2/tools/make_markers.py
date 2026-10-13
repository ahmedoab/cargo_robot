#!/usr/bin/env python3
"""Printable ArUco station markers (dictionary DICT_4X4_50, same as the vision node).

    python3 tools/make_markers.py 0 1 2      -> markers/marker_0.png, marker_1.png, marker_2.png
Print each at 10 x 10 cm (the black square), tape it flat on the floor next to the line.
"""
import sys
from pathlib import Path

import cv2

ids = [int(a) for a in sys.argv[1:]] or [0, 1, 2]
aruco = cv2.aruco
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
out = Path(__file__).resolve().parents[1] / "markers"
out.mkdir(exist_ok=True)
for i in ids:
    if hasattr(aruco, "generateImageMarker"):
        img = aruco.generateImageMarker(dictionary, i, 600)
    else:
        img = aruco.drawMarker(dictionary, i, 600)
    img = cv2.copyMakeBorder(img, 100, 160, 100, 100, cv2.BORDER_CONSTANT, value=255)
    cv2.putText(img, f"station {i}", (100, 790), cv2.FONT_HERSHEY_SIMPLEX, 1.5, 0, 3)
    cv2.imwrite(str(out / f"marker_{i}.png"), img)
    print("wrote", out / f"marker_{i}.png")
