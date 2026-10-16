#!/usr/bin/env python3
"""Generate an A4, true-size ArUco marker for the mannequin knee test."""

from pathlib import Path

import cv2
from PIL import Image, ImageDraw, ImageFont


DPI = 300
PX_PER_MM = DPI / 25.4
MARKER_MM = 100
MARKER_ID = 23
OUTPUT_DIR = Path(__file__).resolve().parent


def mm(value):
    return round(value * PX_PER_MM)


def main():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = cv2.aruco.generateImageMarker(
        dictionary, MARKER_ID, mm(MARKER_MM), borderBits=1)
    page = Image.new("RGB", (mm(210), mm(297)), "white")
    left, top = mm(55), mm(45)
    page.paste(Image.fromarray(marker).convert("RGB"), (left, top))

    draw = ImageDraw.Draw(page)
    font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    font = ImageFont.truetype(font_path, 38)
    draw.text((left, top + mm(110)),
              "ArUco 4x4_50  ID 23", fill="black", font=font)
    draw.text((left, top + mm(119)),
              "Black marker edge: 100 mm. Print at 100% scale.",
              fill="black", font=font)
    line_y = top + mm(138)
    draw.line((left, line_y, left + mm(100), line_y), fill="black", width=4)
    for x in (left, left + mm(100)):
        draw.line((x, line_y - mm(3), x, line_y + mm(3)),
                  fill="black", width=4)
    draw.text((left + mm(35), line_y + mm(5)), "100 mm reference",
              fill="black", font=font)

    pdf_path = OUTPUT_DIR / "mannequin_knee_marker_A4.pdf"
    png_path = OUTPUT_DIR / "mannequin_knee_marker_A4.png"
    page.save(pdf_path, "PDF", resolution=DPI)
    page.save(png_path, "PNG", dpi=(DPI, DPI))
    print(f"[OK] {pdf_path}")
    print(f"[OK] {png_path}")
    print("[CHECK] print PDF at 100% and verify the reference line is 100 mm")


if __name__ == "__main__":
    main()
