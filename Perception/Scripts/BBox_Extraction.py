import os
import cv2
from ultralytics import YOLO

# Load YOLO model
model = YOLO("data_split/Models/bestyolov8m_fine_tuned.pt")

input_folder = "data_split/validation/images"
output_folder = "Keypoint_Images"

os.makedirs(output_folder, exist_ok=True)

image_exts = (".jpg", ".jpeg", ".png", ".bmp")

image_files = sorted([
    f for f in os.listdir(input_folder)
    if f.lower().endswith(image_exts)
])

count = 1

for img_name in image_files:
    img_path = os.path.join(input_folder, img_name)
    img = cv2.imread(img_path)

    if img is None:
        print(f"Could not read {img_name}")
        continue

    results = model(img)
    boxes = results[0].boxes

    if boxes is None:
        continue

    for box in boxes.xyxy:
        x1, y1, x2, y2 = map(int, box)
        crop = img[y1:y2, x1:x2]

        if crop.size == 0:
            continue

        output_name = f"TORe_BBox_{count:04d}.jpg"
        output_path = os.path.join(output_folder, output_name)

        cv2.imwrite(output_path, crop)
        print(f"Saved: {output_name}")

        count += 1


