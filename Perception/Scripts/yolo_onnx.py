from ultralytics import YOLO

model = YOLO("../Model/keypoints_best.pt")
model.export(format="onnx")