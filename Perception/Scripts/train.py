import torch
from pathlib import Path
from ultralytics import YOLO


def train_yolov():
    img_size = 640
    batch_size = 16
    epochs = 200
    model_name = "yolov8l.pt"  
    data_config = "RekTNet/config.yaml"
    project = "runs/final"
    name = "pose"

    if not Path(data_config).exists():
        raise FileNotFoundError(f"Dataset config not found: {data_config}")

    model = YOLO(model_name)
    
    print(f"Image size : {img_size}")
    print(f"Batch size : {batch_size}")
    print(f"Epochs     : {epochs}")
    print(f"Device     : {'GPU (CUDA)' if torch.cuda.is_available() else 'CPU'}")
    print("====================================\n")

    

    model.train(
    data=data_config,
    imgsz=640,
    epochs=200,
    batch=16,
    optimizer="AdamW",
    lr0=0.003,
    hsv_h=0.015,
    hsv_s=0.7,
    hsv_v=0.4,
    degrees=5.0,
    translate=0.1,
    scale=0.5,
    shear=2.0,
    mosaic=1.0,
    close_mosaic=10,
    patience=30
    )

    
    return results


if __name__ == "__main__":
    print(f"PyTorch version : {torch.__version__}")
    print(f"CUDA available  : {torch.cuda.is_available()}")
    if torch.cuda.is_available():
        print(f"CUDA version    : {torch.version.cuda}")
        print(f"GPU             : {torch.cuda.get_device_name(0)}")

    train_yolov()
