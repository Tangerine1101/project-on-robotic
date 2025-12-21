import torch
from ultralytics import YOLO

print(f"CUDA Available: {torch.cuda.is_available()}")
if torch.cuda.is_available():
    print(f"GPU Name: {torch.cuda.get_device_name(0)}")
else:
    print("WARNING: Running on CPU")

# Test load model
#model = YOLO("yolo11n.pt")
#print(f"Model device: {model.device}")