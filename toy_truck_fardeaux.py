"""YOLO training example adapted from the supplied Colab export."""
import os
from roboflow import Roboflow
from ultralytics import YOLO

def main():
    rf = Roboflow(api_key=os.environ["ROBOFLOW_API_KEY"])
    project = rf.workspace("hibara").project("my-first-project-kjp9a")
    version = project.version(4)
    dataset = version.download("yolov8")
    model = YOLO("yolov8n.pt")
    model.train(data=os.path.join(dataset.location, "data.yaml"), epochs=50, imgsz=416, batch=8)

if __name__ == "__main__":
    main()
