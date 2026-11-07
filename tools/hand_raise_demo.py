import cv2
from ultralytics import YOLO


def main():
    model = YOLO("yolo11n-pose.pt")  # human pose model
    camera = cv2.VideoCapture(0)  # 0 for default camer
    try:
        if not camera.isOpened():
            raise RuntimeError("Cannot open camera. Please check permissions.")

        while True:
            ok, frame = camera.read()  # read one frame
            if not ok:
                print("Cannot read camera frame. Exiting.")
                break

            # detect pose key points
            result = model.predict(
                frame, verbose=False, save=False, save_txt=False, save_crop=False
            )[0]
            annotated_frame = result.plot()  
            cv2.imshow("Hand Raise Demo", annotated_frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break  
    finally:
        camera.release() 
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
