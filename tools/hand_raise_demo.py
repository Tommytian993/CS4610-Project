import cv2


def main():
    camera = cv2.VideoCapture(0)  # 0 for default camera
    try:
        if not camera.isOpened():
            raise RuntimeError("Cannot turn on camera, please check default settings")

        while True:
            ok, frame = camera.read()  # read one frame
            if not ok:
                print("Cannot read camera image, exit")
                break

            cv2.imshow("Hand Raise Demo", frame)  
            if cv2.waitKey(1) & 0xFF == ord("q"):  
    finally:
        camera.release() 
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
