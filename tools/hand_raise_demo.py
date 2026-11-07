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

            # first version lets only detect one person to prevent overlap
            # will show this when not detecting, detecting mutiple or keypoints unclear
            status = "SHOW ONE PERSON AND BOTH WRISTS"
            color = (0, 200, 255) # opencv -> BGR

            # result is this frame's yolo detection, 
            # the first selection checks there are keypoints,
            # second selection check if these belong to the same person.
            # * len(result.keypoints) is number of people not keypoints here.
            if result.keypoints is not None and len(result.keypoints) == 1:
                points = result.keypoints.xy[0].cpu().numpy()
                confidence = result.keypoints.conf
                if confidence is not None:
                    scores = confidence[0].cpu().numpy()
                    raised = False
                    visible_arms = 0
                    margin = frame.shape[0] * 0.04
                    # COCO key points：left shoulder 5, right shoulder 6, left wrist 9, right wrist 10
                    for shoulder, wrist in ((5, 9), (6, 10)):
                        if scores[shoulder] >= 0.5 and scores[wrist] >= 0.5:
                            visible_arms += 1
                            if points[wrist, 1] < points[shoulder, 1] - margin:
                                raised = True
                    if raised:
                        status, color = "HAND RAISED", (0, 255, 0)
                    elif visible_arms == 2:
                        status, color = "HANDS DOWN", (255, 255, 255)

            cv2.putText(annotated_frame, status, (20, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
            cv2.imshow("Hand Raise Demo", annotated_frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break  
    finally:
        camera.release() 
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
