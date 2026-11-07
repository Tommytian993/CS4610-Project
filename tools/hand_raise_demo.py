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

                # xy saves each person's keypoint pixels, we only need the first at index zero
                # turn it into a numpy array of 17 (keypoints) by 2 (x,y coords).
                points = result.keypoints.xy[0].cpu().numpy()

                # confidence about the keypoint's placement
                confidence = result.keypoints.conf

                if confidence is not None:

                    # keypoint confidence array
                    scores = confidence[0].cpu().numpy()

                    # assume no hand raised
                    raised = False

                    # visible sides of arm and wrist, could be 0,1,2
                    visible_arms = 0

                    # frame.shape is the height in pixels,
                    # this is how much wrist must be higher than shoulder for the detection
                    margin = frame.shape[0] * 0.04
                    # COCO key points：left shoulder 5, right shoulder 6, left wrist 9, right wrist 10
                    for shoulder, wrist in ((5, 9), (6, 10)):

                        # for now wrist and shoulders values must be at least 0.5
                        # we could change later if hard to detect
                        if scores[shoulder] >= 0.5 and scores[wrist] >= 0.5:
                            visible_arms += 1

                            # get the y coord (0 is x), 
                            # upper-left is origin, so lower in the image higher y
                            if points[wrist, 1] < points[shoulder, 1] - margin:

                                # one side satisfies
                                raised = True
                    
                    # checked both sides, show the detection
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
