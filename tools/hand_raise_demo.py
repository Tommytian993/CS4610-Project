import argparse
import subprocess
import time 
from threading import Thread

import cv2
from ultralytics import YOLO


def publish_to_sim(request_count):
    command = [
        "sudo", "-n", "docker", "exec", "cs4610-stretch-sim", "bash", "-c",
        'source /opt/ros/humble/setup.bash && '
        'exec ros2 topic pub --once -w 1 /delivery/request std_msgs/msg/String "$1"',
        "--", f"{{data: 'hand_raise:{request_count}'}}",
    ]
    try:
        subprocess.run(command, check=True, timeout=8)
    except (OSError, subprocess.SubprocessError) as error:
        print(f"Could not publish simulation request: {error}", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", action="store_true", help="Send hand raises to the simulator")
    args = parser.parse_args()
    model = YOLO("yolo11n-pose.pt")  # human pose model
    camera = cv2.VideoCapture(0)  # 0 for default camera
    raised_since = None  
    lowered_since = None 
    triggered = False  # locked in until both hands are down
    request_count = 0  
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
            observation = None  # None for unclear (not hands down)
            now = time.monotonic()

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
                        observation = True  # at least one hand raised
                    elif visible_arms == 2:
                        observation = False  # clear on both sides, but not raised

            # turn frame detection into one request
            if observation is True:
                lowered_since = None
                if triggered:
                    status, color = "REQUEST SENT - LOWER BOTH HANDS", (0, 255, 0)
                else:
                    if raised_since is None:
                        raised_since = now
                    elapsed = now - raised_since
                    status = f"RAISING: {min(elapsed, 1.0):.1f}/1.0s"
                    if elapsed >= 1.0:  # need to maintain raised for at least one second to trigger
                        triggered = True
                        request_count += 1
                        if args.sim:
                            Thread(
                                target=publish_to_sim, args=(request_count,), daemon=True
                            ).start()
                        status, color = "REQUEST SENT", (0, 255, 0)
                        print(f"[Request {request_count}] Would you like me to deliver an item?",
                              flush=True)
            elif observation is False:
                raised_since = None
                if lowered_since is None:
                    lowered_since = now
                if now - lowered_since >= 0.5:  # release after both hands lowered for more than a second
                    triggered = False
                status = "LOWER BOTH HANDS TO RESET" if triggered else "IDLE - RAISE A HAND"
            else:
                # stop counting when unclear, no one or multiple people in sight
                raised_since = None
                lowered_since = None

            cv2.putText(annotated_frame, f"Requests: {request_count}", (20, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
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
