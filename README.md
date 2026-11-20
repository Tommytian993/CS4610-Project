# CS4610 — Stretch Delivery Assistant

## Project

Our team will create a robot assistant that transfers small objects, between members of study groups and individual students, seated along the window-facing tables on the first floor of West Village H (WVH). Students studying together may sit several seats apart. Sharing an item can require someone to leave a high chair and walk to the other end of the table. Stretch would help by picking up an item from one student's designated pickup area and delivering it to another student's designated drop-off area.

For our initial implementation, we will register students via their seat locations. A student can raise a hand to request service, then select an item and a recipient through a simple interface. Stretch will confirm the request, collect the item, navigate to the recipient, and place it on the table.

## Progress reporting

The `/delivery/status` topic reports `request_received`, `awaiting_confirmation`, `confirmed` or `declined`, `acknowledgment_started`, and `acknowledgment_succeeded` or `acknowledgment_failed` (as well as `rejected_busy`). Run the `status` node to view these events. The acknowledgment events are emitted when `stretch_ack` receives a request on `/delivery/request_accepted`. They report only the head/gripper acknowledgment motion, not a completed object delivery; pickup, navigation, and placement remain project goals.

## Run the MuJoCo demo

Open four terminals in the project root. Keep each one running as you start the next.

1. **Start MuJoCo.** Wait until Stretch appears in the viewer.

   ```bash
   bash ../stretch-humble-sim/start.sh
   ```

2. **Build and start the motion node.** Wait for `Stretch acknowledgment node started.`

   ```bash
   sudo docker exec cs4610-stretch-sim mkdir -p /tmp/cs4610_ws/src/study_delivery &&
   sudo docker cp src/study_delivery/. cs4610-stretch-sim:/tmp/cs4610_ws/src/study_delivery/ &&
   sudo docker exec -it cs4610-stretch-sim bash -c '
     source /opt/ros/humble/setup.bash &&
     source /sim_ws/install/local_setup.bash &&
     cd /tmp/cs4610_ws &&
     colcon build --packages-select study_delivery &&
     source install/local_setup.bash &&
     ros2 run study_delivery stretch_ack
   '
   ```

3. **Start the dialogue node.** Wait for `[IDLE] Waiting for a hand-raise request.`

   ```bash
   sudo docker exec -it cs4610-stretch-sim bash -c '
     source /opt/ros/humble/setup.bash &&
     source /sim_ws/install/local_setup.bash &&
     source /tmp/cs4610_ws/install/local_setup.bash &&
     ros2 run study_delivery delivery_dialogue
   '
   ```

4. **Start the webcam.** Raise one hand for about one second to trigger the demo.

   ```bash
   source .venv/bin/activate &&
   sudo -v &&
   python tools/hand_raise_demo.py --sim
   ```

## Test gesture-controlled pickup on Stretch

The hardware test uses two gestures. Raise one hand for about one second to
move to the pickup-ready pose. Once the driver reports success, hold a thumbs-up
for about one second to close the gripper. Closing commands only the left gripper
joint. The controller handles one completed attempt per launch. This test ends
at gripper closure.

### Build and source the workspace

Run on the Stretch Ubuntu machine with ROS 2 Humble and the Stretch workspace
installed. The hand-raise node uses Ultralytics and the local `yolo11n-pose.pt`.
The thumbs-up node uses MediaPipe in the `stretch-gesture` virtual environment
and the model at `models/gesture_recognizer.task`. Both previews need a desktop
session.

```bash
cd ~/Desktop/CS4610-Project
source ~/ament_ws/install/setup.bash
colcon build --base-paths src --packages-select study_delivery --symlink-install
source install/setup.bash
```

Run the following setup in **each new terminal** before running a command below.

```bash
cd ~/Desktop/CS4610-Project
source ~/ament_ws/install/setup.bash
source install/setup.bash
```

Stop any previous `pickup_controller` with Ctrl+C in its terminal before
starting another instance. For this standalone hardware test, keep the legacy
`delivery_dialogue` and `stretch_ack` nodes stopped. They implement a separate
request and motion workflow.

### Start the camera and gesture detectors

Run each command in its own sourced terminal and leave it running.

**RealSense camera**

```bash
ros2 launch stretch_core d435i_low_resolution.launch.py
```

**Hand raise**

```bash
ros2 run study_delivery hand_raise --ros-args -p image_topic:=/camera/color/image_raw
```

**Thumbs-up**

```bash
ros2 run --prefix "$HOME/.venvs/stretch-gesture/bin/python" \
  study_delivery handover_gesture --ros-args \
  -p image_topic:=/camera/color/image_raw/compressed \
  -p model_path:="$PWD/models/gesture_recognizer.task" \
  -p rotate_clockwise:=true -p show_preview:=true -p thumb_up_score:=0.55
```

The preview shows the current recognition score. The default threshold is
`0.75`. The `0.55` override above was used in the verified lab setup.
Keep the gesture above the selected threshold until the preview shows
`Detected`. Lower your hand before repeating the gesture.

### Dry run

Start the two-stage controller with motion disabled. The camera and both
detectors are sufficient for this test.

```bash
ros2 run study_delivery pickup_controller --ros-args \
  -p hand_raise_mode:=true -p enable_motion:=false
```

Raise your hand and wait for `DRY RUN: Hand raise received; WOULD MOVE TO READY`.
Then give a thumbs-up and check for `DRY RUN: Thumb_Up received; WOULD CLOSE
GRIPPER`. Stop the dry-run controller with Ctrl+C before starting the live one.

### Live test

1. On each robot restart, complete homing with the TA before starting the ROS
   driver. Keep the area clear and the emergency stop within reach. If a ROS
   driver already owns the hardware, stop it before running the standalone
   homing command. Homing moves the robot.

   ```bash
   stretch_robot_home.py
   ```

2. Start the driver in a sourced terminal and leave it running.

   ```bash
   ros2 launch stretch_core stretch_driver.launch.py
   ```

3. In another sourced terminal, verify `data: true` for homing and
   `Action servers: 1` for the trajectory action.

   ```bash
   ros2 topic echo /is_homed --once
   ros2 action info /stretch_controller/follow_joint_trajectory
   ```

4. Have the TA verify the workspace, ready pose, and closed gripper target.
   To record a ready pose, have the TA position the lift at waist height,
   point the wrist forward, and open the gripper. Read the corresponding
   values in the `name` and `position` arrays.

   ```bash
   ros2 topic echo /stretch/joint_states --once
   ```

5. Start the live controller. The values below are the tested pose for the
   lab setup on Stretch SE3-3061. Replace them with the TA-confirmed values
   if the setup changes. `ready_position` follows the order of
   `ready_joints`. Lift is in meters and the remaining positions are in radians.
   Wrist extension is omitted, so the arm keeps its current extension.

   ```bash
   ros2 run study_delivery pickup_controller --ros-args \
     -p hand_raise_mode:=true -p enable_motion:=true -p duration_seconds:=3.0 \
     -p 'ready_joints:=[joint_lift,joint_wrist_yaw,joint_wrist_pitch,joint_wrist_roll,joint_gripper_finger_left]' \
     -p 'ready_position:=[0.79415,1.4854,0.13959,-0.11658,0.64228]' \
     -p close_position:=-0.01
   ```

6. With everyone outside the motion envelope and the TA at the emergency
   stop, raise your hand. Wait for the arm to stop and for `Pickup ready.
   Waiting for Thumb_Up.` Give a thumbs-up and look for `PICKUP CLOSE
   SUCCEEDED.` If the robot moves unexpectedly, use the emergency stop.
   Restart the controller for another attempt.

### Check the ROS graph and logs

```bash
ros2 node list
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/color/image_raw/compressed
ros2 topic echo /delivery/request
ros2 topic echo /delivery/handover_gesture
ros2 action info /stretch_controller/follow_joint_trajectory
ros2 param get /pickup_controller enable_motion
```

Run the continuous commands individually and stop them with Ctrl+C. The
hand-raise detector publishes `hand_raise:<count>` on `/delivery/request`.
The thumbs-up detector publishes `Thumb_Up` on
`/delivery/handover_gesture`. A thumbs-up received before the ready action
succeeds is ignored. If a command is rejected or times out, check the
controller and driver terminals or their logs under `~/.ros/log`.
