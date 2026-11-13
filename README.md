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

## Test hand-raise detection on Stretch

With the real robot driver running and its ROS workspace sourced, check that
`ros2 topic hz /camera/color/image_raw` reports frames. From the project root,
with the vision dependencies installed:

```bash
colcon build --base-paths src --packages-select study_delivery --symlink-install &&
source install/setup.bash &&
ros2 run study_delivery hand_raise --ros-args -p image_topic:=/camera/color/image_raw
```

For a vision-only test, keep `stretch_ack` stopped. In another sourced terminal,
run `ros2 topic echo /delivery/request` and raise one hand for about a second.
Omit `image_topic` to use the original webcam mode with `camera_index`.

ros2 action send_goal /stretch_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [joint_gripper_finger_left], points: [{positions: [${GRIPPER_POS:?GRIPPER_POS is not set}], time_from_start: {sec: 2}}]}}"