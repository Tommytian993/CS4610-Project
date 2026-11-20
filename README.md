# CS4610 — Stretch Delivery Assistant

## Project

Our team will create a robot assistant that transfers small objects, between members of study groups and individual students, seated along the window-facing tables on the first floor of West Village H (WVH). Students studying together may sit several seats apart. Sharing an item can require someone to leave a high chair and walk to the other end of the table. Stretch would help by picking up an item from one student's designated pickup area and delivering it to another student's designated drop-off area.

For our initial implementation, we will register students via their seat locations. A student can raise a hand to request service, then select an item and a recipient through a simple interface. Stretch will confirm the request, collect the item, navigate to the recipient, and place it on the table.

## Progress reporting

The `/delivery/status` topic reports `request_received`, `awaiting_confirmation`, `confirmed` or `declined`, `acknowledgment_started`, and `acknowledgment_succeeded` or `acknowledgment_failed` (as well as `rejected_busy`). Run the `status` node to view events and the tracked delivery state: `request_received`, `awaiting_confirmation`, `confirmed`, `declined`, `in_progress`, `arrived`, `failed`, or `rejected_busy`. Acknowledgment and navigation outcomes are tracked separately because they can run independently. `arrived` means Nav2 reached its goal; it does not mean the item was handed over. Pickup, handover confirmation, and placement remain project goals.

## Side navigation

Run `ros2 run study_delivery side_navigation` to send confirmed deliveries to
either the left or right side of the room. The recipient description must
include `left` or `right`, for example `Left window table, seat 4`.

The node subscribes to `/delivery/navigation_request` and uses Nav2's
`/navigate_to_pose` action. Configure the destination poses with ROS parameters:

```bash
ros2 run study_delivery side_navigation \
  --ros-args \
  -p left_pose:=[2.0,0.0,0.0] \
  -p right_pose:=[-2.0,0.0,3.14]
```

Navigation progress is published to `/delivery/status` as
`navigation_started`, `navigation_succeeded`, or `navigation_failed`.
