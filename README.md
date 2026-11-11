# CS4610 — Stretch Delivery Assistant

## Project

Our team will create a robot assistant that transfers small objects, between members of study groups and individual students, seated along the window-facing tables on the first floor of West Village H (WVH). Students studying together may sit several seats apart. Sharing an item can require someone to leave a high chair and walk to the other end of the table. Stretch would help by picking up an item from one student's designated pickup area and delivering it to another student's designated drop-off area.

For our initial implementation, we will register students via their seat locations. A student can raise a hand to request service, then select an item and a recipient through a simple interface. Stretch will confirm the request, collect the item, navigate to the recipient, and place it on the table.

## Progress reporting

The `/delivery/status` topic reports `request_received`, `awaiting_confirmation`, `confirmed` or `declined`, `acknowledgment_started`, and `acknowledgment_succeeded` or `acknowledgment_failed` (as well as `rejected_busy`). Run the `status` node to view these events. The acknowledgment events are emitted when `stretch_ack` receives a request on `/delivery/request_accepted`. They report only the head/gripper acknowledgment motion, not a completed object delivery; pickup, navigation, and placement remain project goals.
