import unittest

from study_delivery.delivery_events import (
    encode_status_event,
    format_status_event,
    parse_status_event,
)
from study_delivery.delivery_states import DeliveryStateTracker
from study_delivery.side_navigation import (
    encode_navigation_request,
    parse_navigation_request,
    pose_for_side,
    recipient_side,
)


class DeliveryEventsTest(unittest.TestCase):
    def test_status_event_formats_progress_and_context(self):
        payload = encode_status_event(
            "hand_raise:2",
            "acknowledgment_failed",
            "Window table, seat 4",
            "Stretch action server is unavailable.",
        )
        status = format_status_event(payload)
        self.assertIn("[ACKNOWLEDGMENT_FAILED]", status)
        self.assertIn("hand_raise:2", status)
        self.assertIn("Window table, seat 4", status)
        self.assertIn("Stretch action server is unavailable.", status)

    def test_malformed_status_event_is_rejected(self):
        with self.assertRaises(ValueError):
            format_status_event('{"request": "hand_raise:2"}')

    def test_delivery_state_tracks_confirmation_and_navigation(self):
        tracker = DeliveryStateTracker()
        events = (
            ("request_received", None),
            ("awaiting_confirmation", None),
            ("confirmed", "Left window table, seat 4"),
            ("navigation_started", "Left window table, seat 4"),
            ("navigation_succeeded", "Left window table, seat 4"),
        )
        for state, recipient in events:
            delivery = tracker.update({
                "request": "hand_raise:2",
                "state": state,
                "recipient": recipient,
            })

        self.assertEqual(delivery.phase, "arrived")
        self.assertEqual(delivery.navigation, "succeeded")
        self.assertEqual(delivery.acknowledgment, "not_started")

    def test_acknowledgment_does_not_mark_delivery_complete(self):
        tracker = DeliveryStateTracker()
        for state in (
            "request_received",
            "awaiting_confirmation",
            "confirmed",
            "acknowledgment_started",
            "acknowledgment_succeeded",
        ):
            delivery = tracker.update({
                "request": "hand_raise:2",
                "state": state,
            })

        self.assertEqual(delivery.phase, "in_progress")
        self.assertEqual(delivery.acknowledgment, "succeeded")

    def test_delivery_state_rejects_invalid_transitions(self):
        tracker = DeliveryStateTracker()
        tracker.update({"request": "hand_raise:2", "state": "request_received"})
        with self.assertRaises(ValueError):
            tracker.update({"request": "hand_raise:2", "state": "confirmed"})

    def test_status_event_parsing_returns_validated_fields(self):
        payload = encode_status_event("hand_raise:2", "confirmed", "left seat")
        self.assertEqual(parse_status_event(payload)["state"], "confirmed")

    def test_navigation_request_round_trips(self):
        payload = encode_navigation_request(
            "hand_raise:2", "Left window table, seat 4", "left"
        )
        request = parse_navigation_request(payload)
        self.assertEqual(request["request"], "hand_raise:2")
        self.assertEqual(request["side"], "left")
        self.assertEqual(recipient_side(request["recipient"]), "left")

    def test_navigation_side_selects_destination(self):
        left = pose_for_side("left", (-1.0, 2.0, 1.57), (1.0, -2.0, -1.57))
        right = pose_for_side("right", (-1.0, 2.0, 1.57), (1.0, -2.0, -1.57))
        self.assertEqual(left, (-1.0, 2.0, 1.57))
        self.assertEqual(right, (1.0, -2.0, -1.57))

    def test_recipient_requires_a_known_side(self):
        with self.assertRaises(ValueError):
            recipient_side("Window table, seat 4")


if __name__ == "__main__":
    unittest.main()
