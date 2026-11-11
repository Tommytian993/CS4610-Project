import unittest

from study_delivery.delivery_events import (
    encode_status_event,
    format_status_event,
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


if __name__ == "__main__":
    unittest.main()
