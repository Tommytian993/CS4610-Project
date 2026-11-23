from dataclasses import dataclass


@dataclass
class DeliveryState:
    phase: str
    recipient: str = ""
    acknowledgment: str = "not_started"
    navigation: str = "not_started"


class DeliveryStateTracker:
    def __init__(self):
        self._requests = {}

    def update(self, event):
        request = event["request"]
        event_state = event["state"]
        recipient = event.get("recipient") or ""

        if event_state == "request_received":
            if request in self._requests:
                raise ValueError(f"Request {request} was already received.")
            self._requests[request] = DeliveryState("request_received")
            return self._requests[request]

        current = self._requests.get(request)
        if event_state == "rejected_busy":
            if current is None:
                self._requests[request] = DeliveryState("rejected_busy")
                return self._requests[request]
            raise ValueError(f"Request {request} is already being tracked.")
        if current is None:
            raise ValueError(f"Request {request} has no recorded request_received state.")

        if event_state == "awaiting_confirmation":
            self._require_phase(request, current, "request_received")
            current.phase = "awaiting_confirmation"
        elif event_state == "confirmed":
            self._require_phase(request, current, "awaiting_confirmation")
            current.phase = "confirmed"
            current.recipient = recipient
        elif event_state == "declined":
            self._require_phase(request, current, "awaiting_confirmation")
            current.phase = "declined"
        elif event_state.startswith("acknowledgment_"):
            self._update_operation(
                request,
                current,
                "acknowledgment",
                event_state[len("acknowledgment_"):],
            )
        elif event_state.startswith("navigation_"):
            self._update_operation(
                request, current, "navigation", event_state[len("navigation_"):]
            )
        else:
            raise ValueError(f"Unsupported delivery event state: {event_state}.")

        if recipient:
            current.recipient = recipient
        return current

    @staticmethod
    def _require_phase(request, current, expected):
        if current.phase != expected:
            raise ValueError(
                f"Request {request} cannot transition from {current.phase} "
                f"to {expected}."
            )

    @staticmethod
    def _update_operation(request, current, operation, outcome):
        if current.phase not in ("confirmed", "in_progress", "arrived", "failed"):
            raise ValueError(
                f"Request {request} cannot start {operation} from {current.phase}."
            )
        if outcome not in ("started", "succeeded", "failed"):
            raise ValueError(f"Unsupported {operation} outcome: {outcome}.")

        previous = getattr(current, operation)
        valid = (
            previous == "not_started" and outcome in ("started", "failed")
        ) or (previous == "started" and outcome in ("succeeded", "failed"))
        if not valid:
            raise ValueError(
                f"Request {request} cannot transition {operation} "
                f"from {previous} to {outcome}."
            )

        setattr(current, operation, outcome)
        if outcome == "started":
            current.phase = "in_progress"
        elif operation == "navigation":
            if outcome == "succeeded":
                current.phase = "arrived"
            else:
                current.phase = "failed"
