import json


def encode_status_event(request, state, recipient=None, detail=None):
    event = {"request": request, "state": state}
    if recipient is not None:
        event["recipient"] = recipient
    if detail is not None:
        event["detail"] = detail
    return json.dumps(event)


def format_status_event(payload):
    try:
        event = json.loads(payload)
    except json.JSONDecodeError as error:
        raise ValueError("Status event must be valid JSON.") from error

    if not isinstance(event, dict):
        raise ValueError("Status event must be a JSON object.")

    request = event.get("request")
    state = event.get("state")
    recipient = event.get("recipient")
    detail = event.get("detail")
    if not isinstance(request, str) or not request:
        raise ValueError("Status event must include a non-empty request.")
    if not isinstance(state, str) or not state:
        raise ValueError("Status event must include a non-empty state.")
    if recipient is not None and not isinstance(recipient, str):
        raise ValueError("Status event recipient must be a string.")
    if detail is not None and not isinstance(detail, str):
        raise ValueError("Status event detail must be a string.")

    parts = [f"[{state.upper()}] Request {request}"]
    if recipient:
        parts.append(f"recipient: {recipient}")
    if detail:
        parts.append(detail)
    return " | ".join(parts)
