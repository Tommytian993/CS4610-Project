import json


SIDES = ("left", "right")


def recipient_side(recipient):
    text = recipient.lower()
    if "left" in text:
        return "left"
    if "right" in text:
        return "right"
    raise ValueError("Recipient must include a side: left or right.")


def encode_navigation_request(request, recipient, side):
    normalized = side.lower().strip()
    if normalized not in SIDES:
        raise ValueError("Navigation side must be left or right.")
    return json.dumps({
        "request": request,
        "recipient": recipient,
        "side": normalized,
    })


def parse_navigation_request(payload):
    try:
        value = json.loads(payload)
    except json.JSONDecodeError:
        normalized = payload.lower().strip()
        if normalized not in SIDES:
            raise ValueError("Navigation request must include a left or right side.")
        return {"request": "", "recipient": "", "side": normalized}

    if not isinstance(value, dict):
        raise ValueError("Navigation request must be a JSON object.")

    side = value.get("side")
    if not isinstance(side, str) or side.lower().strip() not in SIDES:
        raise ValueError("Navigation request must include side left or right.")

    request = value.get("request", "")
    recipient = value.get("recipient", "")
    if not isinstance(request, str) or not isinstance(recipient, str):
        raise ValueError("Navigation request fields must be strings.")

    return {
        "request": request,
        "recipient": recipient,
        "side": side.lower().strip(),
    }


def pose_for_side(side, left_pose=(2.0, 0.0, 0.0),
                  right_pose=(2.0, 0.0, 0.0)):
    normalized = side.lower().strip()
    if normalized == "left":
        return tuple(left_pose)
    if normalized == "right":
        return tuple(right_pose)
    raise ValueError("Navigation side must be left or right.")
