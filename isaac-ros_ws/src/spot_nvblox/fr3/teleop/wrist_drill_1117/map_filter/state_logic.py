"""Pure state mapping for the map target-selection gate."""


def resolve_map_state(target, tracking_state, tracking_age_s, tracking_timeout_s=3.0):
    target = (target or "").strip()
    if target.upper() in ("", "NONE", "NO_TARGET", "CLEAR"):
        return "NO_TARGET"
    if tracking_age_s is None or tracking_age_s < 0 or tracking_age_s > tracking_timeout_s:
        return f"LOST:{target}"
    state = (tracking_state or "").strip().upper()
    if state == "TRACKING":
        return f"ACTIVE:{target}"
    if state == "LOST":
        return f"LOST:{target}"
    return f"UNKNOWN:{state or 'MISSING'}"
