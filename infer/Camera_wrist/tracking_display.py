"""Display state never authorizes a Robot observation or motion request."""
from model_selection import observation_is_fresh


def point_display(visual, observation, session, now_ns):
    """Only current 2D matches are shown; held/stale/invalid 3D stays yellow."""
    stamp = visual.get('capture_monotonic_ns')
    if (not visual.get('image_valid') or not isinstance(stamp, int)
            or not 0 <= now_ns-stamp <= 200_000_000):
        return None
    state = session.state or {}
    state_stamp = state.get('timestamp_monotonic_ns', 0)
    robot_following = (
        session.following and bool(session.target)
        and 0 <= now_ns-state_stamp <= 200_000_000
        and state.get('follow_state') == 'following'
        and state.get('target_id') == session.target
        and state.get('request_producer_id') == session.producer
        and state.get('request_sequence', 0) >= session.last_request_sequence
        and str(state.get('control_state', '')).lower() != 'fault')
    green = (robot_following and observation_is_fresh(observation, now_ns)
             and observation.get('capture_monotonic_ns') == stamp)
    return dict(pixel=visual['pixel'], color=(0, 255, 0) if green else (0, 255, 255),
                state='3D following' if green else '2D only / Robot paused')
