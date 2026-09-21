"""Explicit Gemini 305 acquisition profile for surfaces around 80 mm away."""
WIDTH, HEIGHT, FPS = 640, 480, 30


def configure_close_range(device, ob):
    presets = device.get_available_preset_list()
    names = [presets.get_name_by_index(i) for i in range(presets.get_count())]
    # Gemini 305 firmware exposes named variants, not a generic Close-Range.
    # Prefer its Default variant; do not silently select High Accuracy.
    matches = []
    for wanted in ('closerangedefault', 'closerange'):
        matches = [name for name in names
                   if name.lower().replace('-', '').replace('_', '').replace(' ', '') == wanted]
        if matches:
            break
    if len(matches) != 1:
        raise ValueError('Unique Close-Range preset unavailable; device presets: %r' % names)
    device.load_preset(matches[0])
    # SDK mode enum: 1 = 128 disparities, 2 = 256 (not integer 256).
    prop = ob.OBPropertyID.OB_PROP_DISP_SEARCH_RANGE_MODE_INT
    device.set_int_property(prop, 2)
    actual = device.get_int_property(prop)
    preset = device.get_current_preset_name()
    if actual != 2 or preset != matches[0]:
        raise ValueError('Close-Range readback failed: preset=%r disparity_mode=%r' % (preset, actual))
    settings = dict(type='camera_configuration', preset=preset,
                    disparity_search_range=256, disparity_mode=actual,
                    width=WIDTH, height=HEIGHT, fps=FPS)
    print('Gemini: %s; disparity=256; requested RGB/depth=%dx%d @ %d FPS' %
          (preset, WIDTH, HEIGHT, FPS))
    return settings


def stream_profiles(pipeline, ob):
    # Fail explicitly if USB bandwidth/firmware cannot provide this combination.
    selected, missing = [], []
    for label, sensor, fmt in (
            ('color', ob.OBSensorType.COLOR_SENSOR, ob.OBFormat.MJPG),
            ('depth', ob.OBSensorType.DEPTH_SENSOR, ob.OBFormat.Y16)):
        profiles = pipeline.get_stream_profile_list(sensor)
        available = [profiles.get_stream_profile_by_index(i).as_video_stream_profile()
                     for i in range(profiles.get_count())]
        matches = [p for p in available if (p.get_width(), p.get_height(), p.get_format(), p.get_fps())
                   == (WIDTH, HEIGHT, fmt, FPS)]
        if matches:
            selected.append(matches[0])
        else:
            modes = ['%dx%d %s @ %d FPS' % (p.get_width(), p.get_height(), p.get_format(), p.get_fps())
                     for p in available]
            missing.append('%s requested %dx%d %s @ %d FPS; available: %s' %
                           (label, WIDTH, HEIGHT, fmt, FPS, '; '.join(modes)))
    if missing:
        raise ValueError('Unsupported camera stream. ' + ' | '.join(missing)
                         + '. Check USB 3 connection/cable/hub; no automatic FPS downgrade.')
    print('RGB/depth stream profiles matched: %dx%d @ %d FPS' % (WIDTH, HEIGHT, FPS))
    return tuple(selected)
