"""Convert a Roboflow object-detection box into the initial scan pixel."""
import math


def scan_start_from_boxes(response, width, height, *, class_name=None,
                          min_confidence=0.4, margin=0):
    """Return the center pixel of one unambiguous detection.

    Roboflow detection coordinates are expected as pixel center x/y and box
    width/height in the same original image supplied by the caller.
    """
    if not isinstance(response, dict):
        raise ValueError("prediction response must be an object")
    if not isinstance(width, int) or not isinstance(height, int) or width <= 0 or height <= 0:
        raise ValueError("invalid original image size")
    if not math.isfinite(min_confidence) or not 0 <= min_confidence <= 1:
        raise ValueError("invalid confidence threshold")
    if not isinstance(margin, int) or margin < 0:
        raise ValueError("invalid tracking margin")
    image = response.get("image")
    if not isinstance(image, dict) or (image.get("width"), image.get("height")) != (width, height):
        raise ValueError("response image dimensions differ from original RGB; coordinate mapping required")
    predictions = response.get("predictions")
    if not isinstance(predictions, list):
        raise ValueError("response lacks object-detection predictions")
    candidates = []
    for item in predictions:
        if not isinstance(item, dict):
            raise ValueError("invalid detection entry")
        if class_name is not None and item.get("class") != class_name:
            continue
        try:
            x, y, box_width, box_height, confidence = (
                float(item[key]) for key in ("x", "y", "width", "height", "confidence"))
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError("invalid detection coordinates or confidence") from error
        if not all(math.isfinite(v) for v in (x, y, box_width, box_height, confidence)):
            raise ValueError("non-finite detection")
        if box_width <= 0 or box_height <= 0 or not 0 <= confidence <= 1:
            raise ValueError("invalid detection box or confidence")
        if confidence < min_confidence:
            continue
        left, top = x-box_width/2, y-box_height/2
        right, bottom = x+box_width/2, y+box_height/2
        if left < 0 or top < 0 or right > width or bottom > height:
            raise ValueError("detection box extends outside original RGB")
        pixel = (x, y)
        if not (margin <= pixel[0] <= width-margin and
                margin <= pixel[1] <= height-margin):
            raise ValueError(
                "box center outside tracking margin: "
                "point=(%.1f, %.1f), image=%dx%d, margin=%d px, "
                "allowed x=[%d, %d], y=[%d, %d]" %
                (pixel[0], pixel[1], width, height, margin,
                 margin, width-margin, margin, height-margin))
        candidates.append(dict(scan_start=[float(pixel[0]), float(pixel[1])],
                               confidence=confidence, box_xyxy=[left, top, right, bottom],
                               class_name=item.get("class")))
    if len(candidates) != 1:
        raise ValueError("expected exactly one valid detection, got %d" % len(candidates))
    return candidates[0]


def response_to_boxes(response):
    """Keep the native model's boxes in original-image pixel coordinates."""
    if isinstance(response, list):
        if len(response) != 1:
            raise ValueError("expected one image response")
        response = response[0]
    if not hasattr(response, 'image') or not hasattr(response, 'predictions'):
        raise ValueError('invalid native detection response')
    image = response.image
    try:
        return _native_boxes(response, image)
    except (AttributeError, TypeError, OverflowError) as error:
        raise ValueError('invalid native detection fields') from error


def _native_boxes(response, image):
    raw = {"image": {"width": int(image.width), "height": int(image.height)},
           "predictions": [
               {"x": item.x, "y": item.y, "width": item.width,
                "height": item.height, "confidence": item.confidence,
                "class": item.class_name}
               for item in response.predictions]}
    for item in raw['predictions']:
        for key in ('x', 'y', 'width', 'height', 'confidence'):
            item[key] = float(item[key])
            if not math.isfinite(item[key]):
                raise ValueError('non-finite detection')
        if not isinstance(item['class'], str):
            raise ValueError('invalid detection class')
    return raw
