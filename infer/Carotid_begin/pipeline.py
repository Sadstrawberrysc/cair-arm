"""Single-frame detection: model -> native boxes -> unique center -> reply.

No credentials, sockets, GPU imports, depth geometry or Robot commands live here.
The model is injected, so the same path can be exercised with an offline fake.
"""
import time

from .detection import response_to_boxes, scan_start_from_boxes


class DetectionPipeline:
    def __init__(self, model, config):
        self.model = model
        self.config = config

    def process(self, image, capture_monotonic_ns):
        """Preserve source identity even when detection cannot select a point.

        Expected model-data errors reject only this frame. Runtime/GPU failures
        propagate to the worker so the caller sees a broken worker connection.
        """
        response = dict(valid=False, capture_monotonic_ns=capture_monotonic_ns)
        started_ns = time.monotonic_ns()
        try:
            result = self.model.infer(image, confidence=self.config.inference_confidence)
            inference_ms = (time.monotonic_ns() - started_ns) / 1e6
            raw = response_to_boxes(result)
            # Preview boxes remain available even if center selection is ambiguous.
            response.update(inference_ms=inference_ms, detections=raw['predictions'],
                            image_size=[image.shape[1], image.shape[0]])
            point = scan_start_from_boxes(
                raw, image.shape[1], image.shape[0],
                class_name=self.config.class_name,
                min_confidence=self.config.min_confidence, margin=self.config.margin)
        except (ValueError, TypeError, KeyError) as error:
            response['reason'] = str(error)
        else:
            response.update(valid=True, prediction=point)
        return response
