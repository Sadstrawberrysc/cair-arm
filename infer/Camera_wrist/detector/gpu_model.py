"""Local ONNX Runtime backend for fixed-size RF-DETR object detection bundles."""

import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from PIL import Image

from .config import DEFAULT_LOCAL_MODEL_ROOT


CUDA_PROVIDER = 'CUDAExecutionProvider'


class LocalRfDetr:
    """Run the RGB stretch pipeline used by the wrist RF-DETR export."""

    def __init__(self, session, config, class_names):
        network = config['network_input']
        if (set(config.get('image_pre_processing', {})) - {'auto-orient'}
                or network['color_mode'] != 'rgb' or network['resize_mode'] != 'stretch'
                or network['input_channels'] != 3 or network['scaling_factor'] != 255
                or network['dynamic_spatial_size_supported']):
            raise ValueError('expected fixed-size RGB RF-DETR with stretch preprocessing')
        size = network['training_input_size']
        self.size = (size['width'], size['height'])
        if any(type(value) is not int or value <= 0 for value in self.size):
            raise ValueError('invalid model input dimensions')
        normalization = np.asarray(network['normalization'], dtype=np.float32)
        if (normalization.shape != (2, 3) or not np.isfinite(normalization).all()
                or (normalization[1] <= 0).any()):
            raise ValueError('invalid RGB normalization values')
        self.mean, self.std = normalization[:, :, None, None]
        if not class_names or any(not name for name in class_names) or len(set(class_names)) != len(class_names):
            raise ValueError('invalid model class names')
        self.class_names = tuple(class_names)
        excluded = set()
        for operation in config.get('class_names_operations', []):
            if operation['type'] != 'class_name_removal' or operation['class_name'] not in class_names:
                raise ValueError('unsupported model class mapping')
            excluded.add(operation['class_name'])
        self.keep_classes = np.asarray([name not in excluded for name in class_names])
        if not self.keep_classes.any():
            raise ValueError('model has no selectable classes')
        inputs = session.get_inputs()
        if (len(inputs) != 1 or inputs[0].type != 'tensor(float)'
                or inputs[0].shape != [1, 3, self.size[1], self.size[0]]):
            raise ValueError('ONNX input does not match model configuration')
        self.input_name = inputs[0].name
        outputs = {out.name: out for out in session.get_outputs()}
        boxes, logits = outputs['dets'], outputs['labels']
        if (boxes.type != 'tensor(float)' or logits.type != 'tensor(float)'
                or len(boxes.shape) != 3 or boxes.shape[0] != 1
                or type(boxes.shape[1]) is not int or boxes.shape[1] <= 0
                or boxes.shape[2] != 4 or logits.shape != [1, boxes.shape[1], len(class_names)]):
            raise ValueError('ONNX outputs do not match RF-DETR boxes and class table')
        self.session = session

    def infer(self, image, *, confidence, max_detections):
        height, width = image.shape[:2]
        resized = Image.fromarray(image).resize(self.size, Image.Resampling.BILINEAR)
        rgb = np.asarray(resized)[:, :, ::-1]
        tensor = np.ascontiguousarray(rgb.transpose(2, 0, 1)[None], dtype=np.float32)
        tensor /= 255.0
        tensor -= self.mean
        tensor /= self.std
        boxes, logits = self.session.run(['dets', 'labels'], {self.input_name: tensor})
        if not np.isfinite(boxes).all() or not np.isfinite(logits).all():
            raise RuntimeError('RF-DETR returned non-finite output')

        # Match the export's query budget before removing the training background class.
        logits = logits[0]
        scores = 1.0 / (1.0 + np.exp(-np.clip(logits.ravel(), -88.0, 88.0)))
        top = np.argsort(-scores, kind='stable')[:logits.shape[0]]
        queries, labels = np.divmod(top, logits.shape[1])
        keep = self.keep_classes[labels] & (scores[top] > confidence)
        bounds = np.asarray([width, height, width, height], dtype=np.float32)
        model_bounds = np.asarray(self.size * 2, dtype=np.float32)
        predictions = []
        for query, label, score in zip(queries[keep][:max_detections],
                                       labels[keep][:max_detections],
                                       scores[top][keep][:max_detections]):
            center, extent = np.split(boxes[0, query], 2)
            half = np.maximum(extent, 0) * 0.5
            box = np.concatenate((center - half, center + half))
            box *= model_bounds
            box /= model_bounds / bounds
            x0, y0, x1, y1 = np.rint(np.clip(box, 0, bounds)).tolist()
            predictions.append(SimpleNamespace(
                x=(x0 + x1) / 2, y=(y0 + y1) / 2, width=x1 - x0, height=y1 - y0,
                confidence=float(score), class_name=self.class_names[label]))
        return SimpleNamespace(image=SimpleNamespace(width=width, height=height),
                               predictions=predictions)


def load_cuda_model(model_id, model_root=DEFAULT_LOCAL_MODEL_ROOT):
    root = Path(model_root).expanduser().resolve()
    directory = (root / model_id).resolve()
    if root not in directory.parents or Path(model_id).is_absolute() or '..' in Path(model_id).parts:
        raise ValueError('model ID must identify a bundle under the local model root')
    try:
        manifest = json.loads((directory / 'model_config.json').read_text())
        if (manifest['canonical_model_id'] != model_id
                or manifest['model_architecture'] != 'rfdetr'
                or manifest['task_type'] != 'object-detection'
                or manifest['backend_type'] != 'onnx'):
            raise ValueError('bundle must contain the requested RF-DETR detection ONNX model')
        config = json.loads((directory / 'inference_config.json').read_text())
        class_names = (directory / 'class_names.txt').read_text().splitlines()
        weights = directory / 'weights.onnx'
        if not weights.is_file():
            raise FileNotFoundError(weights)
    except (OSError, ValueError, KeyError, TypeError) as error:
        raise ValueError(f'invalid local model bundle {directory}: {error}') from error

    import onnxruntime as ort

    ort.disable_telemetry_events()
    if CUDA_PROVIDER not in ort.get_available_providers():
        raise RuntimeError('CUDAExecutionProvider unavailable in the model environment')
    session = ort.InferenceSession(str(weights), providers=[CUDA_PROVIDER, 'CPUExecutionProvider'])
    providers = session.get_providers()
    if not providers or providers[0] != CUDA_PROVIDER:
        raise RuntimeError(f'local ONNX session did not activate CUDAExecutionProvider: {providers}')
    session.disable_fallback()
    try:
        model = LocalRfDetr(session, config, class_names)
    except (ValueError, KeyError, TypeError) as error:
        raise ValueError(f'invalid local model bundle {directory}: {error}') from error
    print(f'Local RF-DETR ONNX: {directory} ({", ".join(providers)})', flush=True)
    return model
