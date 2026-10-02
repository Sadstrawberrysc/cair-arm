"""Validated model settings shared by the CLI and single-frame inference."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ModelConfig:
    model_id: str
    class_name: str = 'carotid'
    min_confidence: float = .4
    inference_confidence: float = .4
    margin: int = 0

    def __post_init__(self):
        if not isinstance(self.model_id, str) or not self.model_id.strip():
            raise ValueError('model_id must be nonempty')
        if not isinstance(self.class_name, str) or not self.class_name.strip():
            raise ValueError('class_name must be nonempty')
        for name in ('min_confidence', 'inference_confidence'):
            value = getattr(self, name)
            if (isinstance(value, bool) or not isinstance(value, (int, float)) or
                    not math.isfinite(value) or not 0 <= value <= 1):
                raise ValueError('%s must be in [0, 1]' % name)
        if type(self.margin) is not int or self.margin < 0:
            raise ValueError('margin must be a nonnegative integer')
