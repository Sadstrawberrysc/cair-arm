"""Synthetic SDK frames exercise integration; these are not SDK quality tests."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sdk_depth import SdkDepthProcessor


class Frame:
    def __init__(self, array, stamp=123456, scale=.1, fmt='Y16'):
        self.array = np.asarray(array, dtype=np.uint16)
        self.stamp, self.scale, self.fmt = stamp, scale, fmt

    def get_width(self): return self.array.shape[1]
    def get_height(self): return self.array.shape[0]
    def get_data(self): return self.array
    def get_global_timestamp_us(self): return self.stamp
    def get_depth_scale(self): return self.scale
    def get_format(self): return self.fmt
    def as_depth_frame(self): return self


class Filter:
    def __init__(self, transform):
        self.params = dict(magnitude=1., alpha=.5, radius=1., disp_diff=160.)
        self.transform = transform

    def set_config_value(self, name, value): self.params[name] = float(value)
    def get_config_value(self, name): return self.params[name]
    def enable(self, value): self.enabled = value
    def process(self, frame): return self.transform(frame)


class SdkDepthTests(unittest.TestCase):
    def setUp(self):
        self.calls = []

        def spatial(frame):
            self.calls.append('spatial')
            data = frame.array.copy()
            data[data > 0] += 1
            return Frame(data, frame.stamp, frame.scale, frame.fmt)

        def fill(frame):
            self.calls.append('holes')
            data = frame.array.copy()
            data[data == 0] = 2500
            return Frame(data, frame.stamp, frame.scale, frame.fmt)

        self.spatial, self.holes = Filter(spatial), Filter(fill)
        filters = {'SpatialAdvancedFilter': self.spatial, 'HoleFillingFilter': self.holes}
        self.sdk = SimpleNamespace(
            OBFormat=SimpleNamespace(Y16='Y16'),
            OBHoleFillingMode=SimpleNamespace(FURTHEST=SimpleNamespace(value=2)),
            FilterFactory=SimpleNamespace(create_filter=filters.__getitem__))
        self.processor = SdkDepthProcessor(self.sdk)

    def test_sdk_chain_includes_large_holes_and_keeps_raw_data(self):
        data = np.full((12, 12), 2500, np.uint16)
        data[3:9, 3:9] = 0  # 36-pixel hole, no area cap requested.
        original = data.copy()
        frame = Frame(data)
        result = self.processor.process(frame)
        self.assertEqual(self.calls, ['spatial', 'holes'])
        np.testing.assert_array_equal(frame.array, original)
        np.testing.assert_array_equal(result.raw_m, original*.0001)
        self.assertAlmostEqual(result.filtered_m[0, 0], .2501)
        self.assertAlmostEqual(result.filtered_m[5, 5], .25)
        self.assertEqual(result.stats['raw_invalid_pixels'], 36)
        self.assertEqual(result.stats['filled_pixels'], 36)
        self.assertEqual(result.stats['filtered_invalid_pixels'], 0)
        self.assertIsNone(self.processor.configuration['hole_area_limit'])
        self.assertFalse(self.processor.configuration['temporal_filter'])
        self.assertEqual(self.holes.params['hole_filling_mode'], 2)

    def test_remaining_holes_stay_invalid_instead_of_using_previous_depth(self):
        self.holes.transform = lambda frame: frame
        result = self.processor.process(Frame(np.zeros((8, 8), np.uint16)))
        self.assertTrue(np.all(result.filtered_m == 0))
        self.assertEqual(result.stats['filled_pixels'], 0)

    def test_sdk_metadata_changes_or_empty_output_are_rejected(self):
        for label, transform in (
                ('timestamp', lambda f: Frame(f.array, f.stamp+1)),
                ('scale', lambda f: Frame(f.array, f.stamp, .2)),
                ('dimensions', lambda f: Frame(f.array[:-1], f.stamp)),
                ('format', lambda f: Frame(f.array, f.stamp, fmt='RGB')),
                ('empty', lambda f: None)):
            with self.subTest(label=label):
                self.holes.transform = transform
                with self.assertRaises(ValueError):
                    self.processor.process(Frame(np.full((8, 8), 2500, np.uint16)))

    def test_raw_snapshot_survives_sdk_input_mutation(self):
        def mutate(frame):
            frame.array[:] = 2700
            return frame
        self.spatial.transform = mutate
        result = self.processor.process(Frame(np.zeros((8, 8), np.uint16)))
        self.assertTrue(np.all(result.raw_m == 0))
        self.assertTrue(np.allclose(result.filtered_m, .27))

    def test_sdk_cannot_hide_timestamp_change_by_mutating_input_metadata(self):
        def mutate(frame):
            frame.stamp += 1
            return frame
        self.spatial.transform = mutate
        with self.assertRaisesRegex(ValueError, 'capture timestamp'):
            self.processor.process(Frame(np.full((8, 8), 2500, np.uint16)))

    def test_diagnostics_pair_filled_and_raw_samples_from_the_same_frame(self):
        data = np.full((8, 8), 2500, np.uint16)
        data[3, 3] = 0
        pair = self.processor.process(Frame(data))
        observation = dict(valid=True, pixel=[3, 3], capture_monotonic_ns=1000)
        result = pair.annotate(observation, 1000, .030)
        self.assertEqual(result['depth_processing']['depth_source'], 'sdk_filled')
        self.assertEqual(result['depth_processing']['raw_target_depth_m'], 0)
        self.assertAlmostEqual(result['depth_processing']['filtered_target_depth_m'], .25)
        self.assertIn('raw_neighborhood', result['depth_processing'])
        self.assertEqual(pair.annotate(observation, 2000, .030), observation)
        failed = dict(valid=False, reason='invalid target depth', depth_neighborhood=dict(
            capture_monotonic_ns=1000, target_pixel_subpixel=[3, 3]))
        self.assertIn('depth_processing', pair.annotate(failed, 1000, .030))

    def test_invalid_format_and_scale_do_not_enter_filters(self):
        for frame in (Frame(np.zeros((8, 8)), fmt='RGB'),
                      Frame(np.zeros((8, 8)), scale=0),
                      Frame(np.zeros((8, 8)), scale=float('nan'))):
            with self.assertRaises(ValueError):
                self.processor.process(frame)
        self.assertEqual(self.calls, [])


if __name__ == '__main__':
    unittest.main()
