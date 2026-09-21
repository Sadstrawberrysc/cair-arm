#!/usr/bin/env python3
"""Export the operator's 22-column view without altering the raw runtime log."""
import argparse
import csv
from pathlib import Path

FIELDS = (['cycle', 'state'] + ['actual_j%d_rad' % i for i in range(1, 8)]
          + ['actual_x_m', 'actual_y_m', 'actual_z_m',
             'actual_rx_rad', 'actual_ry_rad', 'actual_rz_rad',
             'desired_x_m', 'desired_y_m', 'desired_z_m',
             'desired_rx_rad', 'desired_ry_rad', 'desired_rz_rad',
             'robot_model_position_error_mm'])


def export(source):
    destination = source.with_name('runtime_simple.csv')
    temporary = destination.with_suffix('.csv.tmp')
    try:
        with source.open(newline='') as src:
            reader = csv.DictReader(src)
            missing = set(FIELDS) - set(reader.fieldnames or [])
            if missing:
                raise ValueError('Missing runtime columns: ' + ', '.join(sorted(missing)))
            with temporary.open('w', newline='') as dst:
                writer = csv.DictWriter(dst, fieldnames=FIELDS, extrasaction='ignore')
                writer.writeheader()
                for row in reader:
                    if any(row.get(key) is None for key in FIELDS):
                        raise ValueError('Incomplete runtime row')
                    writer.writerow(row)
        temporary.replace(destination)
    finally:
        if temporary.exists():
            temporary.unlink()
    return destination


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    print(export(parser.parse_args().source))
