#!/usr/bin/env python3
"""Record Redis or standalone KWR57 data without a GUI; Ctrl+C saves and plots."""

import sys

if __package__:
    from .main import main
else:
    from main import main

if __name__ == "__main__":
    raise SystemExit(main(["--headless", *sys.argv[1:]]))
