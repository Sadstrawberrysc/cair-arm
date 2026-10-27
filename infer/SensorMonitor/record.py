#!/usr/bin/env python3
"""Record KWR57 without a GUI; Ctrl+C saves and plots the session."""

import sys

if __package__:
    from .main import main
else:
    from main import main

if __name__ == "__main__":
    raise SystemExit(main(["--headless", *sys.argv[1:]]))
