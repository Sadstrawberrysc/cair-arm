#!/usr/bin/env python3
"""Non-mutating environment, asset, GPU, and RealSense checks for Camera_RT."""

import argparse
import hashlib
import importlib
import importlib.metadata
import os
from pathlib import Path
import subprocess
import sys
from typing import List, Tuple


MODULE_ROOT = Path(__file__).resolve().parents[1]
MANIFEST_PATH = Path(__file__).with_name("assets.sha256")
EXPECTED_PACKAGES = {
    "numpy": "1.22.0",
    "scipy": "1.8.1",
    "Pillow": "9.4.0",
    "opencv-python": "4.8.1.78",
    "open3d": "0.17.0",
    "pyrealsense2": "2.54.2.5684",
    "torch": "1.13.1",
    "torchvision": "0.14.1",
    "smplx": "0.1.28",
    "pytorch3d": "0.7.5",
    "torchgeometry": "0.1.2",
    "tqdm": "4.66.1",
    "yacs": "0.1.8",
}
IMPORT_NAMES = (
    "numpy",
    "scipy",
    "PIL",
    "cv2",
    "open3d",
    "torch",
    "torchvision",
    "smplx",
    "pytorch3d",
    "torchgeometry",
    "tqdm",
    "yacs",
)


class Reporter:
    def __init__(self) -> None:
        self.blocked = 0
        self.warnings = 0

    def emit(self, level: str, name: str, detail: str) -> None:
        print("{:<7} {:<24} {}".format(level, name, detail))
        if level == "BLOCKED":
            self.blocked += 1
        elif level == "WARN":
            self.warnings += 1

    def passed(self, name: str, detail: str) -> None:
        self.emit("PASS", name, detail)

    def warn(self, name: str, detail: str) -> None:
        self.emit("WARN", name, detail)

    def block(self, name: str, detail: str) -> None:
        self.emit("BLOCKED", name, detail)


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_manifest() -> List[Tuple[str, Path]]:
    entries = []
    for line_number, raw_line in enumerate(
        MANIFEST_PATH.read_text(encoding="utf-8").splitlines(), start=1
    ):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split(None, 1)
        if len(parts) != 2:
            raise ValueError("invalid manifest line {}".format(line_number))
        entries.append((parts[0], MODULE_ROOT / parts[1].strip()))
    return entries


def check_static(reporter: Reporter) -> None:
    env_name = os.environ.get("CONDA_DEFAULT_ENV", "")
    if env_name == "camera":
        reporter.passed("conda environment", env_name)
    else:
        reporter.block("conda environment", "expected camera, got {!r}".format(env_name))

    current_python = "{}.{}".format(sys.version_info.major, sys.version_info.minor)
    if current_python == "3.8":
        reporter.passed("python", sys.version.split()[0])
    else:
        reporter.block("python", "expected 3.8, got {}".format(sys.version.split()[0]))

    for package, expected in EXPECTED_PACKAGES.items():
        try:
            actual = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            reporter.block("package " + package, "not installed")
            continue
        if actual == expected:
            reporter.passed("package " + package, actual)
        else:
            reporter.block(
                "package " + package,
                "expected {}, got {}".format(expected, actual),
            )

    for module_name in IMPORT_NAMES:
        try:
            importlib.import_module(module_name)
        except Exception as exc:  # import failures need their original diagnostic
            reporter.block("import " + module_name, "{}: {}".format(type(exc).__name__, exc))
        else:
            reporter.passed("import " + module_name, "ok")

    try:
        import torch
        import torchvision

        build_details = "torch {}; torchvision {}; build CUDA {}".format(
            torch.__version__, torchvision.__version__, torch.version.cuda
        )
        if (
            torch.__version__ == "1.13.1+cu117"
            and torchvision.__version__ == "0.14.1+cu117"
            and torch.version.cuda == "11.7"
        ):
            reporter.passed("Torch build", build_details)
        else:
            reporter.block("Torch build", "unexpected " + build_details)
    except Exception as exc:
        reporter.block("Torch build", "{}: {}".format(type(exc).__name__, exc))

    try:
        import pytorch3d
        from pytorch3d import _C

        package_path = Path(pytorch3d.__file__).resolve()
        extension_path = Path(_C.__file__).resolve()
        reporter.passed("PyTorch3D extension", str(extension_path))
        try:
            package_path.relative_to(Path(sys.prefix).resolve())
        except ValueError:
            reporter.warn(
                "PyTorch3D install",
                "editable/external package at {}; preserve source checkout".format(
                    package_path.parent
                ),
            )
        else:
            reporter.passed("PyTorch3D install", str(package_path.parent))
    except Exception as exc:
        reporter.block(
            "PyTorch3D extension", "{}: {}".format(type(exc).__name__, exc)
        )

    # pyrealsense2 is intentionally imported only by --runtime because importing
    # it initializes udev on this SDK build.
    try:
        importlib.metadata.version("pyrealsense2")
    except importlib.metadata.PackageNotFoundError:
        reporter.block("RealSense package", "pyrealsense2 not installed")
    else:
        reporter.passed("RealSense package", "installed; runtime import deferred")

    for relative in ("anatomy_layer.py", "cliff_demo.py"):
        source_path = MODULE_ROOT / relative
        try:
            compile(source_path.read_text(encoding="utf-8"), str(source_path), "exec")
        except Exception as exc:
            reporter.block("source " + relative, "{}: {}".format(type(exc).__name__, exc))
        else:
            reporter.passed("source " + relative, "syntax ok")

    try:
        entries = read_manifest()
    except Exception as exc:
        reporter.block("asset manifest", "{}: {}".format(type(exc).__name__, exc))
        return

    for expected, path in entries:
        relative = path.relative_to(MODULE_ROOT).as_posix()
        if not path.is_file():
            reporter.block("asset " + relative, "missing")
            continue
        actual = file_sha256(path)
        if actual == expected:
            reporter.passed("asset " + relative, "sha256 ok")
        else:
            reporter.block("asset " + relative, "sha256 mismatch")


def check_runtime(reporter: Reporter) -> None:
    gpu_descriptions = []
    gpu_root = Path("/proc/driver/nvidia/gpus")
    for information in sorted(gpu_root.glob("*/information")):
        fields = {}
        try:
            for line in information.read_text(encoding="utf-8").splitlines():
                if ":" in line:
                    key, value = line.split(":", 1)
                    fields[key.strip()] = value.strip()
        except OSError:
            continue
        gpu_descriptions.append(
            "{} at {}".format(
                fields.get("Model", "unknown NVIDIA GPU"),
                fields.get("Bus Location", information.parent.name),
            )
        )
    if gpu_descriptions:
        reporter.passed("GPU kernel binding", "; ".join(gpu_descriptions))
    else:
        reporter.block("GPU kernel binding", "no GPU under /proc/driver/nvidia/gpus")

    nvidia_nodes_visible = Path("/dev/nvidia0").exists()
    try:
        completed = subprocess.run(
            [
                "nvidia-smi",
                "--query-gpu=name,driver_version,memory.total",
                "--format=csv,noheader",
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        reporter.block("NVIDIA driver", "{}: {}".format(type(exc).__name__, exc))
    else:
        detail = (completed.stdout or completed.stderr).strip()
        if completed.returncode == 0 and detail:
            reporter.passed("NVIDIA driver", detail.replace("\n", "; "))
        else:
            if not nvidia_nodes_visible:
                detail = (
                    (detail + "; " if detail else "")
                    + "/dev/nvidia0 is not visible; run this check in a host terminal"
                )
            reporter.block("NVIDIA driver", detail or "nvidia-smi failed")

    try:
        import torch

        if torch.cuda.is_available():
            reporter.passed(
                "Torch CUDA",
                "{}; build CUDA {}; device {}".format(
                    torch.__version__, torch.version.cuda, torch.cuda.get_device_name(0)
                ),
            )
        else:
            reporter.block(
                "Torch CUDA",
                "{}; build CUDA {}; no available device".format(
                    torch.__version__, torch.version.cuda
                ),
            )
    except Exception as exc:
        reporter.block("Torch CUDA", "{}: {}".format(type(exc).__name__, exc))

    realsense_descriptions = []
    for usb_device in sorted(Path("/sys/bus/usb/devices").glob("*")):
        try:
            vendor = (usb_device / "idVendor").read_text().strip().lower()
            product_name = (usb_device / "product").read_text().strip()
        except OSError:
            continue
        if vendor != "8086" or "realsense" not in product_name.lower():
            continue
        try:
            serial = (usb_device / "serial").read_text().strip()
        except OSError:
            serial = "unknown serial"
        try:
            speed = (usb_device / "speed").read_text().strip()
        except OSError:
            speed = "unknown"
        realsense_descriptions.append(
            "{} ({}, {} Mb/s)".format(product_name, serial, speed)
        )
    if realsense_descriptions:
        reporter.passed("RealSense kernel", "; ".join(realsense_descriptions))
    else:
        reporter.block("RealSense kernel", "no RealSense USB device in sysfs")

    try:
        import pyrealsense2 as rs

        devices = list(rs.context().query_devices())
        if devices:
            descriptions = []
            for device in devices:
                name = device.get_info(rs.camera_info.name)
                serial = device.get_info(rs.camera_info.serial_number)
                descriptions.append("{} ({})".format(name, serial))
            reporter.passed("RealSense device", "; ".join(descriptions))
        else:
            reporter.block("RealSense device", "SDK initialized but no device found")
    except Exception as exc:
        detail = "{}: {}".format(type(exc).__name__, exc)
        if realsense_descriptions:
            detail += "; kernel sees the camera, check host /dev and udev access"
        reporter.block("RealSense runtime", detail)

    try:
        import torch
        from torchvision.models.detection import FasterRCNN_ResNet50_FPN_Weights

        weight_url = FasterRCNN_ResNet50_FPN_Weights.DEFAULT.url
        checkpoint = Path(torch.hub.get_dir()) / "checkpoints" / Path(weight_url).name
        if checkpoint.is_file():
            reporter.passed("detector checkpoint", str(checkpoint))
        else:
            reporter.warn(
                "detector checkpoint",
                "not cached; first functional run would download {}".format(weight_url),
            )
    except Exception as exc:
        reporter.block("detector checkpoint", "{}: {}".format(type(exc).__name__, exc))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--static", action="store_true", help="check environment and assets")
    mode.add_argument("--runtime", action="store_true", help="also check GPU and RealSense")
    args = parser.parse_args()

    reporter = Reporter()
    check_static(reporter)
    if args.runtime:
        check_runtime(reporter)

    print(
        "SUMMARY blocked={} warnings={} mode={}".format(
            reporter.blocked, reporter.warnings, "runtime" if args.runtime else "static"
        )
    )
    return 1 if reporter.blocked else 0


if __name__ == "__main__":
    sys.exit(main())
