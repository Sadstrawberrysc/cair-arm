"""Read a Roboflow key from the environment or a private local file."""
import argparse
import getpass
import os
from pathlib import Path
import stat
import tempfile


def key_path():
    return Path.home() / ".config" / "carotid_begin" / "roboflow_api_key"


def get_api_key(path=None):
    """Prefer the process environment; otherwise read the local private file."""
    key = os.environ.get("ROBOFLOW_API_KEY", "").strip()
    if key:
        return key
    path = Path(path) if path is not None else key_path()
    try:
        mode = path.lstat().st_mode
    except FileNotFoundError:
        return None
    if not stat.S_ISREG(mode) or mode & 0o077:
        raise ValueError(f"API Key file must be a regular file with mode 600: {path}")
    key = path.read_text(encoding="utf-8").strip()
    if not key:
        raise ValueError(f"API Key file is empty: {path}")
    return key


def save_api_key(key, path=None):
    """Atomically save a key outside the repository, readable only by the user."""
    if not key or not key.strip() or "\n" in key or "\r" in key:
        raise ValueError("API Key must be a nonempty single line")
    path = Path(path) if path is not None else key_path()
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    path.parent.chmod(0o700)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix=".roboflow_api_key_", delete=False) as stream:
            temporary = Path(stream.name)
            os.fchmod(stream.fileno(), 0o600)
            stream.write(key.strip() + "\n")
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return path


def main(argv=None):
    parser = argparse.ArgumentParser(description="Save a Roboflow API Key in a local private file")
    parser.add_argument("action", choices=["save", "check"])
    args = parser.parse_args(argv)
    if args.action == "save":
        key = getpass.getpass("Roboflow API Key: ")
        try:
            path = save_api_key(key)
        except ValueError as error:
            parser.error(str(error))
        print(f"Saved to {path} (mode 600). Key value was not printed.")
    else:
        try:
            present = bool(get_api_key())
        except ValueError as error:
            parser.error(str(error))
        print("Roboflow API Key available" if present else "Roboflow API Key missing")


if __name__ == "__main__":
    main()
