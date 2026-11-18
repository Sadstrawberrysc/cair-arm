"""Best-effort process precheck; does not open devices or terminate owners."""
import os
from pathlib import Path


def ensure_global_camera_free(proc_root=Path('/proc')):
    owners = []
    for process in proc_root.iterdir():
        if not process.name.isdigit() or int(process.name) == os.getpid():
            continue
        try:
            args = [a.decode(errors='replace') for a in
                    (process / 'cmdline').read_bytes().split(b'\0') if a]
        except (OSError, PermissionError):
            continue
        if not args:
            continue
        executable = Path(args[0]).name
        collision = executable == 'collision_monitor' and '--live' in args
        camera = executable.startswith('python') and any(
            Path(a).name == 'cliff_demo.py' for a in args[1:])
        if collision or camera:
            mode = 'Collision 执行模式' if collision and '--execute' in args else executable
            owners.append('PID {} ({})'.format(process.name, mode))
    if owners:
        raise ValueError('全局相机入口已运行：{}。先在原 Collision 窗口按空格停止、确认停止后退出，'
                         '或在相机窗口按 q 退出，再重新采点；已有目标可直接在原 Collision 窗口按 P／E。'
                         .format('、'.join(owners)))
