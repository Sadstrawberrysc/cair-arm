"""Build conservative moving-body spheres from the repository's collision meshes.

Attachment placements follow rm_75.urdf (preview geometry); this is a height
envelope, not a self/environment collision certificate or hardware measurement.
"""
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    x, y, z = rpy
    cx, sx, cy, sy, cz, sz = math.cos(x), math.sin(x), math.cos(y), math.sin(y), math.cos(z), math.sin(z)
    t = np.eye(4)
    t[:3, :3] = [[cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx],
                  [sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx], [-sy, cy*sx, cy*cx]]
    t[:3, 3] = xyz
    return t


def origin(element):
    if element is None:
        return np.eye(4)
    return transform(*[[float(v) for v in element.get(key, '0 0 0').split()] for key in ('xyz', 'rpy')])


def mesh_vertices(path):
    data = path.read_bytes()
    count = struct.unpack_from('<I', data, 80)[0] if len(data) >= 84 else 0
    if len(data) == 84 + count * 50:
        values = [struct.unpack_from('<9f', data, 84 + i*50 + 12) for i in range(count)]
        vertices = np.asarray(values).reshape(-1, 3)
    else:
        vertices = np.asarray([[float(v) for v in line.split()[1:]]
                               for line in data.decode().splitlines() if line.strip().startswith('vertex ')])
    if vertices.ndim != 2 or vertices.shape[1] != 3 or not np.isfinite(vertices).all():
        raise ValueError(f'Invalid STL vertices: {path}')
    return vertices


def body_spheres(robot):
    collision = robot.parent / 'Collision'
    root = ET.parse(collision / 'rm_75.urdf').getroot()
    joints = {j.find('child').get('link'): j for j in root.findall('joint')}
    zero = {'base_link': np.eye(4)}

    def frame(name):
        if name not in zero:
            joint = joints[name]
            zero[name] = frame(joint.find('parent').get('link')) @ origin(joint.find('origin'))
        return zero[name]

    mdh = []
    t = np.eye(4)
    for alpha, distance in zip((0, -90, 90, -90, 90, -90, 90), (.2405, 0, .256, 0, .210, 0, .144)):
        # Rx(alpha) Tz(d); joints are zero here.
        t = t @ transform(rpy=(math.radians(alpha), 0, 0)) @ transform(xyz=(0, 0, distance))
        mdh.append(t.copy())
    spheres = []
    for link in root.findall('link'):
        name = link.get('name')
        if name in ('base_link', 'support_link'):
            continue
        ancestor = name
        while not ancestor.startswith('Link'):
            ancestor = joints[ancestor].find('parent').get('link')
        index = int(ancestor[4:]) - 1
        vertices = []
        for body in link.findall('collision'):
            mesh = body.find('geometry/mesh')
            if mesh is None:
                raise ValueError(f'Unsupported height envelope geometry: {name}')
            points = mesh_vertices(collision / 'mesh' / Path(mesh.get('filename')).name)
            points *= np.array([float(v) for v in mesh.get('scale', '1 1 1').split()])
            mapping = np.linalg.inv(mdh[index]) @ frame(name) @ origin(body.find('origin'))
            vertices.append(points @ mapping[:3, :3].T + mapping[:3, 3])
        points = np.concatenate(vertices)
        center = (points.min(axis=0) + points.max(axis=0)) / 2
        # 2 mm padding covers URDF's rounded rotations and numerical conversion.
        radius = float(np.linalg.norm(points-center, axis=1).max()) + .002
        spheres.append(dict(name=name, joint_index=index, center_m=center.tolist(), radius_m=radius))
    return spheres
