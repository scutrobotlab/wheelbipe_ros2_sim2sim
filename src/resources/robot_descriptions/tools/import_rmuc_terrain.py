"""Offline STL terrain compiler. Run with uv --with trimesh --with scipy.

Visual geometry preserves every source triangle; collision is its upper
surface at 20 mm spacing. Undercuts/tunnels are not represented by a heightfield.
No STL parsing or geometry conversion occurs in the simulation loop.
"""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import trimesh


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('stl', type=Path)
    parser.add_argument('--spacing', type=float, default=0.02)
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[1] / 'wheelbipeV14_2/mjcf')
    args = parser.parse_args()
    if not 0.005 <= args.spacing <= 0.1:
        parser.error('spacing must be between 0.005 and 0.1 metres')
    out = args.output
    assets = out / 'terrain_rmuc2026'
    assets.mkdir(parents=True, exist_ok=True)
    mesh = trimesh.load_mesh(args.stl, process=False)
    mesh.apply_scale(0.001)  # CAD millimetres to simulation metres.
    bounds = mesh.bounds.copy()
    center = bounds.mean(axis=0)
    # Dominant horizontal surface by area, not vertex count (decorative CAD
    # details contain far more vertices than the playing field).
    tri = mesh.triangles
    flat = np.ptp(tri[:, :, 2], axis=1) < 1e-5
    levels, inverse = np.unique(np.round(tri[flat, :, 2].mean(axis=1), 3), return_inverse=True)
    weights = np.bincount(inverse, weights=mesh.area_faces[flat])
    dominant = levels[weights.argmax()]
    ground = float(np.median(tri[flat, :, 2].mean(axis=1)[np.abs(tri[flat, :, 2].mean(axis=1) - dominant) < .0005]))
    mesh.apply_translation([-center[0], -center[1], -ground])
    bounds = mesh.bounds.copy()
    nx, ny = np.ceil((bounds[1, :2] - bounds[0, :2]) / args.spacing).astype(int) + 1
    xs = np.linspace(bounds[0, 0], bounds[1, 0], nx)
    ys = np.linspace(bounds[0, 1], bounds[1, 1], ny)
    heights = np.full((ny, nx), bounds[0, 2], dtype=np.float32)
    covered = np.zeros((ny, nx), dtype=bool)
    for a, b, c in mesh.triangles:
        den = (b[1]-c[1])*(a[0]-c[0]) + (c[0]-b[0])*(a[1]-c[1])
        if abs(den) < 1e-12:
            continue
        lo = np.minimum(np.minimum(a, b), c)
        hi = np.maximum(np.maximum(a, b), c)
        i0, i1 = np.searchsorted(xs, [lo[0]-1e-9, hi[0]+1e-9])
        j0, j1 = np.searchsorted(ys, [lo[1]-1e-9, hi[1]+1e-9])
        if i0 == i1 or j0 == j1:
            continue
        x, y = xs[None, i0:i1], ys[j0:j1, None]
        u = ((b[1]-c[1])*(x-c[0]) + (c[0]-b[0])*(y-c[1])) / den
        v = ((c[1]-a[1])*(x-c[0]) + (a[0]-c[0])*(y-c[1])) / den
        inside = (u >= -1e-7) & (v >= -1e-7) & (u+v <= 1+1e-7)
        z = u*a[2] + v*b[2] + (1-u-v)*c[2]
        block = heights[j0:j1, i0:i1]
        np.maximum(block, np.where(inside, z, bounds[0, 2]), out=block)
        covered[j0:j1, i0:i1] |= inside
    zmin, zmax = float(heights.min()), float(heights.max())
    with (assets / 'collision.bin').open('wb') as f:
        np.asarray([ny, nx], dtype='<i4').tofile(f)
        ((heights-zmin)/(zmax-zmin)).astype('<f4').tofile(f)
    np.save(assets / 'height_metres.npy', heights)
    # Split only to stay below the per-mesh STL face limit. No decimation,
    # vertex welding, smoothing or geometry removal on the visual path.
    visual_parts = []
    for start in range(0, len(mesh.faces), 150000):
        triangles = mesh.triangles[start:start+150000]
        part = trimesh.Trimesh(vertices=triangles.reshape(-1, 3),
                               faces=np.arange(triangles.size // 3).reshape(-1, 3), process=False)
        name = f'visual_full_{len(visual_parts):02d}'
        part.export(assets / f'{name}.stl')
        visual_parts.append(name)
    # Choose a clear patch on the ground with at least 1 m clearance around it.
    from scipy.ndimage import distance_transform_edt
    clear = (np.abs(heights) < .015) & covered
    clearance = distance_transform_edt(clear, sampling=(ys[1]-ys[0], xs[1]-xs[0]))
    # Prefer the negative-x half of the field, facing into the map.
    eligible = (clearance > 1.2) & (xs[None, :] < -3)
    if not eligible.any():
        raise RuntimeError('No clear spawn patch found; inspect terrain before spawning')
    score = np.where(eligible, clearance, -1)
    j, i = np.unravel_index(score.argmax(), score.shape)
    spawn = [float(xs[i]), float(ys[j]), float(heights[j, i]) + .38]
    scene = ET.parse(out / 'scene_source.xml')
    root = scene.getroot()
    # Fit depth precision and shadow coverage to the entire field, rather than
    # inheriting the small robot test course's 0.82 m scene extent.
    statistic = root.find('statistic')
    statistic.set('center', '0 0 0.8')
    statistic.set('extent', '17')
    visual = root.find('visual')
    ET.SubElement(visual, 'map', znear='0.005', zfar='5', shadowclip='1')
    ET.SubElement(visual, 'quality', shadowsize='8192', offsamples='4')
    asset = root.find('asset')
    for name in visual_parts:
        ET.SubElement(asset, 'mesh', name=name, file=f'../mjcf/terrain_rmuc2026/{name}.stl')
    ET.SubElement(asset, 'hfield', name='rmuc_collision', file='../mjcf/terrain_rmuc2026/collision.bin',
                  size=f'{(xs[-1]-xs[0])/2} {(ys[-1]-ys[0])/2} {zmax-zmin} 0.1')
    world = root.find('worldbody')
    for geom in list(world.findall('geom')):
        if geom.get('name') != 'floor': world.remove(geom)
        else: geom.set('pos', f'0 0 {zmin-.01}')
    for name in visual_parts:
        ET.SubElement(world, 'geom', name=name, type='mesh', mesh=name,
                      contype='0', conaffinity='0', group='2', rgba='0.55 0.62 0.66 1')
    ET.SubElement(world, 'geom', name='rmuc_map_collision', type='hfield', hfield='rmuc_collision',
                  pos=f'0 0 {zmin}', contype='1', conaffinity='1', group='3', rgba='0 0 0 0', friction='1.2 0.02 0.01')
    key = root.find('keyframe/key')
    qpos = key.get('qpos').split()
    qpos[:3] = [str(v) for v in spawn]
    key.set('qpos', ' '.join(qpos))
    ET.indent(scene)
    scene.write(out / 'scene_rmuc2026.xml', encoding='unicode')
    metadata = dict(source=str(args.stl), sha256=hashlib.sha256(args.stl.read_bytes()).hexdigest(),
                    scale=.001, translation=[-center[0], -center[1], -ground], bounds=bounds.tolist(),
                    grid=[int(ny), int(nx)], spacing=[float(xs[1]-xs[0]), float(ys[1]-ys[0])],
                    spawn=spawn, spawn_clearance=float(clearance[j, i]),
                    source_faces=len(mesh.faces), visual_faces=len(mesh.faces), visual_parts=visual_parts, visual_decimation=False,
                    collision='20 mm upper-surface heightfield; no undercuts/tunnels')
    (assets / 'metadata.json').write_text(json.dumps(metadata, indent=2)+'\n')
    print(json.dumps(metadata, indent=2))


if __name__ == '__main__':
    main()
