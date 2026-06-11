"""Decimate the official SolidWorks->URDF HOPPY meshes for MuJoCo.

The official export (HOPPY-E0-final) ships high-resolution binary STLs. MuJoCo
rejects meshes with more than 200,000 faces, so Link2/Link3/Link4 must be
decimated. Decimation preserves each mesh's local coordinate frame, so the URDF
joint transforms still assemble them correctly.

Usage:
    python tools/prepare_cad_view_meshes.py /path/to/HOPPY-E0-final/meshes

Outputs decimated STLs to assets/meshes/hoppy_official_urdf/.
"""
import sys
from pathlib import Path

import trimesh
import fast_simplification

ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "assets" / "meshes" / "hoppy_official_urdf"

# target face count per mesh (None = copy unchanged)
TARGETS = {
    "base_link": None,
    "Link1": None,
    "Link2": 80000,
    "Link3": 80000,
    "Link4": 60000,
}


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    src = Path(sys.argv[1])
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    for name, target in TARGETS.items():
        mesh = trimesh.load(src / f"{name}.STL")
        if target is not None and len(mesh.faces) > target:
            ratio = 1.0 - target / len(mesh.faces)
            v, f = fast_simplification.simplify(mesh.vertices, mesh.faces, target_reduction=ratio)
            mesh = trimesh.Trimesh(vertices=v, faces=f)
        mesh.export(OUT_DIR / f"{name}.STL")
        print(f"{name}: {len(mesh.faces)} faces -> {OUT_DIR / f'{name}.STL'}")


if __name__ == "__main__":
    main()
