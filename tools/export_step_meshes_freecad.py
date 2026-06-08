import sys
from pathlib import Path

import FreeCAD as App
import Import
import Mesh
import MeshPart
import Part


def safe_volume(shape):
    try:
        return float(shape.Volume)
    except Exception:
        return 0.0


def get_real_shapes(doc):
    real_shapes = []

    for obj in doc.Objects:
        if not hasattr(obj, "Shape"):
            continue

        shape = obj.Shape
        if shape.isNull():
            continue

        volume = safe_volume(shape)
        box = shape.BoundBox

        if volume <= 1e-6:
            continue

        if box.XLength > 10000 or box.YLength > 10000 or box.ZLength > 10000:
            continue

        real_shapes.append(shape)

    return real_shapes


def export_step(step_path, out_path):
    step_path = Path(step_path)
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    doc = App.newDocument(step_path.stem)

    Import.insert(str(step_path), doc.Name)
    App.ActiveDocument.recompute()

    real_shapes = get_real_shapes(doc)

    print(f"STEP: {step_path.name}")
    print(f"Real shapes: {len(real_shapes)}")
    print(f"Exporting: {out_path}")

    if not real_shapes:
        App.closeDocument(doc.Name)
        raise RuntimeError(f"No real CAD shapes found in {step_path}")

    compound = Part.makeCompound(real_shapes)

    mesh = MeshPart.meshFromShape(
        Shape=compound,
        LinearDeflection=24.0,
        AngularDeflection=2.2,
        Relative=False,
    )

    mesh.write(str(out_path))

    App.closeDocument(doc.Name)


def main():
    if len(sys.argv) < 3:
        raise SystemExit("Usage: export_step_meshes_freecad.py input.step output.stl")

    export_step(sys.argv[1], sys.argv[2])


main()