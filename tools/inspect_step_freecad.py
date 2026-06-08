import sys
from pathlib import Path

import FreeCAD as App
import Import


def safe_volume(shape):
    try:
        return float(shape.Volume)
    except Exception:
        return 0.0


def inspect_step(step_path):
    step_path = Path(step_path)
    doc = App.newDocument(step_path.stem)

    Import.insert(str(step_path), doc.Name)
    App.ActiveDocument.recompute()

    print(f"\nFILE: {step_path.name}")
    print(f"TOTAL OBJECTS: {len(doc.Objects)}")

    real_objects = []

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

        real_objects.append((obj, volume, box))

    print(f"REAL CAD OBJECTS: {len(real_objects)}")

    for obj, volume, box in real_objects:
        label = getattr(obj, "Label", obj.Name)

        print(
            f"{obj.Name} | {label}: "
            f"Volume={volume:.2f} "
            f"X=({box.XMin:.2f}, {box.XMax:.2f}) "
            f"Y=({box.YMin:.2f}, {box.YMax:.2f}) "
            f"Z=({box.ZMin:.2f}, {box.ZMax:.2f}) "
            f"Size=({box.XLength:.2f}, {box.YLength:.2f}, {box.ZLength:.2f})"
        )

    App.closeDocument(doc.Name)


def main():
    if len(sys.argv) < 2:
        raise SystemExit("Usage: freecadcmd inspect_step_freecad.py file.step")

    inspect_step(sys.argv[1])


main()