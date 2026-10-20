"""
Convert the RCM-7 STL parts into STEP solids with FreeCAD (free, freecad.org).

  freecadcmd stl_to_step_freecad.py        # run from the folder that contains stl/
  or: FreeCAD > Macro > Macros... > select this file > Execute

Writes step/<part>.step for every part and step/RCM7_assembly.step with all parts placed at the
reference pose (yaw 0, pitch 0, insertion 100 mm). Coplanar facets are merged, so flat faces come
out as single planar faces; round features stay faceted (48-64 segments).
"""
import os, glob
import FreeCAD, Mesh, Part, Import

here = os.path.dirname(os.path.abspath(__file__)) if '__file__' in globals() else os.getcwd()
src = os.path.join(here, 'stl')
dst = os.path.join(here, 'step')
os.makedirs(dst, exist_ok=True)

doc = FreeCAD.newDocument('RCM7')
objs, failed = [], []
for f in sorted(glob.glob(os.path.join(src, '*.stl'))):
    name = os.path.splitext(os.path.basename(f))[0]
    try:
        mesh = Mesh.Mesh(f)
        shape = Part.Shape()
        shape.makeShapeFromMesh(mesh.Topology, 0.01)
        solids = []
        for shell in Part.Shape(shape).Shells or [Part.Shell(shape.Faces)]:
            s = Part.Solid(shell)
            if s.Volume < 0: s.reverse()
            solids.append(s)
        solid = (solids[0] if len(solids) == 1 else Part.makeCompound(solids)).removeSplitter()
        obj = doc.addObject('Part::Feature', name.replace('-', '_'))
        obj.Shape = solid
        objs.append(obj)
        Import.export([obj], os.path.join(dst, name + '.step'))
    except Exception as e:
        failed.append((name, str(e)))
doc.recompute()
Import.export(objs, os.path.join(dst, 'RCM7_assembly.step'))
FreeCAD.Console.PrintMessage(f'Converted {len(objs)} parts to {dst}\n')
for n, e in failed:
    FreeCAD.Console.PrintError(f'  failed: {n}: {e}\n')
