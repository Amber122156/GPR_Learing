# HK prototype visual assets

This directory contains derived visual-only meshes from the current STL files
in `../../CAD`.  The original Flappy assets remain in
`../flappy_urdf/meshes`.  Run `scripts/apply_hk_prototype_visuals.m` after
changing a CAD STL.  The current convention is left `+Y` span / `+Z` chord;
the mapping converts this to Flappy left `+Y` span / `-Z` chord and mirrors
the left geometry to create the right side.

| Link | File | STL unit |
|---|---|---|
| torso | `torso_symmetric_visual.STL` | mm |
| left leading edge | `left_LE_symmetric_visual.STL` | mm |
| right leading edge | `right_LE_symmetric_visual.STL` | mm |
| left wing | `left_wing_symmetric_visual.STL` | mm |
| right wing | `right_wing_symmetric_visual.STL` | mm |

The right leading edge and wing are generated from the CAD left components by
reflection about the torso XZ plane; their triangle winding is reversed so
surface normals remain outward.  The supplied right-side CAD components are
not used because the visual model is intentionally strictly symmetric.

No value in this directory changes mass, centre of mass, inertia, joints, or
aerodynamics.  Those remain the frozen Flappy baseline until a separate
physical-parameter update is made.
