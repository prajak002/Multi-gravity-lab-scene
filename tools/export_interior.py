"""Export Corridor.blend -> public/env/iss_corridor.glb.

The blend is a single 43.8 m corridor mesh lit by 40 point lights. Exporting
the lights into the glTF would hand three.js 40 real-time lights, which is not
survivable at 60 fps; they are dumped to JSON instead so the renderer can place
a handful of representative ones along the corridor axis.

The 4k textures the materials reference are NOT next to the blend (they point
at //../../texture/, which does not exist on this machine), so only the
material base colours survive. That is fine — Interior.js re-materialises the
module anyway.
"""
import bpy, json, os, mathutils, sys

OUT = sys.argv[-1]

# --- light survey, before anything is deleted ---------------------------------
lights = []
for o in bpy.data.objects:
    if o.type != 'LIGHT':
        continue
    d = o.data
    w = o.matrix_world.translation
    lights.append({
        'name': o.name,
        'type': d.type,
        # Blender is Z-up, glTF/three are Y-up: (x, y, z)_blender -> (x, z, -y).
        'pos': [round(w.x, 4), round(w.z, 4), round(-w.y, 4)],
        'color': [round(c, 4) for c in d.color],
        'energy': round(d.energy, 3),
    })
lights.sort(key=lambda l: l['pos'][0])

# --- material base colours, for the same reason -------------------------------
mats = []
for m in bpy.data.materials:
    base = None
    if m.use_nodes:
        for n in m.node_tree.nodes:
            if n.type == 'BSDF_PRINCIPLED':
                base = [round(v, 4) for v in n.inputs['Base Color'].default_value]
                break
    mats.append({'name': m.name, 'base_color': base})

with open(os.path.splitext(OUT)[0] + '_lights.json', 'w') as f:
    json.dump({'lights': lights, 'materials': mats}, f, indent=1)

# --- strip everything that is not the corridor shell --------------------------
for o in list(bpy.data.objects):
    if o.type != 'MESH':
        bpy.data.objects.remove(o, do_unlink=True)

# Recompute the bounds of what is actually being exported, so the renderer can
# place the robot inside the tube without guessing.
mn = [1e9] * 3
mx = [-1e9] * 3
for o in bpy.data.objects:
    for c in o.bound_box:
        w = o.matrix_world @ mathutils.Vector(c)
        for i in range(3):
            mn[i] = min(mn[i], w[i])
            mx[i] = max(mx[i], w[i])
meta = {
    'blender_bbox_min': [round(v, 4) for v in mn],
    'blender_bbox_max': [round(v, 4) for v in mx],
    'lights': lights,
    'materials': mats,
}
with open(os.path.splitext(OUT)[0] + '_lights.json', 'w') as f:
    json.dump(meta, f, indent=1)

bpy.ops.export_scene.gltf(
    filepath=OUT,
    export_format='GLB',
    export_yup=True,               # Blender Z-up -> glTF Y-up
    export_apply=True,             # bake modifiers
    export_materials='EXPORT',
    export_image_format='NONE',    # the 4k textures are not on disk anyway
    export_cameras=False,
    export_lights=False,
    export_animations=False,
    export_normals=True,
    export_texcoords=True,
    use_selection=False,
)
print("WROTE", OUT, os.path.getsize(OUT))
