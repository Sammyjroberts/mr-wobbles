from pathlib import Path

import trimesh, numpy as np

OUT_PATH = Path(__file__).parent / "balancer_chassis_v1.stl"

W, H, T = 80.0, 180.0, 6.0          # width(front-back) x height x thickness(mm)
base = trimesh.creation.box(extents=[W, H, T])   # centered at origin

tools = []
# ---- motor-bracket mount grid: M3 clearance (3.4mm) at 12.7mm pitch, lower zone
xs = [-31.75, -19.05, -6.35, 6.35, 19.05, 31.75]
ys = [-82.0, -69.3, -56.6, -43.9, -31.2]
holes = []
for x in xs:
    for y in ys:
        c = trimesh.creation.cylinder(radius=1.7, height=T+2, sections=24)
        c.apply_translation([x, y, 0]); tools.append(c); holes.append((x, y))
# ---- zip-tie strap slots up top (3 x 8 mm) for driver / Pico / battery
slots = []
for y in [10.0, 45.0, 78.0]:
    for x in [-18.0, 18.0]:
        s = trimesh.creation.box(extents=[3, 8, T+2]); s.apply_translation([x, y, 0])
        tools.append(s); slots.append((x, y))
# ---- central wire pass-through (16 x 11 mm)
wc = trimesh.creation.box(extents=[16, 11, T+2]); wc.apply_translation([0, -7, 0]); tools.append(wc)

cut = trimesh.util.concatenate(tools)
plate = trimesh.boolean.difference([base, cut])
plate.export(str(OUT_PATH))
print("watertight:", plate.is_watertight)
print("size mm:", np.round(plate.extents,1))
print("volume cm3:", round(plate.volume/1000,1), " (~", round(plate.volume/1000*1.27,0), "g PETG)")
print("holes:", len(holes), " slots:", len(slots))
