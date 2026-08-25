"""
gen_body.py — generate src/sim/G1Body.js from the shipped URDF.

The leg solver needs one chain; the microgravity work needs the whole robot:
every link's mass, its inertial origin AND its inertia tensor, because in free
fall the body's orientation is governed by conservation of angular momentum and
that needs real inertias, not a sphere approximation.

Regenerate after any URDF change:
    python3 pipeline/gen_body.py > src/sim/G1Body.js
"""
import re, sys

URDF = 'public/robots/g1/g1_29dof.urdf'
t = open(URDF).read()

links = {}
for m in re.finditer(r'<link name="([^"]+)">(.*?)</link>', t, re.S):
    n, b = m.group(1), m.group(2)
    mm = re.search(r'<mass value="([-\d.eE+]+)"', b)
    om = re.search(r'<inertial>\s*<origin xyz="([^"]*)"', b)
    it = re.search(r'<inertia\s+ixx="([-\d.eE+]+)"\s+ixy="([-\d.eE+]+)"\s+ixz="([-\d.eE+]+)"'
                   r'\s+iyy="([-\d.eE+]+)"\s+iyz="([-\d.eE+]+)"\s+izz="([-\d.eE+]+)"', b)
    links[n] = dict(
        mass=float(mm.group(1)) if mm else 0.0,
        com=[float(x) for x in (om.group(1) if om else '0 0 0').split()],
        I=[float(x) for x in it.groups()] if it else [0.0] * 6,
    )

joints = []
for m in re.finditer(r'<joint name="([^"]+)" type="([^"]+)">(.*?)</joint>', t, re.S):
    n, ty, b = m.group(1), m.group(2), m.group(3)
    if ty == 'floating':
        continue
    par = re.search(r'<parent link="([^"]*)"', b)
    ch = re.search(r'<child link="([^"]*)"', b)
    o = re.search(r'<origin[^>]*xyz="([^"]*)"', b)
    rpy = re.search(r'<origin[^>]*rpy="([^"]*)"', b)
    ax = re.search(r'<axis xyz="([^"]*)"', b)
    lim = re.search(r'<limit[^>]*lower="([-\d.eE+]+)"[^>]*upper="([-\d.eE+]+)"', b)
    a = [float(x) for x in (ax.group(1) if ax else '0 0 1').split()]
    axis = 'x' if abs(a[0]) > 0.5 else ('y' if abs(a[1]) > 0.5 else 'z')
    joints.append(dict(
        name=n, type=ty, parent=par.group(1), child=ch.group(1),
        xyz=[float(x) for x in (o.group(1) if o else '0 0 0').split()],
        rpy=[float(x) for x in (rpy.group(1) if rpy else '0 0 0').split()],
        axis=(axis if ty != 'fixed' else None),
        sign=(-1 if (a[0] + a[1] + a[2]) < 0 else 1),
        lower=float(lim.group(1)) if lim else None,
        upper=float(lim.group(2)) if lim else None,
    ))

order = ['pelvis']
out = [dict(name='pelvis', joint=None, parent=-1, xyz=[0, 0, 0], rpy=[0, 0, 0],
            axis=None, sign=1, lower=None, upper=None, **links['pelvis'])]
rem = joints[:]
while rem:
    prog = False
    for j in list(rem):
        if j['parent'] in order:
            L = links.get(j['child'], dict(mass=0.0, com=[0, 0, 0], I=[0.0] * 6))
            out.append(dict(name=j['child'], joint=(j['name'] if j['type'] != 'fixed' else None),
                            parent=order.index(j['parent']), xyz=j['xyz'], rpy=j['rpy'],
                            axis=j['axis'], sign=j['sign'],
                            lower=j['lower'], upper=j['upper'], **L))
            order.append(j['child']); rem.remove(j); prog = True
    if not prog:
        sys.exit('unreachable links: ' + ', '.join(j['child'] for j in rem))

total = sum(l['mass'] for l in out)
f = lambda xs: '[' + ','.join('%.6g' % v for v in xs) + ']'

print(f'''/**
 * G1Body — the full kinematic tree, mass distribution and inertia of the G1.
 *
 * GENERATED from {URDF} by pipeline/gen_body.py. Do not edit by hand.
 *
 * Everything that solves against the robot reads its geometry from here, so a
 * simplified transcription can never drift out of step with the URDF the
 * renderer actually loads. That mattered: the hip roll joint carries a fixed
 * rpy of [0, -0.1749, 0] and the knee an equal and opposite one. They cancel in
 * the neutral pose and nowhere else, and a hand-written chain that dropped them
 * put the solved ankle 53 mm from where urdf-loader draws it.
 *
 * Inertias are included because the microgravity clips conserve angular
 * momentum, which a mass-only model cannot do.
 *
 * Total mass {total:.4f} kg over {len(out)} links.
 */''')
print('export const G1_TOTAL_MASS = %.6f;' % total)
print()
print('/** Tree order: every link appears after its parent. */')
print('export const G1_TREE = [')
for o in out:
    jn = 'null' if o['joint'] is None else '"%s"' % o['joint']
    ax = 'null' if o['axis'] is None else '"%s"' % o['axis']
    lim = ('null' if o['lower'] is None else f"[{o['lower']:.6g},{o['upper']:.6g}]")
    print(f'  {{ name: "{o["name"]}", joint: {jn}, parent: {o["parent"]}, '
          f'xyz: {f(o["xyz"])}, rpy: {f(o["rpy"])}, axis: {ax}, sign: {o["sign"]}, '
          f'limit: {lim}, mass: {o["mass"]:.6g}, com: {f(o["com"])}, '
          f'I: {f(o["I"])} }},')
print('];')
print()
print('/** [ixx, ixy, ixz, iyy, iyz, izz] -> row-major 3x3. */')
print('export const inertiaMatrix = (I) => [I[0], I[1], I[2], I[1], I[3], I[4], I[2], I[4], I[5]];')
