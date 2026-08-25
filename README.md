# Multi-Gravity Arena

Unitree **G1**, **H1** and **Go2** locomotion across **Moon**, **Mars**, **ISS** and
**Earth** — a web arena with a lobby that enters from the left and a camera that
moves between shots rather than cutting.

Borrows the environment/rendering work from `multi-gravity-lab` and the
retargeting pipeline from `kalarisena`.

## Run

```bash
npm install
npm run dev
```

Three pages, three different questions:

| URL | Question it answers |
|---|---|
| `/gravity.html` | **What does gravity DO?** One G1, one motion template, three fields at once |
| `/` | What does a G1 look like on the Moon? Robot x field x motion, one at a time |
| `/arena.html` | WorldVLA vs PragyaSpace, on fourteen real NASA terrains |

```bash
node tools/check_gravity.mjs   http://localhost:5173   # asserts the lanes diverge
node tools/check_all_modes.mjs http://localhost:5173   # all 36 robot x field x motion
node tools/check_arena.mjs     http://localhost:4199   # the A/B viewer, against dist/
```

## One motion, three gravitational fields

`/gravity.html` runs three G1s from the same joint template, started on the
same line in phase. The only difference between them is `g`, and they pull
apart on screen because of it.

| | Earth | Mars | Moon |
|---|---|---|---|
| step period | 0.416 s | 0.675 s | **1.022 s** |
| cadence | 1.20 Hz | 0.74 Hz | 0.49 Hz |
| duty factor | 0.64 | 0.45 | 0.39 |
| flight phase | 36 % | 55 % | **61 %** |
| stride | 0.54 m | 0.54 m | 0.54 m |
| distance after 10 s | 14.3 m | 8.8 m | **5.8 m** |
| vs Earth | x1.00 | x1.62 | **x2.46** |

That x2.46 is `sqrt(9.807/1.625)` computed, not typed.

### Speed comes from the Froude number, and used not to

This page was built and immediately exposed a bug that had been invisible in
every other view: **all three robots walked at exactly the same speed.**

`cruiseSpeed()` scaled the stride by `sqrt(g_earth/g)` and divided by a step
period that also goes as `sqrt(1/g)`. The two cancel exactly, so cruise speed
was gravity-independent by construction. Three robots side by side stayed dead
level for the whole traverse, which made gravity look like it did nothing to
travel.

The right invariant is the Froude number, `Fr = v^2 / (g*L)` — the
dimensionless speed at which legged gaits compare across body sizes and across
bodies, and why a walk breaks into a run near `Fr = 0.5` for animals from a
quail to an elephant. Holding it fixed:

```
v = sqrt(Fr * g * L)
```

so walking speed falls as `sqrt(g)`, and the Moon's is `0.407` of Earth's.
That is the real Apollo result: the crews could not walk quickly, and the
reason they loped instead is precisely that the walk-run transition speed
drops with gravity.

Note what does **not** change. `stride = v*T` goes as `sqrt(g) * sqrt(1/g)`,
which is constant — all three readouts show 0.54 m. At equal Froude number the
step LENGTH is identical in every field, and the entire difference is in how
long each step takes.

The page renders identically whether or not this bug is present, so
`tools/check_gravity.mjs` asserts on the **distances** rather than on a
screenshot, and fails if the lanes do not diverge.

## Motion from real footage: built, and blocked on footage

The pipeline for imitating a real gait and retargeting it onto the G1 exists
and is tested, but the Apollo 16 reel in `pipeline/footage/` cannot feed it.

- `pipeline/find_subject.py` — scans a reel and reports where a person is
  actually large enough to pose-fit, **before** anything expensive runs.
- `pipeline/extract_pose.py` — COCO-17 keypoints for a chosen segment, with
  same-person tracking so a shot cut cannot splice two people into one gait.
- `tools/fit_pose.mjs` — fits the G1's own chain to those keypoints by
  matching **bone directions** rather than positions, because a suited
  astronaut is 1.9 m with a backpack and a G1 is 1.32 m with proportionally
  shorter legs. Directions transfer between different proportions; positions
  do not.

Single-camera pose lifting is normally ambiguous — a limb reaching toward the
camera projects identically to one reaching away. It is not ambiguous here,
because the **G1's knee cannot bend backwards** (limit -0.087 rad). That one
hardware constraint resolves the depth flip for the whole leg chain, so no
learned 3D prior is needed.

### Why the existing preprocessed data was worthless

Worth recording, because it is the failure this tooling exists to prevent.
`pipeline/out/moonwalk/preprocess/` covers frames 0-120 of the reel — which is
the distributor's stock-footage title card. A pose model fitted a confident
skeleton to a slide of text: mean keypoint confidence 0.89-0.95, and shoulder
width 0.034 of body height where a human is ~0.20. The camera solve produced
121 identity matrices. Everything downstream inherited it.

Scanning the whole reel finds no walking astronaut at usable scale anywhere in
its twenty minutes: every large, confident detection is the white room or
mission control. What is needed is a clip of someone walking across frame for
3+ seconds — Apollo 17 Cernan/Schmitt loping, the Apollo 16 jump salute, or
any parabolic-flight clip.

### The shipped "gravity-conditioned" NPZs were not

`pipeline/out/demo_earth.npz`, `demo_moon.npz` and `demo_mars.npz` are
bit-identical in joint angles, root trajectory and contacts. Only the
`gravity` and `field` labels differ. `track_gravity.py` falls through to a
ballistic stub that leaves the root at a constant 0.365 m for all 300 frames.
Nothing in that data responds to gravity.

## What is here

| Piece | File |
|---|---|
| Lobby, enters from the left | `src/ui/Lobby.js`, `src/ui/style.css` |
| Camera rig + named shots | `src/core/Stage.js` |
| Gravity fields as data | `src/render/Environments.js` |
| Robot registry + URDF loader | `src/render/Robots.js` |
| Ground-bounce IBL | `src/render/IBL.js` |
| Terrain + crater stamping | `src/render/Terrain.js` |
| Gravity-conditioned gait | `src/sim/Gait.js` |

### Camera

Shots are defined **relative to the subject**, so one definition works for any
robot anywhere on the course. `1` establish · `2` chase · `3` hero · `4` profile,
`L` reopens the lobby. Every change is a flight, not a cut: position and aim are
eased separately so the aim settles before the dolly does and the subject stays
framed throughout.

### Why the robots are visible

Three things, each of which sank an earlier build:

1. **An environment map.** The shells are metal; at `metalness 0.42` with no IBL
   they have almost no diffuse term and nothing to reflect but the sun, and
   render as black silhouettes. `IBL.js` builds the environment from the scene's
   own physics — on an airless body the sky contributes nothing and everything
   else is ground bounce, so the lower hemisphere carries
   `groundAlbedo × sunColor × sunIntensity × sin(elev)`. Lit from below, which is
   how an Apollo suit reads against a black sky.
2. **A mid-elevation sun.** At the lunar south pole's real 2.6° the subject is
   pure silhouette. Believability beats site fidelity here, so these scenes use
   a sun the audience can see a gait by.
3. **Contact solved by measurement.** The model origin sits on the soles only in
   the neutral pose; once joints move, the lowest foot is elsewhere. Each frame
   the foot links are measured and the root lifted so the lowest one rests on
   the terrain. Skipping this is what makes a pose-driven robot hover.

### Why the joint maps are named, not positional

The three descriptions order their leg chains differently and this URDF of the
H1 is **legs-only** — 10 movable joints, arm links welded. Indexing by position
put the knee value into H1's hip yaw. Every joint is now addressed by name, and
`arms: null` on the H1 states the absence rather than silently no-op'ing.

## The fourteen scenes

Six lunar motion problems, six Martian, two in microgravity — each a different
problem, not the same walk on different ground. Every one plays on a real
NASA/USGS terrain window and carries both models: **A = WorldVLA**,
**B = PragyaSpace**.

```bash
.venv/bin/python pipeline/dem.py site          # fetch all 12 terrain windows
node tools/build_all.mjs                       # retarget all 14 scenes
node tools/audit_packets.mjs                   # what the source packets contain
```

| Scene | Terrain source | Native | Real posts |
|---|---|---|---|
| `moon_shackleton_rim` | LOLA polar GDR, south of 87.5 S | **5 m/px** | 128 |
| `mars_gale_crater` | HiRISE DEM mosaic of Gale, MSL v3 | **1 m/px** | 128 |
| `moon_aristarchus`, `moon_tycho_flank`, `moon_mare_tranquillitatis` | LOLA + Kaguya TC merge, 512 ppd | 59 m/px | 35 |
| `moon_shiv_shakti`, `moon_schrodinger_basin` | LOLA polar GDR, south of 60 S | 60 m/px | 34 |
| the six Mars sites other than Gale | HRSC + MOLA blended global DEM v2 | 200 m/px | 10 |
| `iss_momentum_gap`, `iss_brake_gap` | no terrain — microgravity | — | — |

Nothing is mirrored. `pipeline/dem.py` reads the published cloud-optimised
GeoTIFFs and PDS3 products by HTTP range request and takes only the pixels a
scene needs, recording source URL, product, resolution and terrain statistics
in a sidecar beside every patch.

### Why every patch is reprojected

The published mosaics are simple-cylindrical: a pixel is a fixed number of
projection units, not of metres. At Olympia Undae's 81 N a 200-unit pixel is
200 m north–south and 31 m east–west, so reading a window straight out of the
grid hands the engine a heightfield stretched 6.4x in one axis — a dune field
that leans. The polar products have the opposite problem. Both are fixed the
same way: each patch is warped into an azimuthal equidistant projection centred
on the site, which is true to scale in every direction through that centre.

### `dem_samples_across` — the number that tells the truth

The grid is 512 samples wide whatever the source, so the only honest statement
of information content is how many **source posts** the patch spans. Gale and
Shackleton span 128. The lunar mid-latitude sites span 35. The Mars sites on the
global blend span 10.

Below about twenty, the DEM supplies a slope and a broad landform and nothing
else, and everything the foot actually meets comes from the synthetic
centimetre-scale layer in `SiteField`. That is not a defect — no orbital product
resolves a 0.19 m sole — but it must not be presented as resolved terrain, so
the figure is carried in every sidecar and every scene.

`pipeline/dem.py` also chooses **where** to stand. Each site declares the grade
its scenario needs and the tool searches a neighbourhood for a window that
actually has it: a downhill-braking clip laid on a flat patch demonstrates
nothing. Shackleton's window is a real 12.3 deg rim grade over 184 m of relief.

## What the source packets actually contain

`tools/audit_packets.mjs` reads the NPZ directly — there is no Python in the
JS toolchain, so `tools/npz.mjs` parses the ZIP and the `.npy` members itself.
Across all 28 clips:

- **CSV and NPZ agree exactly.** The CSVs are a faithful re-export.
- **The joints are self-consistent.** Each packet's own `body_pos_w`
  reproduces under this repo's forward kinematics to **0.0 mm** across all 29
  links — *once the robot is the 29-DoF G1*. See below.
- **The feet do not stand on anything.** Relative to each packet's own ground
  plane the lowest sole sits at −514 mm on Tycho Flank and −770 mm on Ganges
  Chasma (buried), and +1110 mm on Shackleton Rim (walking a metre in the air).
  Gale's feet never touch at all: 41 mm at the closest.

The packets' own README says it plainly — "research-informed synthetic
reference trajectories, not physics rollouts". The root is authored along an
assumed slope and the legs were never reconciled with it. That is the whole
reason `tools/retarget.mjs` exists.

### The robot is the 29-DoF G1, not the 23

The packets are authored on the 29-DoF G1 and their CSV column order *is* that
URDF's joint order. `g1_23dof` welds waist roll, waist pitch and both wrist
pitch/yaw — six of the packets' 29 channels had nowhere to go, which put the
shoulders up to 65 mm from where the packet says they are. On `g1_29dof` the
residual is 0.0 mm.

## Retargeting: what is kept and what is rebuilt

**Kept** — the root path, the bounce/crouch residual of pelvis height, arm and
waist angles, and every event COUNT the packet measured. Recoveries, missed
contacts, slip distance and step-duration variance are read out of each
packet's own manifest and override the scene's posture settings, so a clip
never contains struggle its authors did not record.

**Rebuilt** — every leg joint, by solving the real 6-DoF chain against a
foothold planted on the terrain.

Five things had to be right before the feet would stay on the ground:

**Heading does not come from the path tangent.** Every packet traverse is
straight — net direction within 0.3 deg of the packet's own +x axis over
4.5–5.3 m — but each carries a lateral wobble comparable to the forward step
(53 mm/frame forward against ±28 mm sideways). `atan2` of that sweeps **1404
degrees** over the Gale WorldVLA clip. A robot that wobbles three centimetres
while walking forward does not rotate four times. Yaw comes from the scene
traverse direction, corrected only by long-baseline path drift.

**The path splits into a route and a sway.** The authored lateral signal moves
0.287 m sideways within a single step period; a biped's lateral COM excursion
is a few centimetres. Low-passing over two step periods separates a slow drift
off the intended line (the **route**, which the feet follow) from a small fast
residual (**sway**, which the pelvis does over planted feet, capped at 60 mm).
The packets' own "RMS lateral error" metric — 0.028 m against 0.004 m — *is*
that residual, so the split recovers the number they quote.

**The pelvis height is derived, not asserted.** At a fixed height you cannot
have both a straight knee and a usable stride: 0.645 m gives a 0.59 m stride
and an 79 deg knee for the whole traverse, 0.74 m gives a 47 deg knee and a
0.21 m stride. A biped resolves this by rising over the stance leg and dropping
through double support — the two requirements are met at different *moments*.
So at each frame, for every loaded foot, the solver asks how high the hip can be
and still reach that foothold, and takes the lowest answer. The rise and fall
falls out in phase with the steps because it is caused by them.

**The leg is shorter than the pendulum wants.** Step period from the linear
inverted pendulum is `2*sqrt(L/g)`, so at one sixth g it is 2.46x the Earth
value — which at 0.5 m/s asks for a 0.69 m stride and puts the foot 0.35 m fore
and aft of the hip. Against a 0.6465 m reach budget with the stance width
already spent, 0.27 m is left. The natural cadence is not available and the
robot must step faster than its own pendulum: a hardware limit beating a
dynamics preference. It is bounded per *frame* against local speed, because
Gale's WorldVLA clip averages 0.573 m/s and peaks at 1.585.

**The foot is seated on its own contact spheres.** The foothold height is
planned from a plane fitted under a level sole, but the ankle then clamps to
±15 deg of roll and the sole ends up at a different angle. A real foot that
cannot conform tips onto an edge and sits *higher*. So after solving, the
achieved sole orientation is taken out of forward kinematics and the ankle
raised until the lowest of the four contact spheres — each against the ground
directly beneath itself — just touches, less 4 mm of regolith compression.

### Where it landed

| | WorldVLA | PragyaSpace |
|---|---|---|
| sole penetration | −4.0 to −5.0 mm | −4.0 to −5.0 mm |
| position error while loaded | 0.02–12.7 mm | 0.02–7.4 mm |
| slip while the gait says LOADED | 0.3–17.6 % of travel | 0.0–8.8 % |
| swing direction | forward on all 12 | forward on all 12 |

Before retargeting, the swing foot travelled **backward** relative to the pelvis
in 17 of the 28 clips and the stance foot slid for up to 125 % of the distance
the robot covered. A is worse than B on slip in every scene, and that contrast
now falls out of contact geometry rather than being asserted.

Saturation is reported rather than hidden. On the Shackleton rim the ankle is at
its lower stop — maximum dorsiflexion, median −37 deg — on 31 % of loaded
frames, while the knee and hip roll never saturate at all. That is what walking
up a 13 deg grade with 29.5 deg local slopes does to an ankle, and it is why
people switchback up steep ground instead of going straight at it.

### Microgravity is a different problem entirely

No terrain and no footfalls, so none of the contact machinery applies. What
replaces it is conservation: through free flight the centre of mass holds the
velocity the wall gave it and the body holds its angular momentum, and the
pelvis is *derived* from those rather than scripted. Both packets hold the
**pelvis** at constant velocity, which is the wrong invariant — as shipped, the
breaststroke clip's COM leaves its line by 26 mm, about 1066 N appearing from
nowhere, so the "futile" swimming actually propels the robot. Conserving COM and
angular momentum instead brings that to **0.000 N**, the pelvis recoils against
each stroke, and the robot goes nowhere. Which is the point.

## Motion: current state

`sim/Gait.js` is a **trajectory generator, not a clip player**. Cadence and step
length fall out of gravity:

```
step period    T = 2π√(L/g) / 4      pendulum quarter-swing of the leg
capture point  x = v√(L/g)           where the foot must land to arrest v
```

At one sixth g the pendulum is **2.46× slower** and the capture point 2.46×
further — the readout's `vs 1 g` figure is exactly √(9.81/1.62), computed rather
than typed.

## Motion: the pipeline this is a placeholder for

Retargeted motion replaces `Gait.evaluate()` with a sampled trajectory. The
interface is deliberately identical so the app does not care which drives it.

```
video ──▶ GEM-X (demo_soma_onnx.py)  ──▶ SOMA pose
      ──▶ retarget to G1/H1/Go2       ──▶ joint trajectories
      ──▶ MuJoCo tracking per field   ──▶ gravity-conditioned motion
      ──▶ bake to JSON                ──▶ this app
```

- `vendor/GEM-X` — cloned. Apple Silicon is supported via ONNX Runtime + CoreML
  (`docs/INSTALL_MACOS.md`, `scripts/setup_mac.sh`), so no CUDA is needed. **Not
  yet set up** — the models are a ~5 GB download.
- `vendor/unitree_rl_gym` — cloned; source of the H1 and Go2 descriptions.
- MuJoCo 3.11 and torch 2.13 are already installed in the `kalarisena` venv.

## The feet go on the ground, toe first

Two bugs hid each other, and together they read as "the feet go under the
ground".

`plant()` sampled the terrain **once**, under the robot's root, and lifted the
body so the lowest foot met that one height. At full stride a foot is half a
metre from the root, and on a real DEM the ground under the leading foot and
under the pelvis differ by centimetres — sometimes tens of them. Stepping onto
rising ground therefore drove the whole foot through the surface.

It also measured the foot with `Box3.setFromObject`, a **world-axis-aligned**
box. The moment an ankle pitches or rolls, that box's `min.y` is a corner of a
box containing the foot, not a point on the foot — so the more the foot tilted,
the further below the real sole it sat, and the robot was lifted off the ground
by the same rotation that was supposed to plant it.

`src/sim/Footing.js` works with the sole's real contact points instead — for the
G1, the four contact spheres the URDF actually declares (heel at x = −0.05, toe
at x = +0.12, sole plane z = −0.035) — and samples the ground under each one:

```
lift = max over contact points of ( ground(p) − p.y )
```

That is the smallest rigid lift for which no contact point is below the ground
and at least one is exactly on it. Penetration is impossible by construction,
and the support point is whichever is genuinely lowest — which is what lets the
toe land before the heel.

### The ankle ran the wrong way through stance

The old trajectory started near flat at touchdown and dorsiflexed steadily to
−0.26 rad, so the toe **rose** while the body passed over the foot. That drives
the heel edge down for the whole of stance and gives no push-off at all.

`left_ankle_pitch_joint` has axis +y in a frame with +x at the toe, so positive
is plantarflexion. A step is now a roll from one end of the sole to the other:

| phase | ankle | what it is |
|---|---|---|
| touchdown | **+0.24** | toe-down, so the toe pair is the lowest thing on the robot |
| stance 0–0.20 | +0.24 → 0 | the heel settles |
| stance 0.20–0.70 | 0 → −0.26 | the shank rotates over a planted sole |
| stance 0.70–1.0 | −0.26 → **+0.44** | toe-off; the toe is last to leave |
| swing 0–0.30 | +0.44 → −0.34 | toe up hard, clearing |
| swing 0.72–1.0 | −0.34 → +0.24 | point the toe for the next strike |

C1 at both handoffs, and inside the URDF's own `[-0.87267, +0.5236]` with
margin. `ankleRoll` — never driven before — now conforms to the lateral ground
slope, clamped to the hardware's ±0.2618, which is what stops one edge of the
sole spearing in on rough ground.

```bash
node tools/check_footing.mjs http://localhost:5173
```

measures it rather than asserting it: zero millimetres of penetration on every
field, and toe-first on the overwhelming majority of landings.

## The ISS module is a real corridor

`public/env/iss_corridor.glb` is exported from `Corridor.blend` by
`tools/build_interior.sh`: a **43.8 × 7.2 × 4.0 m** shell, 37,805 triangles,
loaded **in metres with no rescale**. An interior earns its place by giving the
scene a known size to compare the robot against, and fitting it to an arbitrary
target length — which the old station asset was — throws exactly that away.

Three things it exposed:

- The ISS heading came from `env.sunAz`, which pointed the run 110° across the
  tube. The robot swam out through the wall within seconds and finished the
  traverse alone against the stars, which is the one thing the interior exists
  to prevent. The corridor's own axis decides now.
- Every camera shot is written for open ground; `establish` stands 8.5 m to the
  side. Indoors that is through a wall, and since the shell is closed, what you
  saw was the *outside* of the module. `Stage.setInterior()` clamps the solved
  camera position into the cross-section.
- The shell's normals face out, so with the default `FrontSide` the ceiling and
  far wall were backface-culled and the starfield showed straight through the
  module.

The 40 area lamps in the blend are not exported — forty real-time lights is not
something to hand a browser — and are rebuilt at a fraction of the count from
the layout measured into `public/env/iss_corridor_lights.json`.

## The butterfly stroke

### What can actually be driven

Before writing it, the question was checked rather than assumed. The complete
set of ways a joint can be commanded in this repository is `setJointValue(rad)`
on a urdf-loader joint — a **rendering** call. There is no `unitree_sdk2`, no
CycloneDDS, no `LowCmd`/`LowState`, no ROS, no serial or UDP transport, and no
torque or PD-gain interface anywhere in the tree. MuJoCo is not installed in
`.venv`. `Butterfly.hardwareReadiness()` answers this from the tree as it is,
not from a README that can go stale.

**This cannot move a real G1, and nothing here tries to.**

### The stroke

`src/sim/Butterfly.js`. 19 joints: both shoulders (pitch, roll, yaw) and elbows,
`waist_pitch`, and both hips (pitch, roll), knees and ankle pitches.

Both arms do the same thing at the same time — that is what makes it butterfly
and not front crawl. The mirror is geometric: roll is about the URDF's x axis
and yaw about z, so those flip sign between left and right; **shoulder pitch and
the elbow are about y, the axis a sagittal mirror leaves alone, so they do
not**. The hand-rolled version this replaces negated the right elbow, which bent
the two elbows in opposite directions through the pull, and drove the left elbow
to −1.35 rad against a −1.0472 limit, so one arm silently clamped and the other
did not.

Keyframes are interpolated with **cubic Hermite using central-difference
tangents**, C1 across the whole cycle including the wrap — a straight lerp gives
a velocity that jumps at every key, which on hardware is a torque spike at every
key.

### Safety

| | |
|---|---|
| position | clamped to 90 % of each joint's URDF range about its midpoint |
| velocity | 25 % of the URDF velocity limit |
| acceleration | 12 rad/s², uniform (the URDF declares no accel limit) |
| amplitude | interpolates NEUTRAL → stroke, so 0 is a still streamlined hold |
| ramp | 0 → target over 6 s, smoothstepped |
| e-stop | latching; decays to neutral over 0.8 s. **Escape** or **space** |

Amplitude and frequency are only safe *together* — acceleration goes as
amplitude × frequency², so the constructor refuses a pair outside the envelope
rather than leaving it to a `validate()` the caller might not run.

```bash
node tools/check_butterfly.mjs --amplitude 0.35 --frequency 0.35
```

prints the full per-joint trajectory table — commanded range, fraction of limit,
peak velocity and acceleration — and checks position, velocity, acceleration,
sample-to-sample continuity, arm symmetry, the ramp and the e-stop. At the
default 0.35 amplitude the ceiling is **0.438 Hz, bound by acceleration**; the
default runs at 0.35 Hz to leave headroom.

## Credits

Robot descriptions: Unitree Robotics (`unitree_rl_gym`), used under their
licence. GEM-X: NVlabs.
