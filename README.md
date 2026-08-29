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
| `/arena.html` | **Forty places across three bodies**, each with real NASA terrain, an A/B packet comparison and three gravity-driven motions |

```bash
node tools/check_gravity.mjs   http://localhost:5173   # asserts the lanes diverge
node tools/check_all_modes.mjs http://localhost:5173   # all 36 robot x field x motion
node tools/check_arena.mjs     http://localhost:4199   # the A/B viewer, against dist/
node tools/check_contact.mjs   http://localhost:4199   # nothing inside the ground or a wall
node tools/check_sites.mjs                             # the two catalogues agree
```

## The arena: three bodies, forty places, four motions

`/arena.html` is a three-level picker, because the question has three parts.
**Which body** decides the gravity, **which place** decides the ground, and
**which motion** decides what the robot is trying to do there.

| | places | terrain | motions available |
|---|---|---|---|
| **Moon** | 16 | LOLA / LOLA+Kaguya, 5–60 m/px | A/B packets on six of them, plus lope, bound and trip everywhere |
| **Mars** | 16 | HiRISE Gale at 1 m/px, HRSC+MOLA elsewhere | as above |
| **ISS** | 8 modules | none — the place is a pressurised volume | the two microgravity scenarios |

The Moon and Mars lists carry the Apollo sites, the rover landing sites, the
Artemis south-polar candidates and the named features the original six each
came from. Every one is a real window of published NASA/USGS topography,
fetched by `pipeline/dem.py` and labelled with how much of it was actually
measured.

## Bounding, loping, and falling over

The complaint this answers is a fair one: for all the gravity in the physics,
the arena looked much the same on the Moon as on Mars. Every motion in it was a
**walk**, and a walk is the worst possible way to show what gravity does,
because a walking robot keeps a foot on the ground almost all the time and
gravity only gets the small fraction of the cycle that is flight. Watch the
Apollo film and the crews are not walking: they lope, they bound, they hang,
and they fall over.

`src/sim/Ballistic.js` generates three motions from the field strength and the
URDF, and `tools/build_motions.mjs` solves them against the real terrain.
Nothing about them is authored — given `g`, everything below follows.

### What actually limits a jump is not what you would guess

Two limits compete through a push-off, and they bind at opposite ends of it:

- **Force.** Two knees at the URDF's own `effort="139"` N·m. Through the crouch
  the leg is folded and the moment arm is long, so the available vertical force
  is modest; as the leg straightens the arm collapses and the force goes up
  without bound.
- **Speed.** The same joints are rated `velocity="20"` rad/s, and the leg
  extends by `|dL/dknee|` metres per radian — which goes to **zero** as the leg
  straightens.

Integrated up the extension against the real chain:

| | take-off | apex | hang | duty | on the ground | bound by |
|---|---|---|---|---|---|---|
| Earth | 1.90 m/s | 0.183 m | 0.39 s | 0.59 | 59 % | knee torque |
| Mars | 1.94 m/s | 0.508 m | 1.05 s | 0.34 | 34 % | knee speed |
| **Moon** | **1.96 m/s** | **1.181 m** | **2.41 s** | **0.18** | **18 %** | knee speed |

Off Earth it is the **speed** limit that binds, and joint speed does not care
about gravity — so the take-off is the same to within 3 % on all three bodies
and every other number in that table is the field. A G1 is 1.32 m tall; on the
Moon it clears very nearly its own height and stays up for two and a half
seconds. Earth is the exception and the interesting one: at 1 g the machine
spends the whole push fighting its own weight and runs out of **torque** first.

The apex ratio is 2.32 against a gravity ratio of 3.721/1.625 = 2.29. It is not
exactly 2.29 because the small force-limited part of the push does respond to
`g`, and that residual is the signature of a real machine rather than a
projectile.

### Three things a generated hop got wrong before it got them right

Each was found by measuring the clip rather than looking at it, and each shows
up as a joint rate no hardware could follow.

**The swing parameter restarted three times per stride.** A hop has three
phases — crouch, flight, absorb — and in a lope the trailing foot is off the
ground for all of them. Driven by the phase-local parameter it travelled from
one foothold to the next during the crouch, snapped back at take-off and did it
again: **3787 deg/s at the hip**, all of it at phase boundaries. A foot in the
air for the whole hop needs a parameter that spans the whole hop.

**The arc landed at the height it took off from.** The ballistic term was
measured from the take-off foothold, but the landing footholds are a stride
further along and on a slope that is somewhere else — so the pelvis teleported
at the touchdown frame, **191 mm on Malapert Massif, 203 on Hadley**, which is
six metres per second in a single frame. The arc now rides a baseline running
from one support height to the other; apex and hang are untouched, and at the
end of flight the ballistic term is exactly zero, so the body arrives at
standing height over the new footholds with nothing left to jump.

**Seating the foot per frame put a step in the target.** Correcting for what
the ankle could not conform to is right, but done per frame it switches on at
the instant a foot becomes loaded. The foothold is seated once, at plan time,
so the swing arc ends exactly where the stance begins.

**And you cannot squat as deep on a hill.** Absorbing a landing folds the leg
over a planted foot, which is dorsiflexion — but on a grade the ankle has
already spent part of its range getting the sole onto the slope. Asking for the
full crouch anyway does not produce a deeper crouch, it produces an ankle on its
stop with the heel driven into the hill. The absorb is now bounded by the range
actually left, which took the Copernicus wall from 64 mm to 25.

### Where the hardware, not the solver, runs out

A generated motion is solved against the terrain the same way a packet is, and
it is seated on its own contact spheres afterwards for the same reason: the
foothold is planned under a level sole, the ankle then clamps to its URDF
limits, and a foot that cannot conform tips onto an edge and sits *higher*.
Adding that pass took the steep lunar sites from 69.5 mm of sole into the hill
on Malapert Massif to 0.0 mm.

It does not fix everything, and it should not. On the **Copernicus** terraced
wall a two-foot bound lands with `ankle_roll` at exactly its -0.2618 rad stop
and the knee fully extended, on **100 % of loaded frames**, with 22.8 mm of
sole in the hill. The cross-slope is simply steeper than the 15 degrees the G1's
ankle can conform to, so no seating pass can put the sole flat. Gale Crater, on
the same solver, saturates on 0 %.

That number is measured and reported rather than iterated away — it is the same
limit that makes people traverse a steep face instead of attacking it square
on, and the arena says so in the panel when it happens.

### Low gravity makes you SLOW

The result people find hardest to believe, and the reason the Apollo crews
loped rather than ran. Forward acceleration comes from friction, and friction
comes from weight:

```
v_max <= mu * g * t_stance
```

`t_stance` is a property of the machine and does not change, so the speed
ceiling falls with `g` directly: **2.41 m/s on Earth, 0.89 on Mars, 0.39 on the
Moon.** One sixth gravity does not make you fast. It hands you a two-and-a-half
second flight phase and almost nothing to push with.

Note what cancels. Range is `v_max x t_flight`, and `v_max` goes as `g` while
`t_flight` goes as `1/g` — so the stride comes out **0.93 m on all three
bodies**, the same structural cancellation the walking page runs into with
stride at fixed Froude number.

### Falling over is slow, and that is the whole point

Toppling is an inverted pendulum about the planted toe, so the time to go over
scales as `1/sqrt(g)`: **0.93 s on Earth against 2.28 s on the Moon.** The
interesting part of a low-gravity fall is not that it looks slow, it is that it
buys over a second of extra warning — which is what the crews used to get a
hand or a foot down, and why they fell so gracefully when they could not. The
`TRIP + RECOVER` motion catches a toe on the terrain and drives the pitch on
that clock rather than on the clip's.

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
| Push-off physics, hop / lope / trip | `src/sim/Ballistic.js` |
| Ground under two side-by-side runs | `src/terrain/LaneField.js` |
| The place catalogue, all three bodies | `tools/sites.mjs` |
| Motions baked onto a site | `tools/build_motions.mjs` |

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

## The fourteen packet scenes

The A/B comparison, which is a different thing from the forty places. Six lunar
motion problems, six Martian, two in microgravity — each a different problem,
not the same walk on different ground. Every one plays on a real NASA/USGS
terrain window and carries both models: **A = WorldVLA**, **B = PragyaSpace**.
The other twenty-six places have no packet, and offer the generated motions
instead.

```bash
.venv/bin/python pipeline/dem.py site          # fetch all 32 terrain windows
node tools/build_all.mjs                       # retarget the 14 packet scenes
node tools/build_motions.mjs                   # generate lope/bound/trip everywhere
node tools/build_index.mjs                     # write the picker's manifest
node tools/audit_packets.mjs                   # what the source packets contain
```

### The patches are rectangular, and turned to face the walk

A traverse is a line, not a disc: the robot spends its whole clip going one way
and a few metres either side of it. A square patch therefore buys most of its
pixels for ground nobody visits, and the arena paid for that twice — once in
download and once in the cap that kept the square affordable, which held **Gale
Crater to a 128 m window** of the best DEM on Mars.

The grid is now **1024 x 512** with its long axis turned onto the scenario's own
bearing. The rotation is applied in the azimuthal-equidistant frame, which is
true to scale in every direction through the site, so turning the grid costs
nothing — unlike rotating a simple-cylindrical window. Everything downstream
keeps working in grid coordinates, where `+x` simply *is* the direction of
travel, and `grid_bearing_deg` in each sidecar ties that back to the compass.

The bearing cannot be known before the window is chosen, because it is derived
from the slope of whichever window the scan settles on — so the scan runs first
on a small square window, and the export is the second pass.

| | before | after |
|---|---|---|
| Shackleton rim | 640 m square, 128 posts | **1280 x 640 m**, 256 x 128 posts |
| Gale Crater | 128 m square, 128 posts | **1200 x 600 m**, 1200 x 600 posts |
| lunar mid-latitude | 2048 m square, 34 posts | **4096 x 2048 m**, 69 x 34 posts |
| Mars global blend | 2048 m square, 10 posts | **4096 x 2048 m**, 20 x 10 posts |

Every site roughly doubles the real observation in it along the axis that
matters, and Gale goes from a courtyard to a kilometre.

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

The grid is a fixed number of samples wide whatever the source, so the only
honest statement of information content is how many **source posts** the patch
spans. It is now reported per axis — `dem_samples_across` along the traverse and
`dem_samples_short` across it — and the picker prints the first of them on every
place's button, because otherwise a 1 m/px HiRISE site and a 200 m/px
global-blend site look equally authoritative in a list.

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

## Nothing is inside the ground any more

Measured, per scene, against the surface the viewer actually draws:
`tools/check_contact.mjs` samples every vertex of every mesh of both robots at
90 instants and compares it with `Arena.ground()` — the same height function
the terrain mesh is built from.

| | worst penetration |
|---|---|
| before | **270 mm** (Medusae Fossae), 228 (Ganges), 201 (Shackleton) — knees and hips buried alongside the feet |
| after | **0 mm** on all fourteen scenes |

Four separate faults, and the largest of them was not in the solver at all.

### The side-by-side offset put the robots on ground they were never solved for

Every foothold in a clip is solved against `SiteField` at the position the
packet walks, and the clip's root height carries that solution frame by frame.
Side-by-side mode then slid each robot **0.9 m sideways** and kept that height.
On terrain with any relief, 0.9 m across is easily a couple of decimetres up or
down — hence the whole machine sunk, not a foot clipping.

The diagnosis was the contrast: superimposed, where the offset is zero, the
same scenes measured 7–19 mm. The offset was the entire story, and every
contact number in the scene files was nevertheless perfect, because the solver
had done its job and the *viewer* had then moved the robot.

It corrupted the comparison as well as the contact: WorldVLA and PragyaSpace
are supposed to differ only in how they move, and in lanes 1.8 m apart they
were also meeting different rocks.

`src/terrain/LaneField.js` moves the **ground** with the robot instead. Each
lane is drawn on its own copy of the same heightfield, translated by that
lane's offset, so a robot stands on exactly the surface its footholds were
planted in — correct by construction rather than by correction, and both models
now walk over identical ground. Two copies abutting would leave a step down the
middle of the frame, so they are blended into one continuous height function:

```
h(p) = Σ w_k(p)·field(p − o_k)  +  (1 − Σ w_k)·field(p)
```

`w_k` is 1 over the corridor its robot walks and falls to 0 by the midline, so
under either robot the sum collapses to that one lane and the height is exact.
One implementation, used by both the mesh builder and the contact guard — the
same discipline `SiteField` enforces between the renderer and the retargeter.

The corridor is a **polyline**, not the chord between the traverse's ends. A
packet traverse is straight in net direction but wanders on the way, and
PragyaSpace's run on Ganges Chasma strays 0.633 m from its chord: with a
straight corridor the lane weight there fell to 0.157, the drawn ground was
80 mm from the solved surface, and the foot went 54 mm into it.

### The drawn sole is not the sole that was planned

The retargeter plans against what the URDF says the robot collides with — four
contact spheres on a plane at `z = -0.035` — and plants them 4 mm into the
regolith on purpose. The shell that is actually **drawn** extends past those
spheres, which is the 7–19 mm that remained with the lanes superimposed.

`Footing.visibleSole()` measures the mesh's own extent, and `Arena._seat()`
lifts by whatever it takes for nothing visible to be under the ground. Two
point sets, because they fail in different places:

- **Convex-hull vertices.** The lowest point of a rigid mesh under any
  orientation is a vertex of its hull, which catches a foot pitched hard
  toe-down — where the lowest thing in the *world* is the front edge of the
  toe, nowhere near the sole plane in the foot's own frame. Missing that left a
  swing foot 64 mm inside a hillside on Ganges. 39,000 mesh vertices reduce to
  a few dozen hull ones.
- **A band across the sole.** A boulder can rise between two hull vertices and
  touch the flat, which left a *loaded* foot 16 mm into a rock on Aristarchus.

The guard reads loaded feet only. Hoisting the body every time a swinging foot
passes over a rock would make the pelvis bob — and would erase the low foot
clearance that is one of the two behaviours the page exists to compare.

### The swing arc's terrain floor spent the whole step ramping

`retarget.mjs` already lifted the swing foot over whatever sits between two
footholds, weighted to zero at both ends so the target does not jump at contact
transitions — a real constraint, since that jump was worth 1485 deg/s at the
knee. But the weight was `sin(pi*u)`, which at 15 % into the swing is only
**0.45**, so through the first and last sixth of every step the floor was at
less than half strength.

A trapezoid holds it at full strength across the middle 70 % and ramps over the
outer 15 % at each end. The ends are what the rate budget cares about and they
are still C1 into the foothold, just over a shorter, deliberately chosen
distance. Rebuilt across all 24 clips: **every contact metric identical** —
penetration, slip and position error to the digit — and the over-rate fraction
went *down* on 17 of them.

### The ISS panels were placed from the pelvis, not the robot

The launch wall and the far bulkhead sat at a fixed offset from the clip's own
start and capture points. A pelvis is not the robot: through the push-off the
feet reach back well past it and through the reach the hands go well forward.
On BrakeGap the pair swept x from −0.07 to 4.16 against panels at 0.28 and
3.82 — **through both walls, by about a third of a metre each**. Taking the
robots' swept bounds puts each panel where the furthest part of either robot
actually arrives, so the push-off lands on the wall and the bulkhead stops the
clip that hits it.

## The ISS tab is eight real modules

There is no terrain to download for the ISS, so the "place" is a pressurised
element and what makes it a place is its **size**. Each of the eight is drawn at
its published pressurised dimensions — Destiny 8.53 x 4.27 m, Kibo 11.19 x 4.4,
Cupola 1.5 x 2.95 — at scale, not fitted to the clip. That is the whole point of
an interior: it gives the eye a known measurement to put beside a 1.32 m robot,
and a shell fitted to an arbitrary target length throws exactly that away.

They are **built rather than loaded**. `env/iss_corridor.glb` is one specific
43.8 m run, and rescaling it to stand in for a 6.87 m laboratory would put the
same lie back in by another route.

The scenario is identical in every module; how much room there is to be wrong in
is not. A push that overshoots by half a metre is a caught handrail in Kibo and
a collision in Cupola.

The camera clamp had to learn the same lesson twice. Every shot is written for
open ground, so indoors the solved position is inside a wall — but clamping into
the *shell* radius is not enough, because the rack faces down each side are what
a camera actually collides with, and standing 1.68 m off the axis of a 4.27 m
module rendered as a flat grey wall filling the frame. The camera is clamped
into the free corridor between the racks, and the standoff it asked for is spent
**along the tube**, which is the only axis with room in it.

## The exported corridor

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
