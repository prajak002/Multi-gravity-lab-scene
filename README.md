# Multi-Gravity Arena

Unitree **G1**, **H1** and **Go2** locomotion across **Moon**, **Mars**, **ISS** and
**Earth** — a web arena with a lobby that enters from the left and a camera that
moves between shots rather than cutting.

Borrows the environment/rendering work from `multi-gravity-lab` and the
retargeting pipeline from `kalarisena`.

## Run

```bash
npm install
npm run dev            # http://localhost:5173
npm run shots          # headless capture of all six states; fails on any console error
```

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

## Credits

Robot descriptions: Unitree Robotics (`unitree_rl_gym`), used under their
licence. GEM-X: NVlabs.
